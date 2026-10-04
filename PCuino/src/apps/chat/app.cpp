// ====================================================================
//  App: Chat — modular peer messaging over the Audio FTP link
// ====================================================================
#include "../../../core_api.h"
#include <string.h>

// Chat wire id. All chat traffic uses the generic APP:<id>:<data> API.
#define CHAT_ID 'c'

// Compact generic-app commands.
// J = join, L = leave, N = sender was told peer is not in chat,
// M:<text> = chat message.
enum ChatTxKind : uint8_t {
  CHAT_TX_NONE = 0,
  CHAT_TX_JOIN,
  CHAT_TX_LEAVE,
  CHAT_TX_MSG,
  CHAT_TX_NOCHAT
};

static bool s_active = false;
static ChatTxKind s_txKinds[2] = { CHAT_TX_NONE, CHAT_TX_NONE };
static uint8_t s_txHead = 0;
static uint8_t s_txCount = 0;

static bool chat_queue(const char* data, ChatTxKind kind) {
  if (!core_app_send(CHAT_ID, data)) return false;
  if (s_txCount < 2) {
    s_txKinds[(s_txHead + s_txCount) & 1] = kind;
    s_txCount++;
  }
  return true;
}

static bool chat_queue_p(const char* fmt, ChatTxKind kind, const char* arg) {
  if (!core_app_send_p(CHAT_ID, fmt, arg)) return false;
  if (s_txCount < 2) {
    s_txKinds[(s_txHead + s_txCount) & 1] = kind;
    s_txCount++;
  }
  return true;
}

static ChatTxKind chat_tx_done_kind() {
  if (!s_txCount) return CHAT_TX_NONE;
  ChatTxKind kind = s_txKinds[s_txHead];
  s_txKinds[s_txHead] = CHAT_TX_NONE;
  s_txHead = (s_txHead + 1) & 1;
  s_txCount--;
  return kind;
}

static void chat_reset_tx_tracking() {
  s_txKinds[0] = s_txKinds[1] = CHAT_TX_NONE;
  s_txHead = 0;
  s_txCount = 0;
}

// Typed text lives in the core's line editor; redraw it so incoming
// messages never wipe a half-typed line.
static void chat_prompt(void) {
  core_clear_line();
  Serial.print(F("\033[35m> \033[0m"));  // magenta prompt
  Serial.write((const uint8_t*)core_input_buffer(), core_input_len());
}

static void chat_enter(void) {
  core_clear_screen();
  Serial.print(F("\033[36m== CHAT with "));
  Serial.print(core_peer_name());
  Serial.println(F(" (/ = leave) ==\033[0m"));
  chat_prompt();
}

static void chat_print_peer(const char* msg) {
  core_clear_line();
  Serial.print(F("\033[36m[peer] \033[0m"));
  Serial.println(msg);
  chat_prompt();
}

static void chat_on_disconnect(void) {
  // Tell the peer only if the link is still up (leaving online); on link loss
  // the core has already cleared the session and nothing can be sent.
  if (s_active && core_online()) {
    // Best effort during disconnect; the core is about to tear down the
    // session, so failure here is harmless.
    chat_queue("L", CHAT_TX_LEAVE);
  }
  s_active = false;
  chat_reset_tx_tracking();
}

static void chat_note_in_ui(const __FlashStringHelper* s) {
  // Notice without leaving chat (never call core_print_prompt here)
  core_clear_line();
  Serial.print(F("\033[33m[Notice] \033[0m"));
  Serial.println(s);
  chat_prompt();
}

static void chat_peer_note(const __FlashStringHelper* tail) {
  if (!s_active) {
    core_notice_peer(tail);
    return;
  }
  core_clear_line();
  Serial.print(F("\033[33m[Notice] \033[0m"));
  Serial.print(core_peer_name());
  Serial.println(tail);
  chat_prompt();
}

static void chat_on_frame(uint8_t op, const char* args) {
  if (op != CORE_OP_APP || !args || !args[0]) return;

  if (args[0] == 'J' && args[1] == 0) {
    chat_peer_note(F(" has joined chat"));
    return;
  }
  if (args[0] == 'L' && args[1] == 0) {
    chat_peer_note(F(" has left chat"));
    return;
  }
  if (args[0] == 'N' && args[1] == 0) {
    // Peer is not in chat — stay in chat UI, only tell the sender.
    if (s_active) chat_note_in_ui(F("Peer is not in chat."));
    return;
  }
  if (args[0] == 'M' && args[1] == ':') {
    const char* msg = args + 2;
    if (s_active) {
      chat_print_peer(msg);
    } else {
      // Not in chat: notice on FTP UI and tell the sender (we are not in chat).
      core_notice(F("Chat message (open chat to reply)."));
      // The notice is still useful even if the tiny TX queue is full.
      chat_queue("N", CHAT_TX_NOCHAT);
      core_prompt_refresh();
    }
  }
}

static bool chat_open(void) {
  if (!core_online()) {
    core_notice(F("Chat requires online."));
    return false;
  }
  if (s_active) return true;
  // Do not enter chat unless the JOIN actually made it into the TX queue.
  // The queue is tiny, so pretending the peer saw JOIN creates a split-brain
  // chat state when another frame is already waiting to transmit.
  if (!chat_queue("J", CHAT_TX_JOIN)) {
    core_notice(F("Chat busy - try again."));
    return false;
  }
  s_active = true;
  chat_enter();
  return true;
}

static void chat_leave(void) {
  if (!s_active) return;
  // Change local state only after LEAVE was accepted by the TX queue.
  // If the queue is full, remain in chat so the user does not silently
  // desynchronise from the peer.
  if (!chat_queue("L", CHAT_TX_LEAVE)) {
    chat_note_in_ui(F("Chat busy - leave not sent."));
    return;
  }
  s_active = false;
  core_app_close();
  core_print_prompt();  // back to the shell prompt
}

static bool chat_on_line(char* line) {
  // Not active: chat is only launched via the app is launched directly by typing "chat"
  if (!s_active) return false;

  // The shell already moved to a new line after Enter: go back up so the
  // typed line is replaced (not duplicated) by [me] / the fresh prompt.
  Serial.print(F("\033[1A"));

  // Active: empty redraws prompt; "exit" leaves
  if (!line[0]) {
    chat_prompt();
    return true;
  }

  // Otherwise the line is a message: echo it only if it was actually queued
  core_clear_line();
  if (chat_queue_p(PSTR("M:%s"), CHAT_TX_MSG, line)) {
    Serial.print(F("\033[32m[me] \033[0m"));
    Serial.println(line);
  } else {
    core_err(F("TX queue full - not sent."));
  }
  chat_prompt();
  return true;
}

static void chat_on_tx_done(uint8_t op, bool ok) {
  if (op != CORE_OP_APP) return;
  ChatTxKind kind = chat_tx_done_kind();
  if (!ok && kind == CHAT_TX_MSG && s_active)
    chat_note_in_ui(F("Message not delivered."));
}


static const AppHooks chat_hooks = {
  "chat",
  "peer messaging",
  chat_on_disconnect,
  chat_open,
  chat_on_frame,
  chat_on_line,
  chat_on_tx_done,
  CHAT_ID,
  nullptr
};

static const AppRegistrar chat_auto_register(&chat_hooks);


bool chat_active(void) {
  return s_active;
}

