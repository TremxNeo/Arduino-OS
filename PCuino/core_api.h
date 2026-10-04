#pragma once
// ====================================================================
//  Audio FTP OS — API for apps                                 (v2 + services)
//
//  Core owns: modem, frames, session, storage/transfer services, shell kernel.
//  An app is one AppHooks table + register function. Everything an app
//  may touch is declared in this file; nothing else is stable.
//
//  Minimal app:
//     static void hi_frame(uint8_t op, const char* data) { ... }
//     static const AppHooks hooks = { "hi", "say hi", nullptr, nullptr,
//                                     hi_open, nullptr, hi_frame };
//     void hi_register(void) { core_register_app(&hooks); }
//  Hooks you leave out (or NULL) are simply skipped.
//  See app_template.cpp for a complete, commented example.
// ====================================================================

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "config.h"

#define CORE_API_VERSION 2     // #if CORE_API_VERSION >= 2 to use the v2 calls below

// ---- Link / session -------------------------------------------------
bool        core_online(void);        // peer identified, online mode
const char* core_peer_name(void);

// ---- Sending to the peer --------------------------------------------
// Messages are queued and acknowledged by the peer; the result arrives in
// your on_tx_done hook. They return false if the queue is full (core_app_send*
// also if offline or `app_id` is invalid).
// Text is printable ASCII; whitespace runs collapse to one space.

// Generic app message (the way to add your own protocol; no core change needed).
// `app_id` = your AppHooks.id. The peer's app with the same id receives it
// in on_frame(CORE_OP_APP, data).
#define CORE_OP_APP        31  // op id seen by on_frame / on_tx_done for generic messages
#define CORE_APP_MAX_DATA  82  // max characters of data per generic message
bool core_app_send(char app_id, const char* data);
bool core_app_send_p(char app_id, const char* progmem_fmt, ...);  // PSTR("x=%u s=%s"); only %s and %u/%d

// ---- Terminal I/O (apps must not fight the shell) --------------------
void core_clear_screen(void);
void core_clear_line(void);
void core_notice(const __FlashStringHelper* s);       // "[Notice] text" (safe while an image plays)
void core_notice_peer(const __FlashStringHelper* s);  // "[Notice] <peer><text>" + prompt refresh
void core_err(const __FlashStringHelper* s);

// Prompts. Pick the right one:
void core_prompt_refresh(void);   // after ANY asynchronous print: prompt + half-typed input, no clearing
void core_print_prompt(void);     // when your app exits: the plain shell prompt (no screen clear)

// What the user has typed so far on the current line (for redraws).
uint8_t     core_input_len(void);
const char* core_input_buffer(void);   // core_input_len() chars, NOT NUL-terminated

// True while the image viewer owns the display: don't draw (core_notice is safe).
bool core_display_locked(void);

// ---- Kernel services ------------------------------------------------
// Applications own command names and UI. One compact dispatcher keeps the
// kernel boundary small on AVR while exposing capabilities, not commands.
// The core shell itself provides cd, ls and rm. Apps use these transfer services;
// each resolves a dir/ prefix, runs, and prints the prompt when it finishes.
enum : uint8_t { CORE_FS_GET = 0, CORE_FS_PUT, CORE_FS_VIEW };
bool core_fs_service(uint8_t service, const char* path);
static inline bool core_transfer_download(const char* p) { return core_fs_service(CORE_FS_GET, p); }
static inline bool core_transfer_upload(const char* p)   { return core_fs_service(CORE_FS_PUT, p); }
static inline bool core_file_view(const char* p)         { return core_fs_service(CORE_FS_VIEW, p); }

// Long `put` (a line longer than the line editor) is streamed through on_stream.
// It is the same write path as a short put: start creates/replaces the file with
// the first part, chunk appends, end reports once and prints the prompt.
// `name` may carry a dir/ prefix. All text is NUL-terminated (data[len] == 0).
// Not in the local storage dir: the core prints the error and ignores the chunks.
bool core_transfer_stream_start(const char* name, const char* initial);
bool core_transfer_stream_chunk(const char* data, uint8_t len);
bool core_transfer_stream_end(void);

// Fire-and-forget transfer encoder for RawTX. `tone_gap_ms` is the silence
// between individual tones; `chunk_gap_ms` is the pause between XDAT chunks.
bool core_transfer_tx_only_start(const char* path, uint16_t tone_gap_ms, uint16_t chunk_gap_ms);
void core_transfer_tx_only_stop(void);

// ---- App plugin interface -------------------------------------------
// Positional initialiser, in this order. Trailing members may be omitted (= NULL/0).
//
//  hook           called when                                         notes
//  ------------   -------------------------------------------------   ---------------------------
//  on_init        once at boot                                        reset state
//  on_disconnect  link lost, or user leaves online                    EVERY app is told. Drop UI and
//                                                                     pending-tx state. To tell the peer
//                                                                     check core_online() first
//  on_open        `apps <name>` typed (no other app is active)        true  = app is now ACTIVE: it owns
//                                                                            the shell and must draw its own
//                                                                            prompt and handle `exit`
//                                                                            (call core_app_close() on exit)
//                                                                     false = one-shot / failed: the shell
//                                                                            prints the prompt
//  on_frame       peer message with your id (op==CORE_OP_APP, data)   must be quick; no blocking
//  on_line        user pressed Enter while YOUR app is active         return true if you handled it
//                                                                     (print the prompt yourself when
//                                                                     done). false: the core shell tries
//                                                                     its own commands (cd ls rm help
//                                                                     apps menu online), else "Unknown".
//  on_tx_done     a message you queued was ACKed (ok) or timed out    op = CORE_OP_APP for generic app sends.
//                                                                     Core prints nothing for these.
//  on_stream      active app, typed line longer than the editor        claim it (true) to receive the rest in
//                 buffer / its end (finish==true)                      chunks (see app_ftp.cpp); the final
//                                                                     partial chunk arrives WITH finish.
struct AppHooks {
  const char* name;
  const char* blurb;
  void (*on_disconnect)(void);
  bool (*on_open)(void);
  void (*on_frame)(uint8_t op_id, const char* data);
  bool (*on_line)(char* line);
  void (*on_tx_done)(uint8_t op_id, bool ok);
  char id;
  bool (*on_stream)(char* data, uint8_t len, bool finish);
};

#define CORE_MAX_APPS 12
// Register an app with the OS. Apps normally use AppRegistrar below, so the
// app maker never needs to edit a central bundle or config file.
bool core_register_app(const AppHooks* app);

// Automatic app registration. Every .cpp in the Arduino sketch is compiled by
// the Arduino build; constructing one of these makes the OS discover the app
// without any manual wiring.
class AppRegistrar {
public:
  explicit AppRegistrar(const AppHooks* app) { core_register_app(app); }
};

// ---- Called by the core only (apps never call these) ----------------
void core_apps_disconnect(void);
void core_apps_app_frame(const char* args);                // "APP:<id>:<data>" -> app with that id
void core_apps_tx_done(uint8_t op_id, char app_id, bool ok);
bool core_apps_line(char* line);
bool core_apps_stream(char* data, uint8_t len, bool finish);

// Shell helpers for the `apps` command
bool        core_app_open(const char* name);   // find by name, call on_open
void        core_app_close(void);                // leave the active app

