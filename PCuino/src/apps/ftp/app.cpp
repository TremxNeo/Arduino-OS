// ====================================================================
//  App: FTP / File Manager
//
//  Owns the transfer vocabulary: put and get. The core shell provides
//  cd, ls and rm itself and they keep working while this app is open.
// ====================================================================
#include "../../../core_api.h"
#include <string.h>

static bool s_active = false;
static bool s_streaming = false;

static void ftp_on_disconnect(void) { s_active = false; s_streaming = false; }

static bool ftp_open(void) {
  s_active = true;
  core_clear_screen();
  Serial.println(F("\033[36m== FTP / FILESYSTEM ==\033[0m"));
  Serial.println(F("put get    |    / = leave   (cd ls rm work too)"));
  core_prompt_refresh();
  return true;
}


static bool ftp_stream(char* data, uint8_t len, bool finish) {
  if (!s_active) return false;
  if (finish) {
    if (!s_streaming) return false;
    s_streaming = false;
    if (len) core_transfer_stream_chunk(data, len);  // the tail typed since the last full chunk
    core_transfer_stream_end();
    return true;
  }

  if (s_streaming) {
    core_transfer_stream_chunk(data, len);
    return true;
  }

  // Only claim a long line when it is actually the FTP `put <name> <text>` form.
  if (len < 5 || strncmp(data, "put ", 4) != 0) return false;
  char* name = data + 4;
  char* content = strchr(name, ' ');
  if (!content) return false;
  *content++ = 0;
  while (*content == ' ') content++;
  if (!*name || !*content) return false;

  core_transfer_stream_start(name, content);  // reports its own errors; keeps swallowing the line
  s_streaming = true;
  return true;
}

static bool ftp_on_line(char* line) {
  if (!s_active) return false;
  if (!line[0]) { core_prompt_refresh(); return true; }

  // Parse without modifying `line`: when it is not ours the core shell parses it again.
  const char* arg = strchr(line, ' ');
  size_t cl = arg ? (size_t)(arg - line) : strlen(line);
  if (arg) while (*arg == ' ') arg++;
  else arg = "";

  if (cl == 3 && !strncmp(line, "get", 3)) { core_transfer_download(arg); return true; }
  if (cl == 3 && !strncmp(line, "put", 3)) { core_transfer_upload(arg); return true; }
  return false;  // cd / ls / rm / help ...: the core shell handles them
}

static const AppHooks ftp_hooks = {
  "ftp", "FTP / filesystem",
  ftp_on_disconnect, ftp_open, nullptr, ftp_on_line,
  nullptr, 0, ftp_stream
};

static const AppRegistrar ftp_auto_register(&ftp_hooks);

