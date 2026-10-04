// ====================================================================
//  App: Viewer
//
//  Owns the `open` shell command. The kernel provides file rendering.
// ====================================================================
#include "../../../core_api.h"
#include <string.h>

static bool s_active = false;
static void viewer_on_disconnect(void) { s_active = false; }

static bool viewer_open(void) {
  s_active = true;
  core_clear_screen();
  Serial.println(F("\033[36m== VIEWER ==\033[0m"));
  Serial.println(F("open <file>    |    / = leave viewer"));
  core_prompt_refresh();
  return true;
}


static bool viewer_on_line(char* line) {
  if (!s_active) return false;
  if (!line[0]) { core_prompt_refresh(); return true; }
  if (strncmp(line, "open", 4) != 0 || (line[4] && line[4] != ' ')) return false;
  char* arg = line + 4;
  while (*arg == ' ') arg++;
  core_file_view(arg);
  return true;
}

static const AppHooks viewer_hooks = {
  "viewer", "text / FTP-B91 viewer",
  viewer_on_disconnect, viewer_open, nullptr, viewer_on_line,
  nullptr, 0, nullptr
};

static const AppRegistrar viewer_auto_register(&viewer_hooks);

