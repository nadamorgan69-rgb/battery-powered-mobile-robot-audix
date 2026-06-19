#pragma once

#include <WString.h>

namespace app {

void initConsoleSink();
void consoleWrite(const char* text);
void consolePrintln(const char* text = "");
void consolePrintf(const char* format, ...);
bool consoleDequeueLine(String& line_out);
void consoleSnapshotHistory(void (*callback)(const char*, void*), void* context);

}  // namespace app
