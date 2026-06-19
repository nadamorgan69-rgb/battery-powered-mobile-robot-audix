#include "console_sink.hpp"

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace app {

namespace {

constexpr std::size_t kHistoryCapacity = 120U;
constexpr std::size_t kPendingCapacity = 120U;
constexpr std::size_t kFormatBufferBytes = 512U;

StaticSemaphore_t g_console_mutex_buffer;
SemaphoreHandle_t g_console_mutex = nullptr;
String g_partial_line;
String g_history[kHistoryCapacity];
String g_pending[kPendingCapacity];
std::size_t g_history_head = 0U;
std::size_t g_history_count = 0U;
std::size_t g_pending_head = 0U;
std::size_t g_pending_count = 0U;

void ensureConsoleMutex() {
    if (g_console_mutex == nullptr) {
        g_console_mutex = xSemaphoreCreateMutexStatic(&g_console_mutex_buffer);
        g_partial_line.reserve(256U);
    }
}

void lockConsole() {
    ensureConsoleMutex();
    xSemaphoreTake(g_console_mutex, portMAX_DELAY);
}

void unlockConsole() {
    xSemaphoreGive(g_console_mutex);
}

void pushHistoryLine(const String& line) {
    const std::size_t slot = (g_history_head + g_history_count) % kHistoryCapacity;
    g_history[slot] = line;
    if (g_history_count < kHistoryCapacity) {
        ++g_history_count;
    } else {
        g_history_head = (g_history_head + 1U) % kHistoryCapacity;
    }
}

void pushPendingLine(const String& line) {
    const std::size_t slot = (g_pending_head + g_pending_count) % kPendingCapacity;
    g_pending[slot] = line;
    if (g_pending_count < kPendingCapacity) {
        ++g_pending_count;
    } else {
        g_pending[g_pending_head] = line;
        g_pending_head = (g_pending_head + 1U) % kPendingCapacity;
    }
}

void finalizeConsoleLine(const String& line) {
    pushHistoryLine(line);
    pushPendingLine(line);
}

void appendTextLocked(const char* text) {
    if (text == nullptr || *text == '\0') {
        return;
    }

    Serial.print(text);

    const char* cursor = text;
    while (*cursor != '\0') {
        const char ch = *cursor++;
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            finalizeConsoleLine(g_partial_line);
            g_partial_line = "";
        } else {
            g_partial_line += ch;
        }
    }
}

}  // namespace

void initConsoleSink() {
    ensureConsoleMutex();
}

void consoleWrite(const char* text) {
    lockConsole();
    appendTextLocked(text);
    unlockConsole();
}

void consolePrintln(const char* text) {
    lockConsole();
    if (text != nullptr && *text != '\0') {
        appendTextLocked(text);
    }
    appendTextLocked("\n");
    unlockConsole();
}

void consolePrintf(const char* format, ...) {
    if (format == nullptr) {
        return;
    }

    char buffer[kFormatBufferBytes] = {};
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    lockConsole();
    appendTextLocked(buffer);
    unlockConsole();
}

bool consoleDequeueLine(String& line_out) {
    lockConsole();
    const bool has_line = g_pending_count > 0U;
    if (has_line) {
        line_out = g_pending[g_pending_head];
        g_pending[g_pending_head] = "";
        g_pending_head = (g_pending_head + 1U) % kPendingCapacity;
        --g_pending_count;
    }
    unlockConsole();
    return has_line;
}

void consoleSnapshotHistory(void (*callback)(const char*, void*), void* context) {
    if (callback == nullptr) {
        return;
    }

    lockConsole();
    for (std::size_t index = 0; index < g_history_count; ++index) {
        const std::size_t slot = (g_history_head + index) % kHistoryCapacity;
        callback(g_history[slot].c_str(), context);
    }
    unlockConsole();
}

}  // namespace app
