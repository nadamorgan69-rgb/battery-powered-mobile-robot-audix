#include "wifi_console.hpp"

#include <Arduino.h>

#include "console_sink.hpp"

#if defined(APP_WIFI_DASHBOARD)

#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

#include "bench_runtime.hpp"
#include "command_interface.hpp"
#include "config.hpp"
#include "imu_driver.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

WebServer g_http_server(WIFI_HTTP_PORT);
WebSocketsServer g_websocket(WIFI_WS_PORT);
bool g_wifi_console_ready = false;

bool startSoftApWithRetry() {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    const IPAddress local_ip(192, 168, 4, 1);
    const IPAddress gateway(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);

    for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
        const bool ap_ok = WiFi.softAP(
            WIFI_AP_SSID,
            WIFI_AP_PASSWORD,
            WIFI_AP_CHANNEL,
            0,
            WIFI_AP_MAX_CLIENTS);
        if (ap_ok) {
            return true;
        }
        consolePrintf("[wifi] hotspot start attempt %u failed; retrying...\n", static_cast<unsigned>(attempt));
        WiFi.softAPdisconnect(true);
        delay(250);
    }
    return false;
}

const char* contentTypeForPath(const String& path) {
    if (path.endsWith(".html")) {
        return "text/html";
    }
    if (path.endsWith(".css")) {
        return "text/css";
    }
    if (path.endsWith(".js")) {
        return "application/javascript";
    }
    if (path.endsWith(".json")) {
        return "application/json";
    }
    if (path.endsWith(".svg")) {
        return "image/svg+xml";
    }
    if (path.endsWith(".ico")) {
        return "image/x-icon";
    }
    return "text/plain";
}

String normalizePath(String path) {
    if (path.isEmpty() || path == "/") {
        return "/index.html";
    }
    if (!path.startsWith("/")) {
        path = "/" + path;
    }
    return path;
}

void serveSpiffsPath(const String& request_path) {
    const String path = normalizePath(request_path);
    if (!SPIFFS.exists(path)) {
        g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        g_http_server.sendHeader("Pragma", "no-cache");
        g_http_server.sendHeader("Expires", "0");
        g_http_server.send(404, "text/plain", "File not found");
        return;
    }

    File file = SPIFFS.open(path, "r");
    if (!file) {
        g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        g_http_server.sendHeader("Pragma", "no-cache");
        g_http_server.sendHeader("Expires", "0");
        g_http_server.send(500, "text/plain", "Failed to open file");
        return;
    }

    g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    g_http_server.sendHeader("Pragma", "no-cache");
    g_http_server.sendHeader("Expires", "0");
    g_http_server.streamFile(file, contentTypeForPath(path));
    file.close();
}

void streamHistoryLine(const char* line, void* context) {
    (void)context;
    if (line == nullptr) {
        return;
    }
    g_http_server.sendContent(line);
    g_http_server.sendContent("\n");
}

void serveConsoleHistory() {
    g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    g_http_server.sendHeader("Pragma", "no-cache");
    g_http_server.sendHeader("Expires", "0");
    g_http_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    g_http_server.send(200, "text/plain", "");
    consoleSnapshotHistory(streamHistoryLine, nullptr);
    g_http_server.sendContent("");
}

void handleHttpCommand() {
    if (!g_http_server.hasArg("line")) {
        g_http_server.send(400, "text/plain", "missing line");
        return;
    }

    String command = g_http_server.arg("line");
    command.trim();
    if (command.length() == 0) {
        g_http_server.send(400, "text/plain", "empty command");
        return;
    }

    processBenchCommandLine(command.c_str());
    g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    g_http_server.send(200, "text/plain", "ok");
}

void appendJsonKeyValue(String& json, const char* key, const char* value, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":";
    json += value;
    if (comma) {
        json += ",";
    }
}

void appendJsonKeyString(String& json, const char* key, const char* value, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":\"";
    json += value;
    json += "\"";
    if (comma) {
        json += ",";
    }
}

void appendJsonKeyFloat(String& json, const char* key, float value, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":";
    json += String(value, 6);
    if (comma) {
        json += ",";
    }
}

void appendJsonKeyInt(String& json, const char* key, long value, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":";
    json += String(value);
    if (comma) {
        json += ",";
    }
}

void appendPidJson(String& json, const char* key, const PidGains& gains, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":{";
    appendJsonKeyFloat(json, "kp", gains.kp);
    appendJsonKeyFloat(json, "ki", gains.ki);
    appendJsonKeyFloat(json, "kd", gains.kd);
    appendJsonKeyInt(json, "version", static_cast<long>(gains.version), false);
    json += "}";
    if (comma) {
        json += ",";
    }
}

void appendWheelJson(String& json, const char* key, std::size_t wheel, const WheelState& wheels, bool comma = true) {
    json += "\"";
    json += key;
    json += "\":{";
    appendJsonKeyInt(json, "count", static_cast<long>(wheels.encoder_counts[wheel]));
    appendJsonKeyFloat(json, "targetRadS", wheels.target_w_rad_s[wheel]);
    appendJsonKeyFloat(json, "measuredRadS", wheels.measured_w_rad_s[wheel]);
    appendJsonKeyFloat(json, "targetRpm", wheels.target_w_rad_s[wheel] * 60.0f / (2.0f * kPi));
    appendJsonKeyFloat(json, "measuredRpm", wheels.measured_w_rad_s[wheel] * 60.0f / (2.0f * kPi));
    appendJsonKeyFloat(json, "pwm", wheels.pwm_output[wheel], false);
    json += "}";
    if (comma) {
        json += ",";
    }
}

const char* headingPhaseName(HeadingControlPhase phase) {
    switch (phase) {
        case HeadingControlPhase::kStraightMove:
            return "straight_move";
        case HeadingControlPhase::kHoldActive:
            return "hold_active";
        case HeadingControlPhase::kRotateMove:
            return "rotate_move";
        case HeadingControlPhase::kSettled:
            return "settled";
        case HeadingControlPhase::kIdle:
        default:
            return "idle";
    }
}

void serveApiState() {
    const CommandState command = getCommandState();
    const WheelState wheels = getWheelState();
    const OdometryState odom = getOdometryState();
    const IMUState imu = getImuState();
    const SensorState sensors = getSensorState();
    const BenchRuntimeSnapshot bench = getBenchRuntimeSnapshot();
    const PidTuningState pid = getPidTuningState();
    const HeadingControlState heading = getHeadingControlState();
    const std::uint32_t now_ms = millis();
    const std::uint32_t command_age_ms = now_ms - command.last_cmd_time_ms;
    const bool motion_allowed = command.robot_enabled && command_age_ms < CMD_TIMEOUT_MS;

    String json;
    json.reserve(3300);
    json += "{";
    json += "\"mode\":{";
    appendJsonKeyString(json, "transport", "wifi");
    appendJsonKeyString(json, "subsystem", firmwareSubsystem());
    appendJsonKeyInt(json, "millis", static_cast<long>(now_ms), false);
    json += "},";

    json += "\"command\":{";
    appendJsonKeyValue(json, "enabled", command.robot_enabled ? "true" : "false");
    appendJsonKeyValue(json, "motionAllowed", motion_allowed ? "true" : "false");
    appendJsonKeyInt(json, "cmdAgeMs", static_cast<long>(command_age_ms));
    appendJsonKeyFloat(json, "vx", command.cmd_vx);
    appendJsonKeyFloat(json, "vy", command.cmd_vy);
    appendJsonKeyFloat(json, "wz", command.cmd_wz, false);
    json += "},";

    json += "\"odom\":{";
    appendJsonKeyFloat(json, "x", odom.x);
    appendJsonKeyFloat(json, "y", odom.y);
    appendJsonKeyFloat(json, "theta", odom.theta);
    appendJsonKeyFloat(json, "xCm", odom.x * 100.0f);
    appendJsonKeyFloat(json, "yCm", odom.y * 100.0f);
    appendJsonKeyFloat(json, "thetaDeg", odom.theta * 180.0f / kPi);
    appendJsonKeyFloat(json, "vx", odom.vx);
    appendJsonKeyFloat(json, "vy", odom.vy);
    appendJsonKeyFloat(json, "wz", odom.wtheta, false);
    json += "},";

    json += "\"imu\":{";
    appendJsonKeyValue(json, "healthy", imuDriver().isHealthy() ? "true" : "false");
    appendJsonKeyFloat(json, "yaw", imu.orientation_z);
    appendJsonKeyFloat(json, "yawDeg", imu.orientation_z * 180.0f / kPi);
    appendJsonKeyFloat(json, "gyroX", imu.gyro_x);
    appendJsonKeyFloat(json, "gyroY", imu.gyro_y);
    appendJsonKeyFloat(json, "gyroZ", imu.gyro_z);
    appendJsonKeyFloat(json, "accelX", imu.accel_x);
    appendJsonKeyFloat(json, "accelY", imu.accel_y);
    appendJsonKeyFloat(json, "accelZ", imu.accel_z, false);
    json += "},";

    json += "\"sensors\":{";
    appendJsonKeyValue(json, "limitSwitchPressed", sensors.limit_switch_pressed ? "true" : "false", false);
    json += "},";

    json += "\"bench\":{";
    appendJsonKeyValue(json, "imuStream", bench.imu_stream_enabled ? "true" : "false");
    appendJsonKeyValue(json, "encoderStream", bench.encoder_stream_enabled ? "true" : "false");
    appendJsonKeyValue(json, "rtosStream", bench.rtos_stream_enabled ? "true" : "false");
    appendJsonKeyValue(json, "targetMoveActive", bench.target_move_active ? "true" : "false");
    appendJsonKeyValue(json, "manualPwmActive", bench.manual_override_active ? "true" : "false");
    appendJsonKeyValue(json, "rpmActive", bench.wheel_speed_override_active ? "true" : "false");
    appendJsonKeyValue(json, "headingHoldActive", bench.heading_hold_active ? "true" : "false");
    appendJsonKeyFloat(json, "headingHoldTargetDeg", bench.heading_hold_target * 180.0f / kPi);
    appendJsonKeyFloat(json, "headingHoldStableMs", bench.heading_hold_stable_ms, false);
    json += "},";

    json += "\"heading\":{";
    appendJsonKeyValue(json, "holdEnabled", heading.hold_enabled ? "true" : "false");
    appendJsonKeyValue(json, "settled", heading.settled ? "true" : "false");
    appendJsonKeyInt(json, "phase", static_cast<long>(heading.phase));
    appendJsonKeyString(json, "phaseName", headingPhaseName(heading.phase));
    appendJsonKeyFloat(json, "targetYaw", heading.target_yaw);
    appendJsonKeyFloat(json, "targetYawDeg", heading.target_yaw * 180.0f / kPi);
    appendJsonKeyFloat(json, "yawError", heading.yaw_error);
    appendJsonKeyFloat(json, "yawErrorDeg", heading.yaw_error * 180.0f / kPi);
    appendJsonKeyFloat(json, "gyroZ", heading.gyro_z);
    appendJsonKeyFloat(json, "commandWz", heading.command_wz);
    appendJsonKeyFloat(json, "stableMs", heading.stable_ms, false);
    json += "},";

    json += "\"pid\":{";
    appendPidJson(json, "wheel", pid.wheel);
    appendPidJson(json, "x", pid.x);
    appendPidJson(json, "y", pid.y);
    appendPidJson(json, "yaw", pid.yaw, false);
    json += "},";

    json += "\"wheels\":{";
    appendWheelJson(json, "fl", kFrontLeft, wheels);
    appendWheelJson(json, "fr", kFrontRight, wheels);
    appendWheelJson(json, "rl", kRearLeft, wheels);
    appendWheelJson(json, "rr", kRearRight, wheels, false);
    json += "}";
    json += "}";

    g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    g_http_server.send(200, "application/json", json);
}

void replayHistoryLine(const char* line, void* context) {
    if (line == nullptr || context == nullptr) {
        return;
    }

    const uint8_t client_id = *static_cast<uint8_t*>(context);
    g_websocket.sendTXT(client_id, line);
}

void handleWebSocketEvent(uint8_t client_id, WStype_t type, uint8_t* payload, std::size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            consoleSnapshotHistory(replayHistoryLine, &client_id);
            consolePrintf("[wifi] browser connected: client=%u\n", static_cast<unsigned>(client_id));
            break;
        }
        case WStype_DISCONNECTED:
            consolePrintf("[wifi] browser disconnected: client=%u\n", static_cast<unsigned>(client_id));
            break;
        case WStype_TEXT: {
            if (payload == nullptr || length == 0U) {
                return;
            }
            String command(reinterpret_cast<const char*>(payload), length);
            command.trim();
            if (command.length() == 0) {
                return;
            }
            processBenchCommandLine(command.c_str());
            break;
        }
        default:
            break;
    }
}

void flushPendingConsoleLines() {
    String line;
    while (consoleDequeueLine(line)) {
        g_websocket.broadcastTXT(line);
    }
}

}  // namespace

void initWifiConsole() {
    const bool ap_ok = startSoftApWithRetry();

    if (!SPIFFS.begin(true)) {
        consolePrintln("[wifi] SPIFFS mount failed.");
    }

    g_http_server.on("/", HTTP_GET, []() {
        serveSpiffsPath("/index.html");
    });
    g_http_server.on("/api/ping", HTTP_GET, []() {
        g_http_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        g_http_server.send(200, "text/plain", "ok");
    });
    g_http_server.on("/api/state", HTTP_GET, serveApiState);
    g_http_server.on("/api/history", HTTP_GET, serveConsoleHistory);
    g_http_server.on("/api/command", HTTP_GET, handleHttpCommand);
    g_http_server.onNotFound([]() {
        serveSpiffsPath(g_http_server.uri());
    });

    g_websocket.begin();
    g_websocket.onEvent(handleWebSocketEvent);
    g_http_server.begin();
    g_wifi_console_ready = true;

    if (ap_ok) {
        consolePrintf(
            "[wifi] hotspot ready: ssid=%s password=%s ip=%s\n",
            WIFI_AP_SSID,
            WIFI_AP_PASSWORD,
            WiFi.softAPIP().toString().c_str());
        consolePrintf("[wifi] hotspot channel=%u max_clients=%u mac=%s\n",
                      static_cast<unsigned>(WIFI_AP_CHANNEL),
                      static_cast<unsigned>(WIFI_AP_MAX_CLIENTS),
                      WiFi.softAPmacAddress().c_str());
        consolePrintf("[wifi] open browser: http://%s/\n", WiFi.softAPIP().toString().c_str());
        consolePrintf("[wifi] websocket bridge: ws://%s:%u/\n", WiFi.softAPIP().toString().c_str(), WIFI_WS_PORT);
    } else {
        consolePrintln("[wifi] hotspot start failed.");
    }
}

void serviceWifiConsole() {
    if (!g_wifi_console_ready) {
        return;
    }

    g_http_server.handleClient();
    g_websocket.loop();
    flushPendingConsoleLines();
}

}  // namespace app

#else

namespace app {

void initWifiConsole() {
    consolePrintln("[wifi] disabled in this firmware environment.");
}

void serviceWifiConsole() {}

}  // namespace app

#endif
