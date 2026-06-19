#include <Arduino.h>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>

#include "bench_runtime.hpp"
#include "command_interface.hpp"
#include "config.hpp"
#include "console_sink.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "odometry.hpp"
#include "shared_state.hpp"
#include "stepper_bench.hpp"

namespace app {

namespace {

constexpr std::size_t kCommandBufferSize = 128U;

StaticSemaphore_t g_command_mutex_buffer;
SemaphoreHandle_t g_command_mutex = nullptr;

void ensureCommandMutex() {
    if (g_command_mutex == nullptr) {
        g_command_mutex = xSemaphoreCreateMutexStatic(&g_command_mutex_buffer);
    }
}

bool equalsIgnoreCase(const char* lhs, const char* rhs) {
    if (lhs == nullptr || rhs == nullptr) {
        return false;
    }

    while (*lhs != '\0' && *rhs != '\0') {
        if (std::tolower(static_cast<unsigned char>(*lhs)) !=
            std::tolower(static_cast<unsigned char>(*rhs))) {
            return false;
        }
        ++lhs;
        ++rhs;
    }

    return (*lhs == '\0') && (*rhs == '\0');
}

float clampUnit(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

float clampRange(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

bool parseFloatToken(const char* token, float& value_out) {
    if (token == nullptr) {
        return false;
    }

    char* end_ptr = nullptr;
    value_out = std::strtof(token, &end_ptr);
    return (end_ptr != token) && (*end_ptr == '\0');
}

bool parsePwmToken(const char* token, float& value_out) {
    if (equalsIgnoreCase(token, "max") || equalsIgnoreCase(token, "+max")) {
        value_out = 1.0f;
        return true;
    }
    if (equalsIgnoreCase(token, "-max")) {
        value_out = -1.0f;
        return true;
    }
    return parseFloatToken(token, value_out);
}

bool parseDurationMs(const char* token, std::uint32_t& duration_ms_out) {
    float duration_seconds = 0.0f;
    if (!parseFloatToken(token, duration_seconds) || duration_seconds <= 0.0f) {
        return false;
    }

    duration_ms_out = static_cast<std::uint32_t>(duration_seconds * 1000.0f);
    return duration_ms_out > 0U;
}

bool parseHoldMs(const char* token, std::uint32_t& hold_ms_out) {
    float hold_ms = 0.0f;
    if (!parseFloatToken(token, hold_ms) || hold_ms <= 0.0f) {
        return false;
    }

    hold_ms_out = static_cast<std::uint32_t>(hold_ms);
    return hold_ms_out > 0U;
}

bool parsePositiveRateHz(const char* token, float& rate_hz_out) {
    if (!parseFloatToken(token, rate_hz_out)) {
        return false;
    }
    return rate_hz_out > 0.0f;
}

bool parseWheelToken(const char* token, WheelIndex& wheel_out) {
    if (equalsIgnoreCase(token, "fl")) {
        wheel_out = kFrontLeft;
        return true;
    }
    if (equalsIgnoreCase(token, "fr")) {
        wheel_out = kFrontRight;
        return true;
    }
    if (equalsIgnoreCase(token, "rl")) {
        wheel_out = kRearLeft;
        return true;
    }
    if (equalsIgnoreCase(token, "rr")) {
        wheel_out = kRearRight;
        return true;
    }
    return false;
}

const char* wheelName(std::size_t wheel) {
    static constexpr const char* kNames[kWheelCount] = {"fl", "fr", "rl", "rr"};
    return kNames[wheel];
}

const char* pidGroupName(PidGainGroup group) {
    switch (group) {
        case PidGainGroup::kWheel:
            return "wheel";
        case PidGainGroup::kX:
            return "x";
        case PidGainGroup::kY:
            return "y";
        case PidGainGroup::kYaw:
            return "yaw";
    }
    return "unknown";
}

bool parsePidGroupToken(const char* token, PidGainGroup& group_out) {
    if (equalsIgnoreCase(token, "wheel") || equalsIgnoreCase(token, "rpm")) {
        group_out = PidGainGroup::kWheel;
        return true;
    }
    if (equalsIgnoreCase(token, "x")) {
        group_out = PidGainGroup::kX;
        return true;
    }
    if (equalsIgnoreCase(token, "y")) {
        group_out = PidGainGroup::kY;
        return true;
    }
    if (equalsIgnoreCase(token, "yaw") || equalsIgnoreCase(token, "z")) {
        group_out = PidGainGroup::kYaw;
        return true;
    }
    return false;
}

float rpmToRadPerSec(float rpm) {
    return rpm * (2.0f * kPi / 60.0f);
}

std::uint8_t wheelMaskFor(WheelIndex wheel) {
    return static_cast<std::uint8_t>(1U << static_cast<std::size_t>(wheel));
}

const char* wheelMaskName(std::uint8_t mask) {
    static constexpr std::uint8_t kAllWheelMask = 0x0FU;
    if (mask == kAllWheelMask) {
        return "all";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        if (mask == (1U << wheel)) {
            return wheelName(wheel);
        }
    }
    return "mixed";
}

void printModeLine() {
    consolePrintf("[mode] transport=%s subsystem=%s\n", firmwareTransport(), firmwareSubsystem());
}

void printPidState() {
    const PidTuningState tuning = getPidTuningState();
    consolePrintf(
        "[pid] wheel=(kp=%.4f ki=%.4f kd=%.4f) x=(kp=%.4f ki=%.4f kd=%.4f) y=(kp=%.4f ki=%.4f kd=%.4f) yaw=(kp=%.4f ki=%.4f kd=%.4f)\n",
        tuning.wheel.kp,
        tuning.wheel.ki,
        tuning.wheel.kd,
        tuning.x.kp,
        tuning.x.ki,
        tuning.x.kd,
        tuning.y.kp,
        tuning.y.ki,
        tuning.y.kd,
        tuning.yaw.kp,
        tuning.yaw.ki,
        tuning.yaw.kd);
}

void printHelp() {
    printModeLine();
    consolePrintln("Available commands:");
#if defined(APP_WIFI_DASHBOARD) && defined(APP_STEPPER_BENCH)
    consolePrintln("  status");
    consolePrintln("  quiet");
    consolePrintln("  rtos on|off");
    consolePrintln("  stepper status");
    consolePrintln("  stepper enable on|off");
    consolePrintln("  stepper jog up|down <rate_hz> <duration_s>");
    consolePrintln("  stepper home <rate_hz> <timeout_s>");
    consolePrintln("  stepper stop");
#elif defined(APP_WIFI_DASHBOARD)
    consolePrintln("  status");
    consolePrintln("  i2cscan");
    consolePrintln("  enable on|off");
    consolePrintln("  stop");
    consolePrintln("  quiet");
    consolePrintln("  imu on|off|cal|zero|reset");
    consolePrintln("  yawhold on|off|zero");
    consolePrintln("  encoders on|off|once|zero|reset");
    consolePrintln("  rtos on|off");
    consolePrintln("  pidx <speed_mps> <duration_s>");
    consolePrintln("  pidy <speed_mps> <duration_s>");
    consolePrintln("  pidyaw <speed_rad_s> <duration_s>");
    consolePrintln("  forward <cm> OR <speed_mps> <duration_s>");
    consolePrintln("  backward <cm>");
    consolePrintln("  strafe <speed_mps> <duration_s>");
    consolePrintln("  rotate <deg> OR <speed_rad_s> <duration_s>");
    consolePrintln("  forwardcm <cm> | backwardcm <cm> | ccwdeg <deg> | cwdeg <deg>");
    consolePrintln("  motor <fl|fr|rl|rr|all> <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  motors all <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  rpm <fl|fr|rl|rr|all> <rpm> <duration_s>");
    consolePrintln("  rpms all <rpm> <duration_s>");
    consolePrintln("  pid show");
    consolePrintln("  pid wheel|x|y|yaw|z <kp> <ki> <kd>");
#if defined(APP_JOYSTICK_DASHBOARD)
    consolePrintln("  drive <vx_mps> <vy_mps> <wz_rad_s> <hold_ms>");
#endif
#elif defined(APP_SERIAL_PID_BENCH)
    consolePrintln("  status");
    consolePrintln("  enable on|off");
    consolePrintln("  stop");
    consolePrintln("  quiet");
    consolePrintln("  imu on|off|cal|zero|reset");
    consolePrintln("  yawhold on|off|zero");
    consolePrintln("  encoders on|off|once|zero|reset");
    consolePrintln("  rtos on|off");
    consolePrintln("  pidx <speed_mps> <duration_s>");
    consolePrintln("  pidy <speed_mps> <duration_s>");
    consolePrintln("  pidyaw <speed_rad_s> <duration_s>");
    consolePrintln("  forward <cm> OR <speed_mps> <duration_s>");
    consolePrintln("  backward <cm>");
    consolePrintln("  strafe <speed_mps> <duration_s>");
    consolePrintln("  rotate <deg> OR <speed_rad_s> <duration_s>");
    consolePrintln("  forwardcm <cm> | backwardcm <cm> | ccwdeg <deg> | cwdeg <deg>");
    consolePrintln("  motor <fl|fr|rl|rr|all> <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  motors all <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  rpm <fl|fr|rl|rr|all> <rpm> <duration_s>");
    consolePrintln("  rpms all <rpm> <duration_s>");
    consolePrintln("  pid show");
    consolePrintln("  pid wheel|x|y|yaw|z <kp> <ki> <kd>");
#elif defined(APP_SERIAL_STEPPER_BENCH)
    consolePrintln("  status");
    consolePrintln("  switch");
    consolePrintln("  enable on|off");
    consolePrintln("  jog up|down <rate_hz> <duration_s>");
    consolePrintln("  home <rate_hz> <timeout_s>");
    consolePrintln("  stop");
    consolePrintln("  quiet");
#elif defined(APP_SERIAL_IMU_ENCODER_BENCH)
    consolePrintln("  status");
    consolePrintln("  i2cscan");
    consolePrintln("  imu on|off|cal|zero|reset");
    consolePrintln("  encoders on|off|once|zero|reset");
    consolePrintln("  rtos on|off");
    consolePrintln("  stop");
    consolePrintln("  quiet");
#else
    consolePrintln("  help");
    consolePrintln("  status");
    consolePrintln("  i2cscan");
    consolePrintln("  enable on|off");
    consolePrintln("  stop");
    consolePrintln("  quiet");
    consolePrintln("  imu on|off|cal|zero|reset");
    consolePrintln("  yawhold on|off|zero");
    consolePrintln("  encoders on|off|once|zero|reset");
    consolePrintln("  rtos on|off");
    consolePrintln("  pidx <speed_mps> <duration_s>");
    consolePrintln("  pidy <speed_mps> <duration_s>");
    consolePrintln("  pidyaw <speed_rad_s> <duration_s>");
    consolePrintln("  forward <cm> OR <speed_mps> <duration_s>");
    consolePrintln("  backward <cm>");
    consolePrintln("  strafe <speed_mps> <duration_s>");
    consolePrintln("  rotate <deg> OR <speed_rad_s> <duration_s>");
    consolePrintln("  forwardcm <cm> | backwardcm <cm> | ccwdeg <deg> | cwdeg <deg>");
    consolePrintln("  motor <fl|fr|rl|rr|all> <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  motors all <pwm_-1_to_1|max> <duration_s>");
    consolePrintln("  rpm <fl|fr|rl|rr|all> <rpm> <duration_s>");
    consolePrintln("  rpms all <rpm> <duration_s>");
    consolePrintln("  stepper status");
    consolePrintln("  stepper enable on|off");
    consolePrintln("  stepper jog up|down <rate_hz> <duration_s>");
    consolePrintln("  stepper home <rate_hz> <timeout_s>");
    consolePrintln("  stepper stop");
#endif
}

void printStatus() {
    printModeLine();
#if defined(APP_STEPPER_BENCH)
    const BenchRuntimeSnapshot bench = getBenchRuntimeSnapshot();
    consolePrintf(
        "[status] stepperMode=yes imuHealthy=skipped bench=(imu=%s enc=%s rtos=%s motion=%s move=%s pwm=%s rpm=%s)\n",
        bench.imu_stream_enabled ? "on" : "off",
        bench.encoder_stream_enabled ? "on" : "off",
        bench.rtos_stream_enabled ? "on" : "off",
        bench.timed_motion_active ? "on" : "off",
        bench.target_move_active ? "on" : "off",
        bench.manual_override_active ? "on" : "off",
        bench.wheel_speed_override_active ? "on" : "off");
    printStepperBenchStatus();
#else
    const CommandState command = getCommandState();
    const WheelState wheel_state = getWheelState();
    const OdometryState odometry = getOdometryState();
    const IMUState imu = getImuState();
    const SensorState sensors = getSensorState();
    const BenchRuntimeSnapshot bench = getBenchRuntimeSnapshot();
    const HeadingControlState heading = getHeadingControlState();
    const bool imu_healthy = imuDriver().isHealthy();

    consolePrintf(
        "[status] enabled=%s imuHealthy=%s cmd=(%.3f, %.3f, %.3f) limit=%s odom=(x=%.3f y=%.3f theta=%.3f vx=%.3f vy=%.3f wz=%.3f) imu=(yaw=%.3f gz=%.3f) heading=(hold=%s phase=%u target=%.1fdeg err=%.1fdeg wz=%.3f stable=%.0fms) bench=(motion=%s move=%s pwm=%s rpm=%s imu=%s enc=%s rtos=%s)\n",
        command.robot_enabled ? "on" : "off",
        imu_healthy ? "yes" : "no",
        command.cmd_vx,
        command.cmd_vy,
        command.cmd_wz,
        sensors.limit_switch_pressed ? "pressed" : "released",
        odometry.x,
        odometry.y,
        odometry.theta,
        odometry.vx,
        odometry.vy,
        odometry.wtheta,
        imu.orientation_z,
        imu.gyro_z,
        heading.hold_enabled ? "on" : "off",
        static_cast<unsigned>(heading.phase),
        heading.target_yaw * 180.0f / kPi,
        heading.yaw_error * 180.0f / kPi,
        heading.command_wz,
        heading.stable_ms,
        bench.timed_motion_active ? "on" : "off",
        bench.target_move_active ? "on" : "off",
        bench.manual_override_active ? "on" : "off",
        bench.wheel_speed_override_active ? "on" : "off",
        bench.imu_stream_enabled ? "on" : "off",
        bench.encoder_stream_enabled ? "on" : "off",
        bench.rtos_stream_enabled ? "on" : "off");

    consolePrintf(
        "[wheels] fl(count=%ld target=%.3f meas=%.3f pwm=%.3f) fr(count=%ld target=%.3f meas=%.3f pwm=%.3f) rl(count=%ld target=%.3f meas=%.3f pwm=%.3f) rr(count=%ld target=%.3f meas=%.3f pwm=%.3f)\n",
        static_cast<long>(wheel_state.encoder_counts[kFrontLeft]),
        wheel_state.target_w_rad_s[kFrontLeft],
        wheel_state.measured_w_rad_s[kFrontLeft],
        wheel_state.pwm_output[kFrontLeft],
        static_cast<long>(wheel_state.encoder_counts[kFrontRight]),
        wheel_state.target_w_rad_s[kFrontRight],
        wheel_state.measured_w_rad_s[kFrontRight],
        wheel_state.pwm_output[kFrontRight],
        static_cast<long>(wheel_state.encoder_counts[kRearLeft]),
        wheel_state.target_w_rad_s[kRearLeft],
        wheel_state.measured_w_rad_s[kRearLeft],
        wheel_state.pwm_output[kRearLeft],
        static_cast<long>(wheel_state.encoder_counts[kRearRight]),
        wheel_state.target_w_rad_s[kRearRight],
        wheel_state.measured_w_rad_s[kRearRight],
        wheel_state.pwm_output[kRearRight]);

    if (stepperBenchAvailable()) {
        printStepperBenchStatus();
    }
    printPidState();
#endif
}

void runI2cScan() {
    consolePrintf(
        "Scanning I2C bus on SDA=%d SCL=%d ...\n",
        IMU_SDA_PIN,
        IMU_SCL_PIN);

    bool found_any = false;
    for (std::uint8_t address = 0x03U; address <= 0x77U; ++address) {
        Wire.beginTransmission(address);
        const std::uint8_t error = Wire.endTransmission();
        if (error == 0U) {
            consolePrintf("Found I2C device at 0x%02X\n", address);
            found_any = true;
        }
    }

    if (!found_any) {
        consolePrintln("No I2C devices found.");
    } else {
        consolePrintln("I2C scan complete.");
    }
}

void handleToggle(const char* label, const char* state_token, void (*setter)(bool)) {
    if (equalsIgnoreCase(state_token, "on")) {
        setter(true);
        consolePrintf("%s enabled.\n", label);
    } else if (equalsIgnoreCase(state_token, "off")) {
        setter(false);
        consolePrintf("%s disabled.\n", label);
    } else {
        consolePrintf("Usage: %s on|off\n", label);
    }
}

void turnOffBenchStreams() {
    setBenchImuStreamEnabled(false);
    setBenchEncoderStreamEnabled(false);
    setBenchRtosStreamEnabled(false);
}

bool baseQuietForImuCalibration() {
    const CommandState command = getCommandState();
    const WheelState wheel_state = getWheelState();
    if (command.robot_enabled &&
        (std::fabs(command.cmd_vx) > 0.001f ||
         std::fabs(command.cmd_vy) > 0.001f ||
         std::fabs(command.cmd_wz) > 0.001f)) {
        return false;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        if (std::fabs(wheel_state.pwm_output[wheel]) > 0.001f ||
            std::fabs(wheel_state.measured_w_rad_s[wheel]) > WHEEL_STOP_EPSILON_RAD_S) {
            return false;
        }
    }
    return true;
}

void zeroImuYawSnapshot() {
    imuDriver().zeroYaw();
    IMUState imu = getImuState();
    imu.orientation_z = 0.0f;
    setImuState(imu);
}

void handleImuCommand(char* context) {
#if defined(APP_STEPPER_BENCH)
    consolePrintln("[imu] unavailable in stepper firmware.");
#else
    const char* subcommand = strtok_r(nullptr, " \t", &context);
    if (equalsIgnoreCase(subcommand, "cal") || equalsIgnoreCase(subcommand, "calibrate")) {
        if (!baseQuietForImuCalibration()) {
            consolePrintln("[imu] calibration refused: stop motors and keep robot still first.");
            return;
        }
        const bool ok = imuDriver().calibrateGyroBias();
        consolePrintf("[imu] gyro bias calibration %s.\n", ok ? "ok" : "failed");
        return;
    }
    if (equalsIgnoreCase(subcommand, "zero")) {
        zeroImuYawSnapshot();
        consolePrintln("[imu] yaw zeroed.");
        return;
    }
    if (equalsIgnoreCase(subcommand, "reset")) {
        if (!baseQuietForImuCalibration()) {
            consolePrintln("[imu] reset refused: stop motors and keep robot still first.");
            return;
        }
        const bool ok = imuDriver().calibrateGyroBias();
        zeroImuYawSnapshot();
        consolePrintf("[imu] reset %s; yaw zeroed.\n", ok ? "ok" : "calibration failed");
        return;
    }

    handleToggle("imu", subcommand, setBenchImuStreamEnabled);
#endif
}

void printEncoderSnapshot() {
    const WheelState wheel_state = getWheelState();
    const OdometryState odometry = getOdometryState();
    consolePrintf(
        "[enc] fl=(count=%ld w=%.3f pwm=%.3f) fr=(count=%ld w=%.3f pwm=%.3f) rl=(count=%ld w=%.3f pwm=%.3f) rr=(count=%ld w=%.3f pwm=%.3f) odom=(vx=%.3f vy=%.3f wz=%.3f)\n",
        static_cast<long>(wheel_state.encoder_counts[kFrontLeft]),
        wheel_state.measured_w_rad_s[kFrontLeft],
        wheel_state.pwm_output[kFrontLeft],
        static_cast<long>(wheel_state.encoder_counts[kFrontRight]),
        wheel_state.measured_w_rad_s[kFrontRight],
        wheel_state.pwm_output[kFrontRight],
        static_cast<long>(wheel_state.encoder_counts[kRearLeft]),
        wheel_state.measured_w_rad_s[kRearLeft],
        wheel_state.pwm_output[kRearLeft],
        static_cast<long>(wheel_state.encoder_counts[kRearRight]),
        wheel_state.measured_w_rad_s[kRearRight],
        wheel_state.pwm_output[kRearRight],
        odometry.vx,
        odometry.vy,
        odometry.wtheta);
}

void zeroEncodersAndOdometry() {
#if defined(APP_BASE_BENCH)
    emergencyStopBench(millis());
#else
    setCommandVelocity(0.0f, 0.0f, 0.0f, millis());
    setRobotEnabled(false);
#endif
    resetEncoders();
    odometryTracker().reset();

    const float zero_wheels[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    const std::int32_t zero_counts[kWheelCount] = {0, 0, 0, 0};
    setWheelTargets(zero_wheels);
    setMeasuredWheelState(zero_wheels, zero_counts);
    setPwmOutputs(zero_wheels);
    setOdometryState(OdometryState{});
    consolePrintln("[enc] zeroed counts and odometry; robot disabled.");
}

bool parsePwmAndDuration(char*& context, float& pwm_out, std::uint32_t& duration_ms_out) {
    if (!parsePwmToken(strtok_r(nullptr, " \t", &context), pwm_out)) {
        return false;
    }
    if (!parseDurationMs(strtok_r(nullptr, " \t", &context), duration_ms_out)) {
        return false;
    }
    pwm_out = clampUnit(pwm_out);
    return true;
}

void scheduleAllMotorOverrideAndPrint(float pwm, std::uint32_t duration_ms) {
    scheduleAllWheelOverride(pwm, duration_ms, millis());
    consolePrintf(
        "Manual motor override scheduled: all pwm=%.3f for %.2f s; robot auto-enabled.\n",
        pwm,
        static_cast<double>(duration_ms) / 1000.0);
}

void scheduleSingleMotorOverrideAndPrint(WheelIndex wheel, float pwm, std::uint32_t duration_ms) {
    scheduleSingleWheelOverride(wheel, pwm, duration_ms, millis());
    consolePrintf(
        "Manual motor override scheduled: %s pwm=%.3f for %.2f s; robot auto-enabled.\n",
        wheelName(static_cast<std::size_t>(wheel)),
        pwm,
        static_cast<double>(duration_ms) / 1000.0);
}

void scheduleWheelRpmOverrideAndPrint(std::uint8_t wheel_mask,
                                      float rpm,
                                      std::uint32_t duration_ms) {
    float target_rad_s[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float target_rad_s_value = rpmToRadPerSec(rpm);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        if ((wheel_mask & (1U << wheel)) != 0U) {
            target_rad_s[wheel] = target_rad_s_value;
        }
    }

    scheduleWheelSpeedOverride(target_rad_s, wheel_mask, rpm, duration_ms, millis());
    consolePrintf(
        "[rpm] scheduled wheel=%s target=%.1frpm duration=%.2fs\n",
        wheelMaskName(wheel_mask),
        rpm,
        static_cast<double>(duration_ms) / 1000.0);
}

void scheduleMotion(float vx, float vy, float wz, const char* duration_token) {
    std::uint32_t duration_ms = 0U;
    if (!parseDurationMs(duration_token, duration_ms)) {
        consolePrintln("Invalid duration. Use seconds, for example 2.0");
        return;
    }

    scheduleTimedMotionCommand(vx, vy, wz, duration_ms, millis());
    consolePrintf(
        "Timed motion scheduled: vx=%.3f vy=%.3f wz=%.3f for %.2f s\n",
        vx,
        vy,
        wz,
        static_cast<double>(duration_ms) / 1000.0);
}

std::uint32_t targetMoveTimeoutMs(float target_delta, float command_speed) {
    const float abs_speed = std::fabs(command_speed);
    if (abs_speed <= 0.0f) {
        return BENCH_TARGET_TIMEOUT_EXTRA_MS;
    }
    const auto travel_ms = static_cast<std::uint32_t>((std::fabs(target_delta) / abs_speed) * 1000.0f);
    return travel_ms + BENCH_TARGET_TIMEOUT_EXTRA_MS;
}

void scheduleLinearTargetMoveCm(float target_cm) {
    const float target_m = target_cm / 100.0f;
    if (std::fabs(target_m) < 0.001f) {
        consolePrintln("Invalid distance. Use centimeters, for example forward 20");
        return;
    }

    const float command_speed = (target_m >= 0.0f)
        ? BENCH_TARGET_LINEAR_SPEED_MPS
        : -BENCH_TARGET_LINEAR_SPEED_MPS;
    const std::uint32_t timeout_ms = targetMoveTimeoutMs(target_m, command_speed);
    scheduleTargetMove(BenchTargetMoveAxis::kX, target_m, command_speed, timeout_ms, millis());
    consolePrintf(
        "[move] scheduled axis=x target=%.1fcm speed=%.3fmps timeout=%.2fs\n",
        target_cm,
        command_speed,
        static_cast<double>(timeout_ms) / 1000.0);
}

void scheduleYawTargetMoveDeg(float target_deg) {
    const float target_rad = target_deg * kPi / 180.0f;
    if (std::fabs(target_rad) < 0.001f) {
        consolePrintln("Invalid angle. Use degrees, for example rotate 90");
        return;
    }

    const float command_speed = (target_rad >= 0.0f)
        ? BENCH_TARGET_YAW_SPEED_RAD_S
        : -BENCH_TARGET_YAW_SPEED_RAD_S;
    const std::uint32_t timeout_ms = targetMoveTimeoutMs(target_rad, command_speed);
    scheduleTargetMove(BenchTargetMoveAxis::kYaw, target_rad, command_speed, timeout_ms, millis());
    consolePrintf(
        "[move] scheduled axis=yaw target=%.1fdeg speed=%.3fradps timeout=%.2fs\n",
        target_deg,
        command_speed,
        static_cast<double>(timeout_ms) / 1000.0);
}

void scheduleDrive(float vx, float vy, float wz, const char* hold_token) {
    std::uint32_t hold_ms = 0U;
    if (!parseHoldMs(hold_token, hold_ms)) {
        consolePrintln("Usage: drive <vx_mps> <vy_mps> <wz_rad_s> <hold_ms>");
        return;
    }

    hold_ms = std::min(hold_ms, BENCH_DRIVE_MAX_HOLD_MS);
    vx = clampRange(vx, -BENCH_DRIVE_MAX_VX_MPS, BENCH_DRIVE_MAX_VX_MPS);
    vy = clampRange(vy, -BENCH_DRIVE_MAX_VY_MPS, BENCH_DRIVE_MAX_VY_MPS);
    wz = clampRange(wz, -BENCH_DRIVE_MAX_WZ_RAD_S, BENCH_DRIVE_MAX_WZ_RAD_S);
    scheduleTimedMotionCommand(vx, vy, wz, hold_ms, millis());
    consolePrintf("[joy] drive vx=%.3f vy=%.3f wz=%.3f hold=%lu ms\n", vx, vy, wz, static_cast<unsigned long>(hold_ms));
}

void handlePidCommand(char* context) {
    const char* subcommand = strtok_r(nullptr, " \t", &context);
    if (equalsIgnoreCase(subcommand, "show")) {
        printPidState();
        return;
    }

    PidGainGroup group = PidGainGroup::kWheel;
    if (!parsePidGroupToken(subcommand, group)) {
        consolePrintln("Usage: pid show OR pid wheel|x|y|yaw|z <kp> <ki> <kd>");
        return;
    }

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parseFloatToken(strtok_r(nullptr, " \t", &context), kp) ||
        !parseFloatToken(strtok_r(nullptr, " \t", &context), ki) ||
        !parseFloatToken(strtok_r(nullptr, " \t", &context), kd)) {
        consolePrintln("Usage: pid wheel|x|y|yaw|z <kp> <ki> <kd>");
        return;
    }

    kp = clampRange(kp, 0.0f, 100.0f);
    ki = clampRange(ki, 0.0f, 100.0f);
    kd = clampRange(kd, 0.0f, 100.0f);
    setPidGains(group, kp, ki, kd);
    consolePrintf("[pid] set %s kp=%.4f ki=%.4f kd=%.4f\n", pidGroupName(group), kp, ki, kd);
    printPidState();
}

void printBaseUnavailable() {
    consolePrintf("[base] unavailable in transport=%s subsystem=%s firmware.\n", firmwareTransport(), firmwareSubsystem());
}

void printStepperAliasUnavailable() {
    consolePrintf("[stepper] unavailable in transport=%s subsystem=%s firmware.\n", firmwareTransport(), firmwareSubsystem());
}

void handleStepperJog(char* context) {
    const char* direction_token = strtok_r(nullptr, " \t", &context);
    const char* rate_token = strtok_r(nullptr, " \t", &context);
    const char* duration_token = strtok_r(nullptr, " \t", &context);
    float rate_hz = 0.0f;
    std::uint32_t duration_ms = 0U;

    if ((!equalsIgnoreCase(direction_token, "up") && !equalsIgnoreCase(direction_token, "down")) ||
        !parsePositiveRateHz(rate_token, rate_hz) ||
        !parseDurationMs(duration_token, duration_ms)) {
        consolePrintln("Usage: stepper jog up|down <rate_hz> <duration_s>");
        return;
    }

    scheduleStepperBenchJog(equalsIgnoreCase(direction_token, "up"), rate_hz, duration_ms);
}

void handleStepperHome(char* context) {
    const char* rate_token = strtok_r(nullptr, " \t", &context);
    const char* timeout_token = strtok_r(nullptr, " \t", &context);
    float rate_hz = 0.0f;
    std::uint32_t timeout_ms = 0U;

    if (!parsePositiveRateHz(rate_token, rate_hz) || !parseDurationMs(timeout_token, timeout_ms)) {
        consolePrintln("Usage: stepper home <rate_hz> <timeout_s>");
        return;
    }

    scheduleStepperBenchHome(rate_hz, timeout_ms);
}

void handleStepperCommand(char* context) {
    const char* subcommand = strtok_r(nullptr, " \t", &context);
    if (subcommand == nullptr) {
        consolePrintln("Usage: stepper status|enable on|off|jog up|down <rate_hz> <duration_s>|home <rate_hz> <timeout_s>|stop");
        return;
    }

    if (equalsIgnoreCase(subcommand, "status")) {
        printStepperBenchStatus();
        return;
    }

    if (equalsIgnoreCase(subcommand, "enable")) {
        const char* state_token = strtok_r(nullptr, " \t", &context);
        if (equalsIgnoreCase(state_token, "on")) {
            setStepperBenchEnabled(true);
        } else if (equalsIgnoreCase(state_token, "off")) {
            setStepperBenchEnabled(false);
        } else {
            consolePrintln("Usage: stepper enable on|off");
        }
        return;
    }

    if (equalsIgnoreCase(subcommand, "stop")) {
        stopStepperBench(true);
        return;
    }

    if (equalsIgnoreCase(subcommand, "jog")) {
        handleStepperJog(context);
        return;
    }

    if (equalsIgnoreCase(subcommand, "home")) {
        handleStepperHome(context);
        return;
    }

    consolePrintf("Unknown stepper command: %s\n", subcommand);
    consolePrintln("Usage: stepper status|enable on|off|jog up|down <rate_hz> <duration_s>|home <rate_hz> <timeout_s>|stop");
}

void processCommandTokens(char* line) {
    char* context = nullptr;
    char* command = strtok_r(line, " \t", &context);
    if (command == nullptr) {
        return;
    }

    if (equalsIgnoreCase(command, "help")) {
        printHelp();
        return;
    }

    if (equalsIgnoreCase(command, "mode")) {
        printModeLine();
        return;
    }

    if (equalsIgnoreCase(command, "status")) {
        printStatus();
        return;
    }

    if (equalsIgnoreCase(command, "pid")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        handlePidCommand(context);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "quiet")) {
        turnOffBenchStreams();
        consolePrintln("[status] streams off.");
        return;
    }

    if (equalsIgnoreCase(command, "i2cscan")) {
#if defined(APP_STEPPER_BENCH)
        consolePrintln("[imu] unavailable in stepper firmware.");
#else
        runI2cScan();
#endif
        return;
    }

    if (equalsIgnoreCase(command, "stepper")) {
        handleStepperCommand(context);
        return;
    }

    if (equalsIgnoreCase(command, "switch")) {
#if defined(APP_STEPPER_BENCH)
        printStepperBenchStatus();
#else
        printStepperAliasUnavailable();
#endif
        return;
    }

    if (equalsIgnoreCase(command, "jog")) {
#if defined(APP_STEPPER_BENCH)
        handleStepperJog(context);
#else
        printStepperAliasUnavailable();
#endif
        return;
    }

    if (equalsIgnoreCase(command, "home")) {
#if defined(APP_STEPPER_BENCH)
        handleStepperHome(context);
#else
        printStepperAliasUnavailable();
#endif
        return;
    }

    if (equalsIgnoreCase(command, "enable")) {
#if defined(APP_STEPPER_BENCH)
        const char* state_token = strtok_r(nullptr, " \t", &context);
        if (equalsIgnoreCase(state_token, "on")) {
            setStepperBenchEnabled(true);
        } else if (equalsIgnoreCase(state_token, "off")) {
            setStepperBenchEnabled(false);
        } else {
            consolePrintln("Usage: enable on|off");
        }
#elif defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        const char* state_token = strtok_r(nullptr, " \t", &context);
        if (equalsIgnoreCase(state_token, "on")) {
            setRobotEnabled(true);
            setCommandVelocity(0.0f, 0.0f, 0.0f, millis());
            consolePrintln("Robot enabled.");
        } else if (equalsIgnoreCase(state_token, "off")) {
            emergencyStopBench(millis());
            consolePrintln("Robot disabled and stopped.");
        } else {
            consolePrintln("Usage: enable on|off");
        }
#endif
        return;
    }

    if (equalsIgnoreCase(command, "stop")) {
#if defined(APP_STEPPER_BENCH)
        stopStepperBench(true);
#elif defined(APP_SERIAL_IMU_ENCODER_BENCH)
        setBenchImuStreamEnabled(false);
        setBenchEncoderStreamEnabled(false);
        setBenchRtosStreamEnabled(false);
        consolePrintln("[status] IMU, encoder, and RTOS streams stopped.");
#else
        emergencyStopBench(millis());
        turnOffBenchStreams();
        consolePrintln("[status] emergency stop applied; streams off; motor pins low.");
#endif
        return;
    }

    if (equalsIgnoreCase(command, "imu")) {
        handleImuCommand(context);
        return;
    }

    if (equalsIgnoreCase(command, "yawhold")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        const char* state_token = strtok_r(nullptr, " \t", &context);
        if (equalsIgnoreCase(state_token, "zero")) {
            setHeadingHoldEnabled(false, millis());
            zeroImuYawSnapshot();
            setHeadingHoldEnabled(true, millis());
            consolePrintln("[heading] yaw zeroed and hold enabled at 0 deg.");
        } else if (equalsIgnoreCase(state_token, "on")) {
            setHeadingHoldEnabled(true, millis());
            consolePrintln("[heading] yaw hold enabled at current heading.");
        } else if (equalsIgnoreCase(state_token, "off")) {
            setHeadingHoldEnabled(false, millis());
            consolePrintln("[heading] yaw hold disabled.");
        } else {
            consolePrintln("Usage: yawhold on|off|zero");
        }
#endif
        return;
    }

    if (equalsIgnoreCase(command, "encoders")) {
#if defined(APP_STEPPER_BENCH)
        consolePrintln("[enc] unavailable in stepper firmware.");
#else
        const char* state_token = strtok_r(nullptr, " \t", &context);
        if (equalsIgnoreCase(state_token, "zero") || equalsIgnoreCase(state_token, "reset")) {
            zeroEncodersAndOdometry();
        } else if (equalsIgnoreCase(state_token, "once")) {
            printEncoderSnapshot();
        } else {
            handleToggle("encoders", state_token, setBenchEncoderStreamEnabled);
        }
#endif
        return;
    }

    if (equalsIgnoreCase(command, "rtos")) {
        handleToggle("rtos", strtok_r(nullptr, " \t", &context), setBenchRtosStreamEnabled);
        return;
    }

    if (equalsIgnoreCase(command, "pidx")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float speed = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), speed)) {
            consolePrintln("Usage: pidx <speed_mps> <duration_s>");
            return;
        }
        scheduleMotion(speed, 0.0f, 0.0f, strtok_r(nullptr, " \t", &context));
#endif
        return;
    }

    if (equalsIgnoreCase(command, "forward") || equalsIgnoreCase(command, "forwardcm")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float value = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), value)) {
            consolePrintln("Usage: forward <cm> OR forward <speed_mps> <duration_s>");
            return;
        }

        const char* optional_duration = strtok_r(nullptr, " \t", &context);
        if (optional_duration != nullptr && equalsIgnoreCase(command, "forward")) {
            scheduleMotion(value, 0.0f, 0.0f, optional_duration);
            return;
        }
        if (optional_duration != nullptr) {
            consolePrintln("Usage: forwardcm <cm>");
            return;
        }
        scheduleLinearTargetMoveCm(value);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "backward") || equalsIgnoreCase(command, "backwardcm")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float value = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), value)) {
            consolePrintln("Usage: backward <cm>");
            return;
        }

        const char* optional_duration = strtok_r(nullptr, " \t", &context);
        if (optional_duration != nullptr && equalsIgnoreCase(command, "backward")) {
            scheduleMotion(-std::fabs(value), 0.0f, 0.0f, optional_duration);
            return;
        }
        if (optional_duration != nullptr) {
            consolePrintln("Usage: backwardcm <cm>");
            return;
        }
        scheduleLinearTargetMoveCm(-std::fabs(value));
#endif
        return;
    }

    if (equalsIgnoreCase(command, "pidy") || equalsIgnoreCase(command, "strafe")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float speed = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), speed)) {
            consolePrintln("Usage: pidy <speed_mps> <duration_s>");
            return;
        }
        scheduleMotion(0.0f, speed, 0.0f, strtok_r(nullptr, " \t", &context));
#endif
        return;
    }

    if (equalsIgnoreCase(command, "pidyaw")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float speed = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), speed)) {
            consolePrintln("Usage: pidyaw <speed_rad_s> <duration_s>");
            return;
        }
        scheduleMotion(0.0f, 0.0f, speed, strtok_r(nullptr, " \t", &context));
#endif
        return;
    }

    if (equalsIgnoreCase(command, "rotate")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float value = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), value)) {
            consolePrintln("Usage: rotate <deg> OR rotate <speed_rad_s> <duration_s>");
            return;
        }

        const char* optional_duration = strtok_r(nullptr, " \t", &context);
        if (optional_duration != nullptr) {
            scheduleMotion(0.0f, 0.0f, value, optional_duration);
            return;
        }
        scheduleYawTargetMoveDeg(value);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "ccwdeg") || equalsIgnoreCase(command, "cwdeg")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        float value = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), value)) {
            consolePrintln("Usage: ccwdeg <deg> OR cwdeg <deg>");
            return;
        }
        if (strtok_r(nullptr, " \t", &context) != nullptr) {
            consolePrintln("Usage: ccwdeg <deg> OR cwdeg <deg>");
            return;
        }
        scheduleYawTargetMoveDeg(equalsIgnoreCase(command, "cwdeg") ? -std::fabs(value) : std::fabs(value));
#endif
        return;
    }

    if (equalsIgnoreCase(command, "drive")) {
#if defined(APP_JOYSTICK_DASHBOARD)
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), vx) ||
            !parseFloatToken(strtok_r(nullptr, " \t", &context), vy) ||
            !parseFloatToken(strtok_r(nullptr, " \t", &context), wz)) {
            consolePrintln("Usage: drive <vx_mps> <vy_mps> <wz_rad_s> <hold_ms>");
            return;
        }
        scheduleDrive(vx, vy, wz, strtok_r(nullptr, " \t", &context));
#else
        consolePrintln("[joy] drive unavailable. Flash esp32_wifi_base_joystick_dashboard.");
#endif
        return;
    }

    if (equalsIgnoreCase(command, "rpms")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        if (!equalsIgnoreCase(strtok_r(nullptr, " \t", &context), "all")) {
            consolePrintln("Usage: rpms all <rpm> <duration_s>");
            return;
        }

        float rpm = 0.0f;
        std::uint32_t duration_ms = 0U;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), rpm) ||
            !parseDurationMs(strtok_r(nullptr, " \t", &context), duration_ms)) {
            consolePrintln("Usage: rpms all <rpm> <duration_s>");
            return;
        }

        scheduleWheelRpmOverrideAndPrint(0x0FU, rpm, duration_ms);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "rpm")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        const char* wheel_token = strtok_r(nullptr, " \t", &context);
        std::uint8_t wheel_mask = 0U;
        WheelIndex wheel = kFrontLeft;

        if (equalsIgnoreCase(wheel_token, "all")) {
            wheel_mask = 0x0FU;
        } else if (parseWheelToken(wheel_token, wheel)) {
            wheel_mask = wheelMaskFor(wheel);
        } else {
            consolePrintln("Usage: rpm <fl|fr|rl|rr|all> <rpm> <duration_s>");
            return;
        }

        float rpm = 0.0f;
        std::uint32_t duration_ms = 0U;
        if (!parseFloatToken(strtok_r(nullptr, " \t", &context), rpm) ||
            !parseDurationMs(strtok_r(nullptr, " \t", &context), duration_ms)) {
            consolePrintln("Usage: rpm <fl|fr|rl|rr|all> <rpm> <duration_s>");
            return;
        }

        scheduleWheelRpmOverrideAndPrint(wheel_mask, rpm, duration_ms);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "motors")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        if (!equalsIgnoreCase(strtok_r(nullptr, " \t", &context), "all")) {
            consolePrintln("Usage: motors all <pwm_-1_to_1|max> <duration_s>");
            return;
        }

        float pwm = 0.0f;
        std::uint32_t duration_ms = 0U;
        if (!parsePwmAndDuration(context, pwm, duration_ms)) {
            consolePrintln("Usage: motors all <pwm_-1_to_1|max> <duration_s>");
            return;
        }

        scheduleAllMotorOverrideAndPrint(pwm, duration_ms);
#endif
        return;
    }

    if (equalsIgnoreCase(command, "motor")) {
#if defined(APP_STEPPER_BENCH) || defined(APP_SERIAL_IMU_ENCODER_BENCH)
        printBaseUnavailable();
#else
        const char* wheel_token = strtok_r(nullptr, " \t", &context);
        WheelIndex wheel = kFrontLeft;
        float pwm = 0.0f;
        std::uint32_t duration_ms = 0U;

        if (equalsIgnoreCase(wheel_token, "all")) {
            if (!parsePwmAndDuration(context, pwm, duration_ms)) {
                consolePrintln("Usage: motor all <pwm_-1_to_1|max> <duration_s>");
                return;
            }
            scheduleAllMotorOverrideAndPrint(pwm, duration_ms);
            return;
        }

        if (!parseWheelToken(wheel_token, wheel)) {
            consolePrintln("Usage: motor <fl|fr|rl|rr|all> <pwm_-1_to_1|max> <duration_s>");
            return;
        }
        if (!parsePwmAndDuration(context, pwm, duration_ms)) {
            consolePrintln("Usage: motor <fl|fr|rl|rr|all> <pwm_-1_to_1|max> <duration_s>");
            return;
        }

        scheduleSingleMotorOverrideAndPrint(wheel, pwm, duration_ms);
#endif
        return;
    }

    consolePrintf("Unknown command: %s\n", command);
    printHelp();
}

}  // namespace

void processBenchCommandLine(const char* line) {
    if (line == nullptr) {
        return;
    }

    ensureCommandMutex();
    xSemaphoreTake(g_command_mutex, portMAX_DELAY);

    char buffer[kCommandBufferSize] = {};
    std::strncpy(buffer, line, kCommandBufferSize - 1U);
    buffer[kCommandBufferSize - 1U] = '\0';
    processCommandTokens(buffer);

    xSemaphoreGive(g_command_mutex);
}

void commandRxTask(void*) {
    char command_buffer[kCommandBufferSize] = {};
    std::size_t command_length = 0U;

    for (;;) {
        markTaskHeartbeat(kCommandTaskHeartbeat);

        while (Serial.available() > 0) {
            const char input = static_cast<char>(Serial.read());
            if (input == '\r' || input == '\n') {
                if (command_length > 0U) {
                    command_buffer[command_length] = '\0';
                    processBenchCommandLine(command_buffer);
                    command_length = 0U;
                    command_buffer[0] = '\0';
                }
                continue;
            }

            if (command_length < (kCommandBufferSize - 1U)) {
                command_buffer[command_length++] = input;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace app
