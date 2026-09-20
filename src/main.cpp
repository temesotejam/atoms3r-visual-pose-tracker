#include <Arduino.h>
#include <M5Unified.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "app_config.h"
#include "camera_driver.h"
#include "marker_detector.h"
#include "marker_tracker.h"
#include "pose_estimator.h"
#include "vision_types.h"

namespace {

CameraDriver g_camera;
MarkerDetector g_detector(appcfg::kFrameWidth, appcfg::kFrameHeight);
PoseEstimator g_pose(appcfg::kFxPx, appcfg::kFyPx,
                     appcfg::kCxPx, appcfg::kCyPx,
                     appcfg::kMarkerSideM);

RectI g_lane_a{appcfg::kLaneAX, appcfg::kLaneAY,
               appcfg::kLaneAW, appcfg::kLaneAH};
RectI g_lane_b{appcfg::kLaneBX, appcfg::kLaneBY,
               appcfg::kLaneBW, appcfg::kLaneBH};

MarkerTracker g_tracker_a(appcfg::kMarkerAId, g_lane_a, g_detector, g_pose);
MarkerTracker g_tracker_b(appcfg::kMarkerBId, g_lane_b, g_detector, g_pose);

portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
ImuTelemetry g_imu;
TaskHandle_t g_imu_task = nullptr;

uint32_t g_last_telemetry_ms = 0;
uint32_t g_frame_count = 0;
uint32_t g_camera_failures = 0;
uint32_t g_max_vision_us = 0;
uint32_t g_frame_dt_us = 0;
uint64_t g_last_frame_timestamp_us = 0;

uint8_t* g_previous_gray = nullptr;
bool g_have_previous_gray = false;

const char* stateName(TrackState state) {
    switch (state) {
        case TrackState::Track: return "track";
        case TrackState::Recover: return "recover";
        default: return "acquire";
    }
}

const char* sourceName(const MarkerObservation& m) {
    if (m.flow_tracked) return "pyramid";
    if (m.decoded_this_frame) return "aruco";
    return "none";
}

void controlStep(const ImuTelemetry&) {
    // Reserved for the future controller. Vision must never block this task.
}

void imuControlTask(void*) {
    const TickType_t period_ticks =
        pdMS_TO_TICKS(appcfg::kImuControlPeriodUs / 1000);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t max_step_us = 0;
    uint32_t deadline_misses = 0;
    uint32_t loop_count = 0;

    for (;;) {
        const uint32_t t0 = micros();

        ImuTelemetry sample;
        sample.enabled = M5.Imu.isEnabled();

        if (sample.enabled) {
            M5.Imu.update();
            const auto data = M5.Imu.getImuData();
            sample.sample_timestamp_us =
                static_cast<uint64_t>(esp_timer_get_time());
            sample.ax = data.accel.x;
            sample.ay = data.accel.y;
            sample.az = data.accel.z;
            sample.gx = data.gyro.x;
            sample.gy = data.gyro.y;
            sample.gz = data.gyro.z;
        }

        const uint32_t step_us = micros() - t0;
        if (step_us > max_step_us) max_step_us = step_us;
        if (step_us > appcfg::kImuControlPeriodUs) ++deadline_misses;
        ++loop_count;

        sample.loop_count = loop_count;
        sample.deadline_misses = deadline_misses;
        sample.max_step_us = max_step_us;

        portENTER_CRITICAL(&g_imu_mux);
        g_imu = sample;
        portEXIT_CRITICAL(&g_imu_mux);

        controlStep(sample);
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

void printMarkerJson(const char* name, const MarkerObservation& m) {
    Serial.printf(
        "\"%s\":{\"valid\":%s,\"state\":\"%s\",\"source\":\"%s\","
        "\"id\":%d,\"rotation\":%d,\"hamming\":%d,\"quality\":%.3f,"
        "\"refined\":%s,\"track_sad\":%.2f,"
        "\"flow_ok\":%u,\"flow_fail\":%u,\"decode_ok\":%u,\"reacquire\":%u,"
        "\"cx_px\":%.2f,\"cy_px\":%.2f,\"side_px\":%.2f,"
        "\"image_angle_deg\":%.2f,"
        "\"constrained_x_m\":%.5f,\"constrained_y_m\":%.5f,"
        "\"constrained_z_m\":%.5f,"
        "\"perspective_asymmetry\":%.4f,\"tilt_reliable\":%s,"
        "\"x_m\":%.5f,\"y_m\":%.5f,\"z_m\":%.5f,"
        "\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,"
        "\"vision_us\":%u}",
        name,
        m.valid ? "true" : "false",
        stateName(m.state),
        sourceName(m),
        m.id, m.rotation, m.hamming, m.quality,
        m.corner_refined ? "true" : "false",
        m.track_mean_sad,
        m.flow_success_count, m.flow_fail_count,
        m.decode_success_count, m.reacquire_count,
        m.center_x_px, m.center_y_px, m.side_px,
        m.image_angle_deg,
        m.constrained_x_m, m.constrained_y_m, m.constrained_z_m,
        m.perspective_asymmetry,
        m.tilt_reliable ? "true" : "false",
        m.x_m, m.y_m, m.z_m,
        m.roll_deg, m.pitch_deg, m.yaw_deg,
        m.vision_processing_us);
}

void printTelemetry(const MarkerObservation& a,
                    const MarkerObservation& b,
                    uint32_t total_vision_us) {
    ImuTelemetry imu;
    portENTER_CRITICAL(&g_imu_mux);
    imu = g_imu;
    portEXIT_CRITICAL(&g_imu_mux);

    Serial.printf(
        "{\"t_us\":%llu,\"frame\":%u,\"frame_dt_us\":%u,"
        "\"camera_failures\":%u,"
        "\"vision_total_us\":%u,\"vision_max_us\":%u,"
        "\"motion_axis\":\"horizontal\","
        "\"imu\":{\"enabled\":%s,\"loops\":%u,\"misses\":%u,"
        "\"max_step_us\":%u,\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f},",
        static_cast<unsigned long long>(esp_timer_get_time()),
        g_frame_count, g_frame_dt_us, g_camera_failures,
        total_vision_us, g_max_vision_us,
        imu.enabled ? "true" : "false",
        imu.loop_count, imu.deadline_misses, imu.max_step_us,
        imu.ax, imu.ay, imu.az,
        imu.gx, imu.gy, imu.gz);

    printMarkerJson("marker_a", a);
    Serial.print(",");
    printMarkerJson("marker_b", b);
    Serial.println("}");
}

} // namespace

void setup() {
    Serial.begin(921600);
    delay(300);

    Serial.println();
    Serial.println("AtomS3R Visual Pose Tracker boot");
    Serial.println("Init order: camera I2C0 first, BMI270 I2C1 second");
    Serial.println("Motion model: horizontal; A upper band, B lower band");
    Serial.println("Tracking: ArUco acquire/reacquire + two-level local flow");

    if (!psramFound()) {
        Serial.println("FATAL: PSRAM not detected");
        while (true) delay(1000);
    }

    const size_t frame_bytes =
        static_cast<size_t>(appcfg::kFrameWidth) * appcfg::kFrameHeight;
    g_previous_gray = static_cast<uint8_t*>(
        heap_caps_malloc(frame_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!g_previous_gray) {
        g_previous_gray = static_cast<uint8_t*>(malloc(frame_bytes));
    }
    if (!g_previous_gray) {
        Serial.println("FATAL: previous-frame buffer allocation failed");
        while (true) delay(1000);
    }

    if (!g_detector.begin()) {
        Serial.println("FATAL: detector scratch allocation failed");
        while (true) delay(1000);
    }

    if (!g_camera.begin()) {
        Serial.printf("FATAL: camera init: %s\n", g_camera.lastError());
        while (true) delay(1000);
    }
    Serial.println("CAMERA READY: GC0308 SCCB owns I2C0");

    M5.In_I2C.setPort(I2C_NUM_1, GPIO_NUM_45, GPIO_NUM_0);
    const bool imu_ok =
        M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5AtomS3RCam);

    Serial.printf("IMU init=%s, type=%d\n",
                  imu_ok ? "ok" : "failed",
                  static_cast<int>(M5.Imu.getType()));
    if (!imu_ok || !M5.Imu.isEnabled()) {
        Serial.println(
            "WARNING: BMI270 unavailable; vision will continue without IMU");
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        imuControlTask,
        "imu_control_200hz",
        4096,
        nullptr,
        configMAX_PRIORITIES - 3,
        &g_imu_task,
        0);

    if (created != pdPASS) {
        Serial.println("FATAL: could not create IMU/control task");
        while (true) delay(1000);
    }

    Serial.printf(
        "READY: QVGA grayscale, marker A=%d, marker B=%d, "
        "marker side=%.1f mm, serial=921600\n",
        appcfg::kMarkerAId, appcfg::kMarkerBId,
        appcfg::kMarkerSideM * 1000.0f);
}

void loop() {
    CameraFrame frame;
    if (!g_camera.capture(frame)) {
        ++g_camera_failures;
        delay(2);
        return;
    }

    ++g_frame_count;
    if (g_last_frame_timestamp_us &&
        frame.timestamp_us > g_last_frame_timestamp_us) {
        const uint64_t dt =
            frame.timestamp_us - g_last_frame_timestamp_us;
        g_frame_dt_us =
            dt > 0xffffffffULL ? 0xffffffffU : static_cast<uint32_t>(dt);
    }
    g_last_frame_timestamp_us = frame.timestamp_us;

    const uint32_t t0 = micros();

    MarkerObservation a = g_tracker_a.process(
        frame.data, g_previous_gray, g_have_previous_gray,
        frame.timestamp_us);
    MarkerObservation b = g_tracker_b.process(
        frame.data, g_previous_gray, g_have_previous_gray,
        frame.timestamp_us);

    const uint32_t vision_us = micros() - t0;
    if (vision_us > g_max_vision_us) g_max_vision_us = vision_us;

    const size_t frame_bytes =
        static_cast<size_t>(appcfg::kFrameWidth) * appcfg::kFrameHeight;
    memcpy(g_previous_gray, frame.data, frame_bytes);
    g_have_previous_gray = true;

    g_camera.release();

    const uint32_t now_ms = millis();
    if (now_ms - g_last_telemetry_ms >= appcfg::kTelemetryPeriodMs) {
        g_last_telemetry_ms = now_ms;
        printTelemetry(a, b, vision_us);
    }

    delay(1);
}
