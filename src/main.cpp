#include <Arduino.h>
#include <M5Unified.h>
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

const char* stateName(TrackState state) {
    switch (state) {
        case TrackState::Track: return "track";
        case TrackState::Recover: return "recover";
        default: return "acquire";
    }
}

// Integration point for the real controller. Keeping this in the high-rate
// task proves that vision can remain asynchronous. Do not block here.
void controlStep(const ImuTelemetry&) {
    // Intentionally empty in the first hardware-vision milestone.
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
            sample.sample_timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
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
        "\"%s\":{\"valid\":%s,\"state\":\"%s\",\"id\":%d,"
        "\"rotation\":%d,\"hamming\":%d,\"quality\":%.3f,"
        "\"cx_px\":%.2f,\"cy_px\":%.2f,\"side_px\":%.2f,"
        "\"image_angle_deg\":%.2f,"
        "\"x_m\":%.5f,\"y_m\":%.5f,\"z_m\":%.5f,"
        "\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,"
        "\"vision_us\":%u}",
        name,
        m.valid ? "true" : "false",
        stateName(m.state),
        m.id, m.rotation, m.hamming, m.quality,
        m.center_x_px, m.center_y_px, m.side_px,
        m.image_angle_deg,
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
        "{\"t_us\":%llu,\"frame\":%u,\"camera_failures\":%u,"
        "\"vision_total_us\":%u,\"vision_max_us\":%u,"
        "\"imu\":{\"enabled\":%s,\"loops\":%u,\"misses\":%u,"
        "\"max_step_us\":%u,\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f},",
        static_cast<unsigned long long>(esp_timer_get_time()),
        g_frame_count, g_camera_failures,
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

    if (!psramFound()) {
        Serial.println("FATAL: PSRAM not detected");
        while (true) delay(1000);
    }

    if (!g_detector.begin()) {
        Serial.println("FATAL: detector scratch allocation failed");
        while (true) delay(1000);
    }

    // Important: do not call M5.begin() here. On AtomS3R-CAM its display/
    // board-detection path can touch hardware I2C0 before esp32-camera owns
    // the GC0308 SCCB bus. The camera SCCB pins are GPIO12 SDA / GPIO9 SCL
    // and esp32-camera uses hardware I2C0 by default, so claim it first.
    if (!g_camera.begin()) {
        Serial.printf("FATAL: camera init: %s\n", g_camera.lastError());
        while (true) delay(1000);
    }
    Serial.println("CAMERA READY: GC0308 SCCB owns I2C0");

    // Bring up only the onboard BMI270 from M5Unified, without running the
    // full M5.begin() auto-detection stack. AtomS3R-CAM internal I2C wiring:
    //   I2C1, SDA=GPIO45, SCL=GPIO0.
    // Passing the explicit board type preserves M5Unified's AtomS3R axis map.
    M5.In_I2C.setPort(I2C_NUM_1, GPIO_NUM_45, GPIO_NUM_0);
    const bool imu_ok =
        M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5AtomS3RCam);

    Serial.printf("IMU init=%s, type=%d\n",
                  imu_ok ? "ok" : "failed",
                  static_cast<int>(M5.Imu.getType()));
    if (!imu_ok || !M5.Imu.isEnabled()) {
        Serial.println("WARNING: BMI270 unavailable; vision will continue without IMU");
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
        "READY: QVGA grayscale, marker A=%d, marker B=%d, marker side=%.1f mm\n",
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
    const uint32_t t0 = micros();

    MarkerObservation a =
        g_tracker_a.process(frame.data, frame.timestamp_us);
    MarkerObservation b =
        g_tracker_b.process(frame.data, frame.timestamp_us);

    const uint32_t vision_us = micros() - t0;
    if (vision_us > g_max_vision_us) g_max_vision_us = vision_us;

    g_camera.release();

    const uint32_t now_ms = millis();
    if (now_ms - g_last_telemetry_ms >= appcfg::kTelemetryPeriodMs) {
        g_last_telemetry_ms = now_ms;
        printTelemetry(a, b, vision_us);
    }

    // Vision owns no deadline. The high-rate IMU/control task stays on core 0.
    delay(1);
}
