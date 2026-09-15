#include "camera_driver.h"

#include <Arduino.h>
#include "esp_timer.h"

#include "app_config.h"

namespace {
constexpr int PIN_CAM_POWER_N = 18;
constexpr int PIN_CAM_SDA = 12;
constexpr int PIN_CAM_SCL = 9;
constexpr int PIN_CAM_VSYNC = 10;
constexpr int PIN_CAM_HREF = 14;
constexpr int PIN_CAM_XCLK = 21;
constexpr int PIN_CAM_PCLK = 40;
constexpr int PIN_CAM_D0 = 3;   // Y2
constexpr int PIN_CAM_D1 = 42;  // Y3
constexpr int PIN_CAM_D2 = 46;  // Y4
constexpr int PIN_CAM_D3 = 48;  // Y5
constexpr int PIN_CAM_D4 = 4;   // Y6
constexpr int PIN_CAM_D5 = 17;  // Y7
constexpr int PIN_CAM_D6 = 11;  // Y8
constexpr int PIN_CAM_D7 = 13;  // Y9
}

bool CameraDriver::begin() {
    pinMode(PIN_CAM_POWER_N, OUTPUT);
    digitalWrite(PIN_CAM_POWER_N, LOW);
    delay(500);

    camera_config_t c = {};
    c.pin_pwdn = -1;
    c.pin_reset = -1;
    c.pin_xclk = PIN_CAM_XCLK;
    c.pin_sccb_sda = PIN_CAM_SDA;
    c.pin_sccb_scl = PIN_CAM_SCL;
    c.pin_d7 = PIN_CAM_D7;
    c.pin_d6 = PIN_CAM_D6;
    c.pin_d5 = PIN_CAM_D5;
    c.pin_d4 = PIN_CAM_D4;
    c.pin_d3 = PIN_CAM_D3;
    c.pin_d2 = PIN_CAM_D2;
    c.pin_d1 = PIN_CAM_D1;
    c.pin_d0 = PIN_CAM_D0;
    c.pin_vsync = PIN_CAM_VSYNC;
    c.pin_href = PIN_CAM_HREF;
    c.pin_pclk = PIN_CAM_PCLK;

    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;

    // GC0308 can emit native Y8 in grayscale mode on ESP32-S3. This avoids
    // an RGB conversion and halves the QVGA frame payload versus RGB565.
    c.pixel_format = PIXFORMAT_GRAYSCALE;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.sccb_i2c_port = -1;

    const esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        _last_error = "esp_camera_init failed";
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        _last_error = "camera sensor missing";
        return false;
    }

    // M5Stack's AtomS3R-CAM examples use a vertically flipped image.
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    // Hardware logs show that marker loss correlates strongly with motion.
    // The GC0308 default AEC may select a relatively long indoor exposure,
    // which smears the 4x4 payload. For the motion-robust test, hold exposure
    // to a moderately shorter value while keeping automatic gain enabled.
    // Failure of an optional sensor setter is non-fatal; the startup log tells
    // us exactly which controls the installed esp32-camera version supports.
    if (appcfg::kCameraFastExposure) {
        int rc_aec = -99;
        int rc_value = -99;
        int rc_agc = -99;

        if (s->set_exposure_ctrl) rc_aec = s->set_exposure_ctrl(s, 0);
        if (s->set_aec_value) {
            rc_value = s->set_aec_value(s, appcfg::kCameraExposureValue);
        }
        if (s->set_gain_ctrl) rc_agc = s->set_gain_ctrl(s, 1);

        Serial.printf(
            "CAMERA MOTION TUNE: exposure=%d, aec_off_rc=%d, exposure_rc=%d, agc_auto_rc=%d\n",
            appcfg::kCameraExposureValue, rc_aec, rc_value, rc_agc);
    }

    _last_error = "ok";
    return true;
}

bool CameraDriver::capture(CameraFrame& out) {
    if (_fb) {
        esp_camera_fb_return(_fb);
        _fb = nullptr;
    }

    _fb = esp_camera_fb_get();
    if (!_fb) {
        _last_error = "esp_camera_fb_get failed";
        return false;
    }

    out.data = _fb->buf;
    out.length = _fb->len;
    out.width = _fb->width;
    out.height = _fb->height;

    // esp32-camera timestamps each framebuffer. Fall back to esp_timer if a
    // platform build ever returns an empty timeval.
    if (_fb->timestamp.tv_sec || _fb->timestamp.tv_usec) {
        out.timestamp_us =
            static_cast<uint64_t>(_fb->timestamp.tv_sec) * 1000000ULL +
            static_cast<uint64_t>(_fb->timestamp.tv_usec);
    } else {
        out.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
    }

    return true;
}

void CameraDriver::release() {
    if (_fb) {
        esp_camera_fb_return(_fb);
        _fb = nullptr;
    }
}
