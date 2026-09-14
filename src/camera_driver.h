#pragma once

#include <stdint.h>
#include "esp_camera.h"

struct CameraFrame {
    const uint8_t* data = nullptr;
    size_t length = 0;
    int width = 0;
    int height = 0;
    uint64_t timestamp_us = 0;
};

class CameraDriver {
public:
    bool begin();
    bool capture(CameraFrame& out);
    void release();
    const char* lastError() const { return _last_error; }

private:
    camera_fb_t* _fb = nullptr;
    const char* _last_error = "ok";
};
