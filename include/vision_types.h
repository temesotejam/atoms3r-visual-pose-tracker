#pragma once

#include <stdint.h>

struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

struct RectI {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
};

enum class TrackState : uint8_t {
    Acquire = 0,
    Track = 1,
    Recover = 2,
};

struct MarkerObservation {
    bool valid = false;
    bool corner_refined = false;
    int id = -1;
    int rotation = 0;
    TrackState state = TrackState::Acquire;

    uint64_t frame_timestamp_us = 0;
    uint32_t vision_processing_us = 0;

    Point2f corners[4];
    float center_x_px = 0.0f;
    float center_y_px = 0.0f;
    float side_px = 0.0f;
    float image_angle_deg = 0.0f;

    // Raw homography-decomposition pose. Keep this for diagnostics, but do
    // not feed roll/pitch directly into an EKF unless camera intrinsics are
    // calibrated and tilt_reliable is true. A nearly fronto-parallel planar
    // marker is intrinsically weak for out-of-plane tilt estimation.
    float x_m = 0.0f;
    float y_m = 0.0f;
    float z_m = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    // Mechanism-friendly constrained position estimate. This assumes the
    // marker is approximately fronto-parallel and uses marker center + mean
    // apparent side length. It is intended as the stable visual measurement
    // to evaluate for the mainly-horizontal-motion use case. It is still only
    // approximate until the real camera intrinsics are calibrated.
    float constrained_x_m = 0.0f;
    float constrained_y_m = 0.0f;
    float constrained_z_m = 0.0f;

    // Perspective excitation / observability diagnostics for the raw 3-D
    // tilt estimate. perspective_asymmetry compares opposite projected edge
    // lengths. A small value means roll/pitch are weakly observable from the
    // square in this frame. tilt_reliable is intentionally conservative.
    float perspective_asymmetry = 0.0f;
    bool tilt_reliable = false;

    float quality = 0.0f;
    int hamming = 99;
};

struct ImuTelemetry {
    bool enabled = false;
    uint64_t sample_timestamp_us = 0;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    uint32_t loop_count = 0;
    uint32_t deadline_misses = 0;
    uint32_t max_step_us = 0;
};
