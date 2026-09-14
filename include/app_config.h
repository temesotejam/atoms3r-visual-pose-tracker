#pragma once

#include <stdint.h>

namespace appcfg {

static constexpr int kFrameWidth = 320;
static constexpr int kFrameHeight = 240;
static constexpr int kCameraFpsTarget = 15;

// Standard OpenCV ArUco DICT_4X4_50 IDs used by the first prototype.
static constexpr int kMarkerAId = 0;
static constexpr int kMarkerBId = 1;

// Physical black-square side length of the printed marker.
// Change this to the actual printed size before trusting metric XYZ.
static constexpr float kMarkerSideM = 0.050f;

// Approximate QVGA intrinsics. These are only good enough to bring up the
// pipeline. Calibrate the actual camera and replace these values before
// using metric pose in the EKF.
static constexpr float kFxPx = 290.0f;
static constexpr float kFyPx = 290.0f;
static constexpr float kCxPx = 159.5f;
static constexpr float kCyPx = 119.5f;

// Initial acquisition lanes. The marker motion is assumed to be mainly
// vertical. Adjust these four values after mounting the camera.
// Lane A defaults to left half, lane B to right half.
static constexpr int kLaneAX = 0;
static constexpr int kLaneAY = 0;
static constexpr int kLaneAW = 170;
static constexpr int kLaneAH = 240;

static constexpr int kLaneBX = 150;
static constexpr int kLaneBY = 0;
static constexpr int kLaneBW = 170;
static constexpr int kLaneBH = 240;

// A lane describes where the marker CENTER is expected to move. During
// acquisition the image-search ROI is expanded beyond the lane so that a
// marker near a lane boundary is not cut in half before its ID is decoded.
// 96 px is slightly more than half of kMaxMarkerSidePx (180 px).
static constexpr int kAcquireLaneGuardPx = 96;

// Tracking ROI around the predicted center.
static constexpr int kTrackMarginXPx = 24;
static constexpr int kTrackMarginYPx = 52;
static constexpr int kMaxMissesBeforeLaneAcquire = 3;

// Candidate geometry.
static constexpr int kMinMarkerSidePx = 18;
static constexpr int kMaxMarkerSidePx = 180;
static constexpr int kMinBlackComponentAreaPx = 90;
static constexpr int kMaxHammingError = 1;

// High-rate side of the application. The vision task is deliberately not
// allowed to dictate this period.
static constexpr uint32_t kImuControlPeriodUs = 5000; // 200 Hz
static constexpr uint32_t kTelemetryPeriodMs = 100;   // 10 Hz serial JSON

} // namespace appcfg
