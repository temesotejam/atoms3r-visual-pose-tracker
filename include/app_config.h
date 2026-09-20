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
// Verified against the CAD definition and the 10 mm grid in the hardware image.
static constexpr float kMarkerSideM = 0.006f;

// Approximate QVGA intrinsics. These are only good enough to bring up the
// pipeline. Calibrate the actual camera and replace these values before
// using metric pose in the EKF.
static constexpr float kFxPx = 290.0f;
static constexpr float kFyPx = 290.0f;
static constexpr float kCxPx = 159.5f;
static constexpr float kCyPx = 119.5f;

// Keep false until the actual AtomS3R-CAM module has been calibrated. Raw
// planar homography roll/pitch is not considered EKF-grade while this is false.
static constexpr bool kCameraIntrinsicsCalibrated = false;

// Raw homography 6DoF is diagnostic only and is intentionally disabled in
// normal operation. The constrained horizontal-motion measurements do not
// need the 8x8 homography solve/decomposition, so leave this false unless a
// dedicated camera-calibration / pose-debug experiment needs the raw fields.
static constexpr bool kEnableRawHomographyPose = false;

// Even after calibration, a nearly fronto-parallel square gives little
// perspective information about out-of-plane tilt. This relative opposite-edge
// asymmetry is a conservative first gate; hardware logs should be used to tune
// it before enabling visual roll/pitch corrections.
static constexpr float kTiltMinPerspectiveAsymmetry = 0.035f;
static constexpr int kTiltMinMarkerSidePx = 32;

// Initial acquisition regions. Hardware mounting shows that both markers move
// mainly HORIZONTALLY: marker A stays in the upper band and marker B in the
// lower band. These rectangles constrain the expected marker CENTER only.
static constexpr int kLaneAX = 0;
static constexpr int kLaneAY = 0;
static constexpr int kLaneAW = 320;
static constexpr int kLaneAH = 105;

static constexpr int kLaneBX = 0;
static constexpr int kLaneBY = 115;
static constexpr int kLaneBW = 320;
static constexpr int kLaneBH = 125;

// Expand the center-path region in both X and Y during acquisition so the
// complete ~50 px marker remains visible when its center approaches a band
// boundary. 40 px is deliberately larger than half the observed marker side.
static constexpr int kAcquireLaneGuardPx = 40;

// Tracking ROI around the predicted center. Horizontal travel gets the larger
// margin; vertical motion is expected to be small for the mounted mechanism.
static constexpr int kTrackMarginXPx = 52;
static constexpr int kTrackMarginYPx = 20;
static constexpr int kMaxMissesBeforeLaneAcquire = 3;

// Candidate geometry.
static constexpr int kMinMarkerSidePx = 18;
static constexpr int kMaxMarkerSidePx = 180;
static constexpr int kMinBlackComponentAreaPx = 90;
static constexpr int kMaxHammingError = 1;

// The mechanism is much more constrained than a general image tracker: each
// marker stays on its own nearly-fixed horizontal line and only translates a
// short distance in X. Try an ultra-light 1-D strip match first. It samples
// only three rows and every other X pixel, then shifts the already-known marker
// geometry without re-fitting four corners or solving pose.
static constexpr int kOneDSearchPx = 10;
static constexpr int kOneDPatchHalfWidthPx = 11;
static constexpr int kOneDSampleStepPx = 2;
static constexpr float kOneDMaxMeanSad = 30.0f;

// If the marker is fully lost, do not run the expensive lane-wide ArUco scan
// on every frame. Recovery around a recent track is still attempted every
// frame; only the cold/full acquisition state is duty-cycled.
static constexpr int kAcquireDecodeEveryNFrames = 3;

// Two-level local block tracking remains as a safety fallback behind the 1-D
// tracker. It is more general (X/Y motion + corner re-fit) but substantially
// more expensive, so it should normally be bypassed.
static constexpr int kFlowCoarseSearchPx = 16;
static constexpr int kFlowRefineSearchPx = 3;
static constexpr int kFlowCoarsePatchRadiusPx = 6;
static constexpr int kFlowFinePatchRadiusPx = 5;
static constexpr float kFlowMaxMeanSad = 38.0f;
static constexpr float kFlowMinSideRatio = 0.65f;
static constexpr float kFlowMaxSideRatio = 1.55f;

// Lightweight sub-pixel-ish corner refinement. The coarse connected-component
// extrema are only used to acquire/decode a marker. Once an ID is accepted,
// each outer black/white edge is re-measured at several locations, a line is
// fitted to the edge samples, and adjacent lines are intersected. This is much
// cheaper than a general-purpose OpenCV cornerSubPix pass and targets the
// roll/pitch quantization seen in hardware logs with ~30-50 px markers.
static constexpr int kCornerRefineSamplesPerEdge = 10;
static constexpr int kCornerRefineSearchRadiusPx = 5;
static constexpr float kCornerRefineMinContrast = 10.0f;
static constexpr float kCornerRefineMaxShiftFraction = 0.22f;

// High-rate side of the application. The vision task is deliberately not
// allowed to dictate this period.
static constexpr uint32_t kImuControlPeriodUs = 5000; // 200 Hz
static constexpr uint32_t kTelemetryPeriodMs = 100;   // 10 Hz serial JSON

} // namespace appcfg
