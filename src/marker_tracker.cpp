#include "marker_tracker.h"

#include <Arduino.h>
#include <limits.h>
#include <math.h>

#include "app_config.h"
#include "marker_detector.h"
#include "pose_estimator.h"

namespace {

float pointDistance(const Point2f& a, const Point2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return sqrtf(dx*dx + dy*dy);
}

int patchSad(const uint8_t* previous_gray, const uint8_t* gray,
             int prev_x, int prev_y, int curr_x, int curr_y,
             int radius, int step) {
    if (!previous_gray || !gray || radius < 1 || step < 1) return INT_MAX;

    if (prev_x - radius < 0 || prev_x + radius >= appcfg::kFrameWidth ||
        prev_y - radius < 0 || prev_y + radius >= appcfg::kFrameHeight ||
        curr_x - radius < 0 || curr_x + radius >= appcfg::kFrameWidth ||
        curr_y - radius < 0 || curr_y + radius >= appcfg::kFrameHeight) {
        return INT_MAX;
    }

    int sad = 0;
    for (int oy = -radius; oy <= radius; oy += step) {
        const uint8_t* prev_row =
            previous_gray + (prev_y + oy) * appcfg::kFrameWidth;
        const uint8_t* curr_row =
            gray + (curr_y + oy) * appcfg::kFrameWidth;
        for (int ox = -radius; ox <= radius; ox += step) {
            sad += abs(static_cast<int>(prev_row[prev_x + ox]) -
                       static_cast<int>(curr_row[curr_x + ox]));
        }
    }
    return sad;
}

int patchSampleCount(int radius, int step) {
    int n = 0;
    for (int v = -radius; v <= radius; v += step) ++n;
    return n*n;
}

// Ultra-light horizontal-only match for the real mechanism. We intentionally
// sample just three rows through the known marker and every other X pixel.
// There is no Y search, connected-component scan, corner fit, or homography.
int stripSad1D(const uint8_t* previous_gray, const uint8_t* gray,
               int prev_x, int prev_y, int curr_x,
               int half_width, int sample_step, int row_offset) {
    if (!previous_gray || !gray || half_width < 1 ||
        sample_step < 1 || row_offset < 1) {
        return INT_MAX;
    }

    if (prev_x - half_width < 0 ||
        prev_x + half_width >= appcfg::kFrameWidth ||
        curr_x - half_width < 0 ||
        curr_x + half_width >= appcfg::kFrameWidth ||
        prev_y - row_offset < 0 ||
        prev_y + row_offset >= appcfg::kFrameHeight) {
        return INT_MAX;
    }

    const int rows[3] = {prev_y - row_offset, prev_y, prev_y + row_offset};
    int sad = 0;
    for (int r = 0; r < 3; ++r) {
        const uint8_t* prev_row = previous_gray + rows[r] * appcfg::kFrameWidth;
        const uint8_t* curr_row = gray + rows[r] * appcfg::kFrameWidth;
        for (int ox = -half_width; ox <= half_width; ox += sample_step) {
            sad += abs(static_cast<int>(prev_row[prev_x + ox]) -
                       static_cast<int>(curr_row[curr_x + ox]));
        }
    }
    return sad;
}

int stripSampleCount(int half_width, int sample_step) {
    int n = 0;
    for (int ox = -half_width; ox <= half_width; ox += sample_step) ++n;
    return 3 * n;
}

// MarkerDetector refines geometrically ordered TL/TR/BR/BL corners. The
// tracker's stored corners are canonical marker corners after the ArUco
// rotation correction, so convert between the two orders around refinement.
void canonicalToGeometric(const Point2f canonical[4], int rotation,
                          Point2f geometric[4]) {
    const int r = rotation & 3;
    for (int j = 0; j < 4; ++j) {
        geometric[j] = canonical[(j + r) & 3];
    }
}

void geometricToCanonical(const Point2f geometric[4], int rotation,
                          Point2f canonical[4]) {
    const int corner_shift = (4 - (rotation & 3)) & 3;
    for (int i = 0; i < 4; ++i) {
        canonical[i] = geometric[(i + corner_shift) & 3];
    }
}

} // namespace

MarkerTracker::MarkerTracker(int marker_id, const RectI& lane,
                             MarkerDetector& detector,
                             const PoseEstimator& estimator)
    : _id(marker_id), _lane(lane), _detector(detector),
      _estimator(estimator), _last_roi(lane) {}

RectI MarkerTracker::clampToFrame(const RectI& in) const {
    int x0 = in.x;
    int y0 = in.y;
    int x1 = in.x + in.w;
    int y1 = in.y + in.h;

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > appcfg::kFrameWidth) x1 = appcfg::kFrameWidth;
    if (y1 > appcfg::kFrameHeight) y1 = appcfg::kFrameHeight;

    if (x1 <= x0 || y1 <= y0) {
        return RectI{0, 0, appcfg::kFrameWidth, appcfg::kFrameHeight};
    }
    return RectI{x0, y0, x1 - x0, y1 - y0};
}

RectI MarkerTracker::computeSearchRoi(uint64_t frame_timestamp_us) const {
    if (!_have_track || _misses >= appcfg::kMaxMissesBeforeLaneAcquire) {
        const int guard = appcfg::kAcquireLaneGuardPx;
        return clampToFrame(RectI{
            _lane.x - guard,
            _lane.y - guard,
            _lane.w + 2 * guard,
            _lane.h + 2 * guard
        });
    }

    float dt = 0.0f;
    if (_last.frame_timestamp_us &&
        frame_timestamp_us > _last.frame_timestamp_us) {
        dt = static_cast<float>(
            frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
        if (dt > 0.25f) dt = 0.25f;
    }

    const float px = _last.center_x_px + _vx_px_s * dt;
    const float py = _last.center_y_px + _vy_px_s * dt;
    const float side = _last.side_px > 1.0f ? _last.side_px : 32.0f;
    const float recover_scale = 1.0f + 0.75f * _misses;

    int half_w = static_cast<int>(
        0.65f * side + appcfg::kTrackMarginXPx * recover_scale);
    int half_h = static_cast<int>(
        0.70f * side + appcfg::kTrackMarginYPx * recover_scale);

    if (half_w < 42) half_w = 42;
    if (half_h < 28) half_h = 28;

    return clampToFrame(RectI{
        static_cast<int>(px) - half_w,
        static_cast<int>(py) - half_h,
        2 * half_w + 1,
        2 * half_h + 1
    });
}

bool MarkerTracker::geometryPlausible(
    const MarkerObservation& candidate) const {
    if (!_last.valid || _last.side_px <= 1.0f) return true;

    const float ratio = candidate.side_px / _last.side_px;
    if (ratio < appcfg::kFlowMinSideRatio ||
        ratio > appcfg::kFlowMaxSideRatio) {
        return false;
    }

    float min_edge = 1.0e9f;
    float max_edge = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const float e =
            pointDistance(candidate.corners[i], candidate.corners[(i + 1) & 3]);
        if (e < min_edge) min_edge = e;
        if (e > max_edge) max_edge = e;
    }

    if (min_edge < appcfg::kMinMarkerSidePx * 0.60f) return false;
    if (max_edge > min_edge * 1.80f) return false;
    return true;
}

void MarkerTracker::updateVelocity(const MarkerObservation& current) {
    if (_have_track && _last.valid &&
        current.frame_timestamp_us > _last.frame_timestamp_us) {
        const float dt = static_cast<float>(
            current.frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
        if (dt > 1e-4f && dt < 0.5f) {
            const float raw_vx =
                (current.center_x_px - _last.center_x_px) / dt;
            const float raw_vy =
                (current.center_y_px - _last.center_y_px) / dt;
            _vx_px_s = 0.55f * _vx_px_s + 0.45f * raw_vx;
            _vy_px_s = 0.75f * _vy_px_s + 0.25f * raw_vy;
        }
    }
}

void MarkerTracker::resetFrameDiagnostics() {
    _one_d_fail_reason = TrackFailReason::None;
    _one_d_pred_x_px = 0;
    _one_d_best_x_px = 0;
    _one_d_best_offset_px = 0;
    _one_d_best_mean_sad = 0.0f;

    _pyramid_fail_reason = TrackFailReason::None;
    _pyramid_pred_x_px = 0;
    _pyramid_pred_y_px = 0;
    _pyramid_best_x_px = 0;
    _pyramid_best_y_px = 0;

    _fullframe_aruco_attempted = false;
    _fullframe_aruco_hit = false;
    _fullframe_reacquired = false;
    _fullframe_aruco_us = 0;
}

void MarkerTracker::stampDiagnostics(MarkerObservation& out) const {
    out.one_d_fail_reason = _one_d_fail_reason;
    out.one_d_pred_x_px = _one_d_pred_x_px;
    out.one_d_best_x_px = _one_d_best_x_px;
    out.one_d_best_offset_px = _one_d_best_offset_px;
    out.one_d_best_mean_sad = _one_d_best_mean_sad;

    out.pyramid_fail_reason = _pyramid_fail_reason;
    out.pyramid_pred_x_px = _pyramid_pred_x_px;
    out.pyramid_pred_y_px = _pyramid_pred_y_px;
    out.pyramid_best_x_px = _pyramid_best_x_px;
    out.pyramid_best_y_px = _pyramid_best_y_px;

    out.aruco_local_roi = _last_roi;
    out.fullframe_aruco_attempted = _fullframe_aruco_attempted;
    out.fullframe_aruco_hit = _fullframe_aruco_hit;
    out.fullframe_reacquired = _fullframe_reacquired;
    out.fullframe_aruco_us = _fullframe_aruco_us;
}

void MarkerTracker::stampCounters(MarkerObservation& out) const {
    out.stroke_success_count = _stroke_successes;
    out.stroke_fail_count = _stroke_failures;
    out.one_d_success_count = _one_d_successes;
    out.one_d_fail_count = _one_d_failures;
    out.flow_success_count = _flow_successes;
    out.flow_fail_count = _flow_failures;
    out.decode_success_count = _decode_successes;
    out.reacquire_count = _reacquires;
    stampDiagnostics(out);
}

bool MarkerTracker::trackWithFullStroke(
    const uint8_t* gray,
    int global_threshold,
    uint64_t frame_timestamp_us,
    MarkerObservation& out) {
    if (!_have_track || !_last.valid || !gray) return false;

    MarkerDetector::StrokeMatch match;
    if (!_detector.locateFullStroke1D(
            gray, global_threshold,
            _id, _last.rotation,
            _last.center_y_px, _last.side_px,
            match)) {
        return false;
    }

    const float dx = match.center_x_px - _last.center_x_px;
    const float dy = match.center_y_px - _last.center_y_px;

    out = _last;
    out.valid = true;
    out.corner_refined = false;
    out.stroke_tracked = true;
    out.one_d_tracked = false;
    out.flow_tracked = false;
    out.decoded_this_frame = false;
    out.stroke_score = match.score;
    out.stroke_hamming = match.hamming;
    out.stroke_border_black = match.border_black;
    out.track_mean_sad = 0.0f;
    out.frame_timestamp_us = frame_timestamp_us;
    out.state = TrackState::Track;
    out.center_x_px = match.center_x_px;
    out.center_y_px = match.center_y_px;

    for (int i = 0; i < 4; ++i) {
        out.corners[i].x = _last.corners[i].x + dx;
        out.corners[i].y = _last.corners[i].y + dy;
    }

    if (out.constrained_z_m > 0.0f) {
        out.constrained_x_m =
            (out.center_x_px - appcfg::kCxPx) *
            out.constrained_z_m / appcfg::kFxPx;
        out.constrained_y_m =
            (out.center_y_px - appcfg::kCyPx) *
            out.constrained_z_m / appcfg::kFyPx;
    }
    out.tilt_reliable = false;

    if (appcfg::kEnableRawHomographyPose) {
        if (!_estimator.estimate(out.corners, out)) return false;
    }
    return true;
}

bool MarkerTracker::trackWith1D(const uint8_t* previous_gray,
                                const uint8_t* gray,
                                uint64_t frame_timestamp_us,
                                MarkerObservation& out) {
    if (!_have_track || !_previous_frame_valid || !_last.valid ||
        !previous_gray || !gray ||
        frame_timestamp_us <= _last.frame_timestamp_us) {
        _one_d_fail_reason = TrackFailReason::Preconditions;
        return false;
    }

    const float dt = static_cast<float>(
        frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.25f) {
        _one_d_fail_reason = TrackFailReason::Timing;
        return false;
    }

    const int prev_x = static_cast<int>(lroundf(_last.center_x_px));
    const int prev_y = static_cast<int>(lroundf(_last.center_y_px));
    const int pred_x = static_cast<int>(
        lroundf(_last.center_x_px + _vx_px_s * dt));
    _one_d_pred_x_px = pred_x;

    int half_width = static_cast<int>(lroundf(0.42f * _last.side_px));
    if (half_width < 6) half_width = 6;
    if (half_width > appcfg::kOneDPatchHalfWidthPx) {
        half_width = appcfg::kOneDPatchHalfWidthPx;
    }

    int row_offset = static_cast<int>(lroundf(0.22f * _last.side_px));
    if (row_offset < 2) row_offset = 2;
    if (row_offset > 7) row_offset = 7;

    constexpr int kCostCount = 2 * appcfg::kOneDSearchPx + 1;
    int costs[kCostCount];
    for (int i = 0; i < kCostCount; ++i) costs[i] = INT_MAX;

    int best_cost = INT_MAX;
    int best_offset = 0;

    for (int dx = -appcfg::kOneDSearchPx;
         dx <= appcfg::kOneDSearchPx; ++dx) {
        const int cost = stripSad1D(
            previous_gray, gray,
            prev_x, prev_y, pred_x + dx,
            half_width, appcfg::kOneDSampleStepPx, row_offset);
        costs[dx + appcfg::kOneDSearchPx] = cost;
        if (cost < best_cost) {
            best_cost = cost;
            best_offset = dx;
        }
    }

    _one_d_best_offset_px = best_offset;
    _one_d_best_x_px = pred_x + best_offset;

    if (best_cost == INT_MAX) {
        _one_d_fail_reason = TrackFailReason::Bounds;
        return false;
    }
    if (abs(best_offset) >= appcfg::kOneDSearchPx) {
        _one_d_fail_reason = TrackFailReason::SearchBoundary;
        return false;
    }

    const int sample_count =
        stripSampleCount(half_width, appcfg::kOneDSampleStepPx);
    const float mean_sad =
        sample_count > 0
            ? static_cast<float>(best_cost) / sample_count
            : 255.0f;
    _one_d_best_mean_sad = mean_sad;
    if (mean_sad > appcfg::kOneDMaxMeanSad) {
        _one_d_fail_reason = TrackFailReason::Sad;
        return false;
    }

    // Cheap sub-pixel interpolation of the 1-D SAD minimum. Clamp to half a
    // pixel so ambiguous/flat cost curves cannot create a large correction.
    float sub_px = 0.0f;
    const int best_i = best_offset + appcfg::kOneDSearchPx;
    if (best_i > 0 && best_i + 1 < kCostCount &&
        costs[best_i - 1] != INT_MAX &&
        costs[best_i + 1] != INT_MAX) {
        const float l = static_cast<float>(costs[best_i - 1]);
        const float c = static_cast<float>(costs[best_i]);
        const float r = static_cast<float>(costs[best_i + 1]);
        const float denom = l - 2.0f*c + r;
        if (fabsf(denom) > 1.0f) {
            sub_px = 0.5f * (l - r) / denom;
            if (sub_px < -0.5f) sub_px = -0.5f;
            if (sub_px > 0.5f) sub_px = 0.5f;
        }
    }

    const float current_x =
        static_cast<float>(pred_x + best_offset) + sub_px;
    const float dx_px = current_x - _last.center_x_px;

    // Keep the last validated square geometry and translate it only in X.
    // The mechanism itself supplies the missing constraints: Y, scale and
    // rotation are effectively constant over this short travel.
    out = _last;
    out.valid = true;
    out.corner_refined = false;
    out.stroke_tracked = false;
    out.one_d_tracked = true;
    out.flow_tracked = false;
    out.decoded_this_frame = false;
    out.track_mean_sad = mean_sad;
    out.frame_timestamp_us = frame_timestamp_us;
    out.state = TrackState::Track;
    out.center_x_px = _last.center_x_px + dx_px;
    for (int i = 0; i < 4; ++i) {
        out.corners[i].x = _last.corners[i].x + dx_px;
    }

    if (out.constrained_z_m > 0.0f) {
        out.constrained_x_m =
            (out.center_x_px - appcfg::kCxPx) *
            out.constrained_z_m / appcfg::kFxPx;
    }
    out.tilt_reliable = false;

    // A dedicated raw-pose debug build may still request a full update.
    if (appcfg::kEnableRawHomographyPose) {
        if (!_estimator.estimate(out.corners, out)) return false;
    }

    return true;
}

bool MarkerTracker::trackWithPyramid(const uint8_t* previous_gray,
                                     const uint8_t* gray,
                                     uint64_t frame_timestamp_us,
                                     MarkerObservation& out) {
    if (!_have_track || !_previous_frame_valid || !_last.valid ||
        !previous_gray || !gray ||
        frame_timestamp_us <= _last.frame_timestamp_us) {
        _pyramid_fail_reason = TrackFailReason::Preconditions;
        return false;
    }

    const float dt = static_cast<float>(
        frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.25f) {
        _pyramid_fail_reason = TrackFailReason::Timing;
        return false;
    }

    const int prev_x = static_cast<int>(lroundf(_last.center_x_px));
    const int prev_y = static_cast<int>(lroundf(_last.center_y_px));
    const int pred_x = static_cast<int>(
        lroundf(_last.center_x_px + _vx_px_s * dt));
    const int pred_y = static_cast<int>(
        lroundf(_last.center_y_px + _vy_px_s * dt));
    _pyramid_pred_x_px = pred_x;
    _pyramid_pred_y_px = pred_y;

    int coarse_best = INT_MAX;
    int coarse_x = pred_x;
    int coarse_y = pred_y;

    for (int dy = -appcfg::kFlowCoarseSearchPx;
         dy <= appcfg::kFlowCoarseSearchPx; dy += 2) {
        for (int dx = -appcfg::kFlowCoarseSearchPx;
             dx <= appcfg::kFlowCoarseSearchPx; dx += 2) {
            const int x = pred_x + dx;
            const int y = pred_y + dy;
            const int sad = patchSad(
                previous_gray, gray,
                prev_x, prev_y, x, y,
                appcfg::kFlowCoarsePatchRadiusPx, 2);
            if (sad < coarse_best) {
                coarse_best = sad;
                coarse_x = x;
                coarse_y = y;
            }
        }
    }

    _pyramid_best_x_px = coarse_x;
    _pyramid_best_y_px = coarse_y;
    if (coarse_best == INT_MAX) {
        _pyramid_fail_reason = TrackFailReason::Bounds;
        return false;
    }

    // A best match exactly at the coarse boundary means the true displacement
    // may lie outside the searched region. Decode the marker again instead of
    // accepting a clipped optical-flow estimate.
    if (abs(coarse_x - pred_x) >= appcfg::kFlowCoarseSearchPx ||
        abs(coarse_y - pred_y) >= appcfg::kFlowCoarseSearchPx) {
        _pyramid_fail_reason = TrackFailReason::SearchBoundary;
        return false;
    }

    int fine_best = INT_MAX;
    int fine_x = coarse_x;
    int fine_y = coarse_y;

    for (int dy = -appcfg::kFlowRefineSearchPx;
         dy <= appcfg::kFlowRefineSearchPx; ++dy) {
        for (int dx = -appcfg::kFlowRefineSearchPx;
             dx <= appcfg::kFlowRefineSearchPx; ++dx) {
            const int x = coarse_x + dx;
            const int y = coarse_y + dy;
            const int sad = patchSad(
                previous_gray, gray,
                prev_x, prev_y, x, y,
                appcfg::kFlowFinePatchRadiusPx, 1);
            if (sad < fine_best) {
                fine_best = sad;
                fine_x = x;
                fine_y = y;
            }
        }
    }

    _pyramid_best_x_px = fine_x;
    _pyramid_best_y_px = fine_y;
    if (fine_best == INT_MAX) {
        _pyramid_fail_reason = TrackFailReason::Bounds;
        return false;
    }

    const int sample_count =
        patchSampleCount(appcfg::kFlowFinePatchRadiusPx, 1);
    const float mean_sad =
        sample_count > 0 ? static_cast<float>(fine_best) / sample_count
                         : 255.0f;
    if (mean_sad > appcfg::kFlowMaxMeanSad) {
        _pyramid_fail_reason = TrackFailReason::Sad;
        return false;
    }

    const float dx = static_cast<float>(fine_x - prev_x);
    const float dy = static_cast<float>(fine_y - prev_y);

    Point2f shifted_canonical[4];
    for (int i = 0; i < 4; ++i) {
        shifted_canonical[i] = {
            _last.corners[i].x + dx,
            _last.corners[i].y + dy
        };
    }

    Point2f coarse_geometric[4];
    canonicalToGeometric(
        shifted_canonical, _last.rotation, coarse_geometric);

    Point2f refined_geometric[4];
    if (!_detector.refineKnownCorners(
            gray, coarse_geometric, refined_geometric)) {
        _pyramid_fail_reason = TrackFailReason::Refine;
        return false;
    }

    Point2f refined_canonical[4];
    geometricToCanonical(
        refined_geometric, _last.rotation, refined_canonical);

    out = MarkerObservation{};
    out.valid = true;
    out.corner_refined = true;
    out.flow_tracked = true;
    out.decoded_this_frame = false;
    out.id = _id;
    out.rotation = _last.rotation;
    out.hamming = _last.hamming;
    out.quality = _last.quality;
    out.track_mean_sad = mean_sad;
    out.frame_timestamp_us = frame_timestamp_us;
    out.state = TrackState::Track;

    for (int i = 0; i < 4; ++i) out.corners[i] = refined_canonical[i];

    if (!_estimator.estimate(refined_canonical, out)) {
        _pyramid_fail_reason = TrackFailReason::Pose;
        return false;
    }
    if (!geometryPlausible(out)) {
        _pyramid_fail_reason = TrackFailReason::Geometry;
        return false;
    }
    return true;
}

MarkerObservation MarkerTracker::process(const uint8_t* gray,
                                         const uint8_t* previous_gray,
                                         bool have_previous_frame,
                                         uint64_t frame_timestamp_us,
                                         int global_threshold) {
    resetFrameDiagnostics();

    MarkerObservation obs;
    obs.id = _id;
    obs.frame_timestamp_us = frame_timestamp_us;
    _last_roi = computeSearchRoi(frame_timestamp_us);

    const uint32_t t0 = micros();

    // Primary path: search the whole physical stroke in the current frame.
    // It does not depend on the previous-frame displacement, so a fast jump
    // across the old +/-10/16 px windows can still be recovered immediately.
    if (_have_track && _last.valid) {
        if (trackWithFullStroke(
                gray, global_threshold, frame_timestamp_us, obs)) {
            ++_stroke_successes;
            _misses = 0;
            updateVelocity(obs);
            _acquire_decode_cooldown =
                appcfg::kRecoveryDecodeEveryNFrames - 1;
            stampCounters(obs);
            obs.vision_processing_us = micros() - t0;
            _last = obs;
            _previous_frame_valid = true;
            return obs;
        }
        ++_stroke_failures;
    }

    const bool can_flow =
        have_previous_frame && _previous_frame_valid &&
        _have_track && _last.valid && previous_gray;

    if (can_flow) {
        if (trackWith1D(
                previous_gray, gray, frame_timestamp_us, obs)) {
            ++_one_d_successes;
            _misses = 0;
            updateVelocity(obs);
            _acquire_decode_cooldown =
                appcfg::kRecoveryDecodeEveryNFrames - 1;
            stampCounters(obs);
            obs.vision_processing_us = micros() - t0;
            _last = obs;
            _previous_frame_valid = true;
            return obs;
        }
        ++_one_d_failures;

        if (trackWithPyramid(
                previous_gray, gray, frame_timestamp_us, obs)) {
            ++_flow_successes;
            _misses = 0;
            updateVelocity(obs);
            _acquire_decode_cooldown =
                appcfg::kRecoveryDecodeEveryNFrames - 1;
            stampCounters(obs);
            obs.vision_processing_us = micros() - t0;
            _last = obs;
            _previous_frame_valid = true;
            return obs;
        }
        ++_flow_failures;
    }

    const bool was_recovery =
        _have_track && (_misses > 0 || can_flow);

    const bool full_acquire =
        !_have_track || _misses >= appcfg::kMaxMissesBeforeLaneAcquire;
    const int decode_period =
        _have_track
            ? appcfg::kRecoveryDecodeEveryNFrames
            : appcfg::kAcquireDecodeEveryNFrames;

    if (_acquire_decode_cooldown > 0) {
        --_acquire_decode_cooldown;
        ++_misses;
        _vx_px_s *= 0.75f;
        _vy_px_s = 0.0f;

        obs = MarkerObservation{};
        obs.id = _id;
        obs.frame_timestamp_us = frame_timestamp_us;
        obs.valid = false;
        obs.state = (_have_track &&
                     _misses < appcfg::kMaxMissesBeforeLaneAcquire)
                        ? TrackState::Recover
                        : TrackState::Acquire;
        stampCounters(obs);
        obs.vision_processing_us = micros() - t0;
        _previous_frame_valid = false;
        return obs;
    }
    _acquire_decode_cooldown = decode_period > 0 ? decode_period - 1 : 0;

    bool aruco_found = _detector.detect(
        gray, _last_roi, _id, obs, global_threshold);

    // Root-cause diagnostic: if a marker that was previously tracked is not
    // found inside the normal local/lane ROI, retry the exact same image over
    // the entire framebuffer. A hit here proves the camera still saw/decoded
    // the marker and the loss was caused by our local search region.
    if (!aruco_found &&
        appcfg::kEnableFullFrameLossDiagnostic &&
        was_recovery &&
        !(_last_roi.x == 0 && _last_roi.y == 0 &&
          _last_roi.w == appcfg::kFrameWidth &&
          _last_roi.h == appcfg::kFrameHeight)) {
        _fullframe_aruco_attempted = true;
        const uint32_t full_t0 = micros();
        const RectI full_frame{
            0, 0, appcfg::kFrameWidth, appcfg::kFrameHeight
        };
        aruco_found = _detector.detect(
            gray, full_frame, _id, obs, global_threshold);
        _fullframe_aruco_us = micros() - full_t0;
        _fullframe_aruco_hit = aruco_found;
        _fullframe_reacquired = aruco_found;
    }

    if (!aruco_found) {
        ++_misses;
        _vx_px_s *= 0.75f;
        _vy_px_s = 0.0f;

        obs = MarkerObservation{};
        obs.id = _id;
        obs.frame_timestamp_us = frame_timestamp_us;
        obs.valid = false;
        obs.state = (_have_track &&
                     _misses < appcfg::kMaxMissesBeforeLaneAcquire)
                        ? TrackState::Recover
                        : TrackState::Acquire;
        stampCounters(obs);
        obs.vision_processing_us = micros() - t0;
        _previous_frame_valid = false;
        return obs;
    }

    Point2f canonical[4];
    const int corner_shift = (4 - (obs.rotation & 3)) & 3;
    for (int i = 0; i < 4; ++i) {
        canonical[i] = obs.corners[(i + corner_shift) & 3];
    }
    for (int i = 0; i < 4; ++i) obs.corners[i] = canonical[i];

    if (!_estimator.estimate(canonical, obs)) {
        ++_misses;
        _vx_px_s *= 0.75f;
        _vy_px_s *= 0.75f;
        obs.valid = false;
        obs.state = TrackState::Recover;
        stampCounters(obs);
        obs.vision_processing_us = micros() - t0;
        _previous_frame_valid = false;
        return obs;
    }

    ++_decode_successes;
    if (was_recovery) ++_reacquires;

    _acquire_decode_cooldown =
        appcfg::kRecoveryDecodeEveryNFrames - 1;
    obs.stroke_tracked = false;
    obs.one_d_tracked = false;
    obs.decoded_this_frame = true;
    obs.flow_tracked = false;
    obs.track_mean_sad = 0.0f;
    obs.state = TrackState::Track;

    updateVelocity(obs);
    _misses = 0;
    _have_track = true;
    stampCounters(obs);
    obs.vision_processing_us = micros() - t0;
    _last = obs;
    _previous_frame_valid = true;
    return obs;
}
