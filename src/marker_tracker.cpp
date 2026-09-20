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

void MarkerTracker::stampCounters(MarkerObservation& out) const {
    out.flow_success_count = _flow_successes;
    out.flow_fail_count = _flow_failures;
    out.decode_success_count = _decode_successes;
    out.reacquire_count = _reacquires;
}

bool MarkerTracker::trackWithPyramid(const uint8_t* previous_gray,
                                     const uint8_t* gray,
                                     uint64_t frame_timestamp_us,
                                     MarkerObservation& out) {
    if (!_have_track || !_previous_frame_valid || !_last.valid ||
        !previous_gray || !gray ||
        frame_timestamp_us <= _last.frame_timestamp_us) {
        return false;
    }

    const float dt = static_cast<float>(
        frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.25f) return false;

    const int prev_x = static_cast<int>(lroundf(_last.center_x_px));
    const int prev_y = static_cast<int>(lroundf(_last.center_y_px));
    const int pred_x = static_cast<int>(
        lroundf(_last.center_x_px + _vx_px_s * dt));
    const int pred_y = static_cast<int>(
        lroundf(_last.center_y_px + _vy_px_s * dt));

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

    if (coarse_best == INT_MAX) return false;

    // A best match exactly at the coarse boundary means the true displacement
    // may lie outside the searched region. Decode the marker again instead of
    // accepting a clipped optical-flow estimate.
    if (abs(coarse_x - pred_x) >= appcfg::kFlowCoarseSearchPx ||
        abs(coarse_y - pred_y) >= appcfg::kFlowCoarseSearchPx) {
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

    if (fine_best == INT_MAX) return false;

    const int sample_count =
        patchSampleCount(appcfg::kFlowFinePatchRadiusPx, 1);
    const float mean_sad =
        sample_count > 0 ? static_cast<float>(fine_best) / sample_count
                         : 255.0f;
    if (mean_sad > appcfg::kFlowMaxMeanSad) return false;

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

    if (!_estimator.estimate(refined_canonical, out)) return false;
    if (!geometryPlausible(out)) return false;
    return true;
}

MarkerObservation MarkerTracker::process(const uint8_t* gray,
                                         const uint8_t* previous_gray,
                                         bool have_previous_frame,
                                         uint64_t frame_timestamp_us) {
    MarkerObservation obs;
    obs.id = _id;
    obs.frame_timestamp_us = frame_timestamp_us;
    _last_roi = computeSearchRoi(frame_timestamp_us);

    const uint32_t t0 = micros();
    const bool can_flow =
        have_previous_frame && _previous_frame_valid &&
        _have_track && _last.valid && previous_gray;

    if (can_flow) {
        if (trackWithPyramid(
                previous_gray, gray, frame_timestamp_us, obs)) {
            ++_flow_successes;
            _misses = 0;
            updateVelocity(obs);
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

    if (!_detector.detect(gray, _last_roi, _id, obs)) {
        ++_misses;
        _vx_px_s *= 0.75f;
        _vy_px_s *= 0.75f;

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
