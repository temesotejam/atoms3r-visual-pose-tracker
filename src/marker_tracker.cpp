#include "marker_tracker.h"

#include <Arduino.h>
#include <math.h>

#include "app_config.h"
#include "marker_detector.h"
#include "pose_estimator.h"

MarkerTracker::MarkerTracker(int marker_id, const RectI& lane,
                             MarkerDetector& detector,
                             const PoseEstimator& estimator)
    : _id(marker_id), _lane(lane), _detector(detector), _estimator(estimator),
      _last_roi(lane) {}

RectI MarkerTracker::intersectWithLane(const RectI& in) const {
    const int x0 = in.x > _lane.x ? in.x : _lane.x;
    const int y0 = in.y > _lane.y ? in.y : _lane.y;
    const int x1a = in.x + in.w;
    const int x1b = _lane.x + _lane.w;
    const int y1a = in.y + in.h;
    const int y1b = _lane.y + _lane.h;
    const int x1 = x1a < x1b ? x1a : x1b;
    const int y1 = y1a < y1b ? y1a : y1b;

    RectI out{x0, y0, x1-x0, y1-y0};
    if (out.w < 1 || out.h < 1) return _lane;
    return out;
}

RectI MarkerTracker::computeSearchRoi(uint64_t frame_timestamp_us) const {
    if (!_have_track || _misses >= appcfg::kMaxMissesBeforeLaneAcquire) {
        // `_lane` describes the expected path of the marker center, not a hard
        // crop boundary for the entire square. If the square straddles the
        // boundary (the old defaults meet near image x=160), clipping the
        // black border destroys the ArUco payload before decode. Expand only
        // the acquisition ROI in X; after a lock we immediately return to the
        // much smaller predicted tracking ROI below.
        const int guard = appcfg::kAcquireLaneGuardPx;
        int x0 = _lane.x - guard;
        int x1 = _lane.x + _lane.w + guard;
        if (x0 < 0) x0 = 0;
        if (x1 > appcfg::kFrameWidth) x1 = appcfg::kFrameWidth;

        int y0 = _lane.y;
        int y1 = _lane.y + _lane.h;
        if (y0 < 0) y0 = 0;
        if (y1 > appcfg::kFrameHeight) y1 = appcfg::kFrameHeight;

        return RectI{x0, y0, x1 - x0, y1 - y0};
    }

    float dt = 0.0f;
    if (_last.frame_timestamp_us && frame_timestamp_us > _last.frame_timestamp_us) {
        dt = static_cast<float>(frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
        if (dt > 0.25f) dt = 0.25f;
    }

    const float px = _last.center_x_px + _vx_px_s*dt;
    const float py = _last.center_y_px + _vy_px_s*dt;

    const float side = _last.side_px > 1.0f ? _last.side_px : 32.0f;
    const float recover_scale = 1.0f + 0.75f*_misses;

    int half_w = static_cast<int>(0.65f*side +
                                  appcfg::kTrackMarginXPx*recover_scale);
    int half_h = static_cast<int>(0.70f*side +
                                  appcfg::kTrackMarginYPx*recover_scale);

    if (half_w < 28) half_w = 28;
    if (half_h < 42) half_h = 42;

    RectI r{
        static_cast<int>(px) - half_w,
        static_cast<int>(py) - half_h,
        2*half_w + 1,
        2*half_h + 1
    };
    return intersectWithLane(r);
}

MarkerObservation MarkerTracker::process(const uint8_t* gray,
                                         uint64_t frame_timestamp_us) {
    MarkerObservation obs;
    obs.id = _id;
    obs.frame_timestamp_us = frame_timestamp_us;

    _last_roi = computeSearchRoi(frame_timestamp_us);

    const uint32_t t0 = micros();
    if (!_detector.detect(gray, _last_roi, _id, obs)) {
        ++_misses;
        obs.valid = false;
        obs.state = (_have_track &&
                     _misses < appcfg::kMaxMissesBeforeLaneAcquire)
                        ? TrackState::Recover : TrackState::Acquire;
        obs.vision_processing_us = micros() - t0;
        return obs;
    }

    // `rotation` is the CCW rotation of the canonical dictionary bits that
    // matches the image. To recover the physical marker's canonical corner
    // order we must rotate the geometric TL/TR/BR/BL indices in the opposite
    // direction. Example: a marker observed 90 deg CCW has its canonical TL
    // at the image's BL corner.
    Point2f canonical[4];
    const int corner_shift = (4 - (obs.rotation & 3)) & 3;
    for (int i = 0; i < 4; ++i) {
        canonical[i] = obs.corners[(i + corner_shift) & 3];
    }
    for (int i = 0; i < 4; ++i) obs.corners[i] = canonical[i];

    if (!_estimator.estimate(canonical, obs)) {
        ++_misses;
        obs.valid = false;
        obs.state = TrackState::Recover;
        obs.vision_processing_us = micros() - t0;
        return obs;
    }

    if (_have_track && _last.valid &&
        frame_timestamp_us > _last.frame_timestamp_us) {
        const float dt = static_cast<float>(
            frame_timestamp_us - _last.frame_timestamp_us) * 1e-6f;
        if (dt > 1e-4f && dt < 0.5f) {
            const float raw_vx = (obs.center_x_px - _last.center_x_px) / dt;
            const float raw_vy = (obs.center_y_px - _last.center_y_px) / dt;
            _vx_px_s = 0.55f*_vx_px_s + 0.45f*raw_vx;
            _vy_px_s = 0.55f*_vy_px_s + 0.45f*raw_vy;
        }
    }

    _misses = 0;
    _have_track = true;
    obs.state = TrackState::Track;
    obs.vision_processing_us = micros() - t0;
    _last = obs;
    return obs;
}
