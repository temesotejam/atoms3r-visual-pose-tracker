#pragma once

#include "vision_types.h"

class MarkerDetector;
class PoseEstimator;

class MarkerTracker {
public:
    MarkerTracker(int marker_id, const RectI& lane,
                  MarkerDetector& detector, const PoseEstimator& estimator);

    MarkerObservation process(const uint8_t* gray, uint64_t frame_timestamp_us);
    RectI currentSearchRoi() const { return _last_roi; }

private:
    RectI computeSearchRoi(uint64_t frame_timestamp_us) const;
    RectI clampToFrame(const RectI& r) const;

    int _id;
    RectI _lane;
    MarkerDetector& _detector;
    const PoseEstimator& _estimator;

    bool _have_track = false;
    int _misses = 0;
    float _vx_px_s = 0.0f;
    float _vy_px_s = 0.0f;
    MarkerObservation _last;
    RectI _last_roi;
};
