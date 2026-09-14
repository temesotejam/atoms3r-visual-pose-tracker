#pragma once

#include "vision_types.h"

class PoseEstimator {
public:
    PoseEstimator(float fx_px, float fy_px, float cx_px, float cy_px,
                  float marker_side_m)
        : _fx(fx_px), _fy(fy_px), _cx(cx_px), _cy(cy_px),
          _marker_side(marker_side_m) {}

    bool estimate(const Point2f canonical_corners[4], MarkerObservation& out) const;

private:
    bool solveHomography(const Point2f src[4], const Point2f dst[4],
                         float h[9]) const;

    float _fx;
    float _fy;
    float _cx;
    float _cy;
    float _marker_side;
};
