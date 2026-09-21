#pragma once

#include "app_config.h"
#include "vision_types.h"

class MarkerDetector;
class PoseEstimator;

class MarkerTracker {
public:
    MarkerTracker(int marker_id, const RectI& lane,
                  MarkerDetector& detector, const PoseEstimator& estimator);

    MarkerObservation process(const uint8_t* gray,
                              const uint8_t* previous_gray,
                              bool have_previous_frame,
                              uint64_t frame_timestamp_us,
                              int& aruco_threshold_cache);
    RectI currentSearchRoi() const { return _last_roi; }

private:
    RectI computeSearchRoi(uint64_t frame_timestamp_us) const;
    RectI clampToFrame(const RectI& r) const;
    bool trackWithWideTemplate(const uint8_t* gray,
                               uint64_t frame_timestamp_us,
                               MarkerObservation& out);
    bool captureWideTemplate(const uint8_t* gray,
                             const MarkerObservation& reference);
    bool trackWith1D(const uint8_t* previous_gray,
                     const uint8_t* gray,
                     uint64_t frame_timestamp_us,
                     MarkerObservation& out);
    bool trackWithPyramid(const uint8_t* previous_gray,
                          const uint8_t* gray,
                          uint64_t frame_timestamp_us,
                          MarkerObservation& out);
    bool geometryPlausible(const MarkerObservation& candidate) const;
    void updateVelocity(const MarkerObservation& current);
    void stampCounters(MarkerObservation& out) const;
    void resetFrameDiagnostics();
    void stampDiagnostics(MarkerObservation& out) const;

    int _id;
    RectI _lane;
    MarkerDetector& _detector;
    const PoseEstimator& _estimator;

    bool _have_track = false;
    bool _previous_frame_valid = false;
    int _misses = 0;
    float _vx_px_s = 0.0f;
    float _vy_px_s = 0.0f;
    int _acquire_decode_cooldown = 0;
    uint32_t _wide_template_successes = 0;
    uint32_t _wide_template_failures = 0;
    uint32_t _one_d_successes = 0;
    uint32_t _one_d_failures = 0;
    uint32_t _flow_successes = 0;
    uint32_t _flow_failures = 0;
    uint32_t _decode_successes = 0;
    uint32_t _reacquires = 0;

    static constexpr int kWideTemplateCols =
        (2 * appcfg::kWideTemplateHalfWidthPx) /
            appcfg::kWideTemplateSampleStepPx + 1;
    static constexpr int kWideTemplateSamples =
        appcfg::kWideTemplateRows * kWideTemplateCols;

    bool _wide_template_valid = false;
    uint8_t _wide_template[kWideTemplateSamples] = {};
    int _wide_template_mean = 0;
    int _wide_template_center_y = 0;
    int _wide_template_row_step = 4;
    float _wide_template_side_px = 0.0f;

    float _wide_last_sad = 0.0f;
    int _wide_last_contrast = 0;
    int _wide_last_x = 0;
    int _wide_last_y = 0;
    uint32_t _wide_last_us = 0;

    TrackFailReason _one_d_fail_reason = TrackFailReason::None;
    int _one_d_pred_x_px = 0;
    int _one_d_best_x_px = 0;
    int _one_d_best_offset_px = 0;
    float _one_d_best_mean_sad = 0.0f;

    TrackFailReason _pyramid_fail_reason = TrackFailReason::None;
    int _pyramid_pred_x_px = 0;
    int _pyramid_pred_y_px = 0;
    int _pyramid_best_x_px = 0;
    int _pyramid_best_y_px = 0;

    bool _fullframe_aruco_attempted = false;
    bool _fullframe_aruco_hit = false;
    bool _fullframe_reacquired = false;
    uint32_t _fullframe_aruco_us = 0;

    MarkerObservation _last;
    RectI _last_roi;
};
