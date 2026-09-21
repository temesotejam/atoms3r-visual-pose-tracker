#pragma once

#include <stdint.h>
#include "vision_types.h"

class MarkerDetector {
public:
    struct StrokeMatch {
        bool valid = false;
        float center_x_px = 0.0f;
        float center_y_px = 0.0f;
        float score = 1.0e9f;
        int hamming = 99;
        int border_black = 0;
        int rotation = -1;
    };

    MarkerDetector(int frame_width, int frame_height);
    ~MarkerDetector();

    bool begin();

    // Returns a geometrically ordered TL,TR,BR,BL candidate. The caller then
    // rotates those corners into canonical marker orientation using rotation.
    bool detect(const uint8_t* gray, const RectI& roi, int expected_id,
                MarkerObservation& out, int threshold_override = -1);

    // One global threshold per camera frame removes ROI-dependent Otsu changes
    // between A/B and between small/large recovery regions.
    int computeGlobalThreshold(const uint8_t* gray) const;

    // Current-frame constrained search. X spans the full usable image width;
    // Y and scale come from the already-identified mechanism geometry.
    bool locateFullStroke1D(const uint8_t* gray, int threshold,
                            int expected_id, int expected_rotation,
                            float center_y_px, float side_px,
                            StrokeMatch& out) const;

    // Re-measure the four outer square edges around an already-known
    // geometrically ordered TL/TR/BR/BL marker. This lets the fast tracker
    // reuse the same edge-line refinement without decoding the ID every frame.
    bool refineKnownCorners(const uint8_t* gray, const Point2f coarse[4],
                            Point2f refined[4]) const;

private:
    struct Candidate {
        bool valid = false;
        Point2f corners[4];
        int area = 0;
        float side_px = 0.0f;
        float score = -1.0e30f;
        int rotation = 0;
        int hamming = 99;
        int threshold = 128;
    };

    struct EdgeLine {
        Point2f p;
        Point2f d;
    };

    int otsuThreshold(const uint8_t* gray, const RectI& roi) const;
    bool decodeCandidate(const uint8_t* gray, const Point2f corners[4],
                         int threshold, int expected_id,
                         int* rotation_out, int* hamming_out,
                         float* border_score_out) const;
    bool solveUnitSquareHomography(const Point2f corners[4], float h[9]) const;
    uint8_t sampleGray(const uint8_t* gray, float x, float y) const;
    RectI clampRect(const RectI& r) const;

    bool fitRefinedEdge(const uint8_t* gray,
                        const Point2f& a, const Point2f& b,
                        EdgeLine& line) const;
    bool intersectLines(const EdgeLine& a, const EdgeLine& b,
                        Point2f& out) const;
    bool refineCorners(const uint8_t* gray,
                       const Point2f coarse[4], Point2f refined[4]) const;

    int _width;
    int _height;
    uint8_t* _visited = nullptr;
    uint32_t* _queue = nullptr;
};
