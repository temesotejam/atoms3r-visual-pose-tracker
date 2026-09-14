#pragma once

#include <stdint.h>
#include "vision_types.h"

class MarkerDetector {
public:
    MarkerDetector(int frame_width, int frame_height);
    ~MarkerDetector();

    bool begin();

    // Returns a geometrically ordered TL,TR,BR,BL candidate. The caller then
    // rotates those corners into canonical marker orientation using rotation.
    bool detect(const uint8_t* gray, const RectI& roi, int expected_id,
                MarkerObservation& out);

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

    int otsuThreshold(const uint8_t* gray, const RectI& roi) const;
    bool decodeCandidate(const uint8_t* gray, const Point2f corners[4],
                         int threshold, int expected_id,
                         int* rotation_out, int* hamming_out,
                         float* border_score_out) const;
    bool solveUnitSquareHomography(const Point2f corners[4], float h[9]) const;
    uint8_t sampleGray(const uint8_t* gray, float x, float y) const;
    RectI clampRect(const RectI& r) const;

    int _width;
    int _height;
    uint8_t* _visited = nullptr;
    uint32_t* _queue = nullptr;
};
