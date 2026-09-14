#pragma once

#include <stdint.h>

// Minimal subset of OpenCV DICT_4X4_50. We intentionally keep only the two
// known IDs used by this experiment instead of carrying a general dictionary.
// Each 16-bit word is row-major, MSB first. Rotations are 0, 90, 180, 270
// degrees anticlockwise, matching OpenCV's predefined dictionary storage.
namespace aruco4x4 {

struct Code {
    int id;
    uint16_t rotations[4];
};

static constexpr Code kCodes[] = {
    {0, {0xB532u, 0xEB48u, 0x4CADu, 0x12D7u}},
    {1, {0x0F9Au, 0x6547u, 0x59F0u, 0xE2A6u}},
};

static constexpr int kCodeCount = sizeof(kCodes) / sizeof(kCodes[0]);

inline int popcount16(uint16_t v) {
#if defined(__GNUC__)
    return __builtin_popcount(static_cast<unsigned int>(v));
#else
    int n = 0;
    while (v) {
        n += (v & 1u);
        v >>= 1u;
    }
    return n;
#endif
}

inline bool match(uint16_t observed, int expected_id, int max_hamming,
                  int* rotation_out, int* hamming_out) {
    int best_hamming = 17;
    int best_rotation = -1;

    for (int i = 0; i < kCodeCount; ++i) {
        if (expected_id >= 0 && kCodes[i].id != expected_id) {
            continue;
        }
        for (int r = 0; r < 4; ++r) {
            const int d = popcount16(observed ^ kCodes[i].rotations[r]);
            if (d < best_hamming) {
                best_hamming = d;
                best_rotation = r;
            }
        }
    }

    if (rotation_out) *rotation_out = best_rotation;
    if (hamming_out) *hamming_out = best_hamming;
    return best_rotation >= 0 && best_hamming <= max_hamming;
}

} // namespace aruco4x4
