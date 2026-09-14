#include "marker_detector.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"

#include "app_config.h"
#include "aruco_dictionary.h"

namespace {

float dist(const Point2f& a, const Point2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return sqrtf(dx*dx + dy*dy);
}

float polygonArea4(const Point2f p[4]) {
    float a = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) & 3;
        a += p[i].x*p[j].y - p[j].x*p[i].y;
    }
    return 0.5f * fabsf(a);
}

bool pointsDistinct(const Point2f p[4]) {
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            const float dx = p[i].x - p[j].x;
            const float dy = p[i].y - p[j].y;
            if (dx*dx + dy*dy < 16.0f) return false;
        }
    }
    return true;
}

} // namespace

MarkerDetector::MarkerDetector(int frame_width, int frame_height)
    : _width(frame_width), _height(frame_height) {}

MarkerDetector::~MarkerDetector() {
    if (_visited) free(_visited);
    if (_queue) free(_queue);
}

bool MarkerDetector::begin() {
    const size_t pixels = static_cast<size_t>(_width) * _height;

    _visited = static_cast<uint8_t*>(
        heap_caps_malloc(pixels, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    _queue = static_cast<uint32_t*>(
        heap_caps_malloc(pixels * sizeof(uint32_t),
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (!_visited) _visited = static_cast<uint8_t*>(malloc(pixels));
    if (!_queue) _queue = static_cast<uint32_t*>(malloc(pixels * sizeof(uint32_t)));

    return _visited && _queue;
}

RectI MarkerDetector::clampRect(const RectI& in) const {
    RectI r = in;
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > _width) r.w = _width - r.x;
    if (r.y + r.h > _height) r.h = _height - r.y;
    if (r.w < 0) r.w = 0;
    if (r.h < 0) r.h = 0;
    return r;
}

int MarkerDetector::otsuThreshold(const uint8_t* gray, const RectI& input_roi) const {
    const RectI roi = clampRect(input_roi);
    uint32_t hist[256] = {};

    uint32_t count = 0;
    uint64_t sum = 0;
    for (int y = roi.y; y < roi.y + roi.h; ++y) {
        const uint8_t* row = gray + y * _width;
        for (int x = roi.x; x < roi.x + roi.w; ++x) {
            const uint8_t v = row[x];
            ++hist[v];
            sum += v;
            ++count;
        }
    }
    if (!count) return 128;

    uint64_t sum_bg = 0;
    uint32_t weight_bg = 0;
    float best = -1.0f;
    int threshold = 128;

    for (int t = 0; t < 256; ++t) {
        weight_bg += hist[t];
        if (!weight_bg) continue;
        const uint32_t weight_fg = count - weight_bg;
        if (!weight_fg) break;

        sum_bg += static_cast<uint64_t>(t) * hist[t];
        const float mean_bg = static_cast<float>(sum_bg) / weight_bg;
        const float mean_fg =
            static_cast<float>(sum - sum_bg) / weight_fg;
        const float d = mean_bg - mean_fg;
        const float between =
            static_cast<float>(weight_bg) * weight_fg * d * d;

        if (between > best) {
            best = between;
            threshold = t;
        }
    }

    // Avoid pathological all-white/all-black thresholds during bring-up.
    if (threshold < 24) threshold = 96;
    if (threshold > 232) threshold = 180;
    return threshold;
}

bool MarkerDetector::solveUnitSquareHomography(const Point2f corners[4],
                                                float h[9]) const {
    const Point2f src[4] = {{0,0}, {1,0}, {1,1}, {0,1}};
    float a[8][9] = {};

    for (int i = 0; i < 4; ++i) {
        const float u = src[i].x;
        const float v = src[i].y;
        const float x = corners[i].x;
        const float y = corners[i].y;
        const int r0 = 2*i;
        const int r1 = r0 + 1;

        a[r0][0]=u; a[r0][1]=v; a[r0][2]=1;
        a[r0][6]=-u*x; a[r0][7]=-v*x; a[r0][8]=x;

        a[r1][3]=u; a[r1][4]=v; a[r1][5]=1;
        a[r1][6]=-u*y; a[r1][7]=-v*y; a[r1][8]=y;
    }

    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        float maxv = fabsf(a[col][col]);
        for (int row = col + 1; row < 8; ++row) {
            const float v = fabsf(a[row][col]);
            if (v > maxv) { maxv = v; pivot = row; }
        }
        if (maxv < 1e-7f) return false;

        if (pivot != col) {
            for (int j = col; j < 9; ++j) {
                const float tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
        }

        const float inv = 1.0f / a[col][col];
        for (int j = col; j < 9; ++j) a[col][j] *= inv;

        for (int row = 0; row < 8; ++row) {
            if (row == col) continue;
            const float f = a[row][col];
            for (int j = col; j < 9; ++j) a[row][j] -= f*a[col][j];
        }
    }

    for (int i = 0; i < 8; ++i) h[i] = a[i][8];
    h[8] = 1.0f;
    return true;
}

uint8_t MarkerDetector::sampleGray(const uint8_t* gray, float x, float y) const {
    int cx = static_cast<int>(x + 0.5f);
    int cy = static_cast<int>(y + 0.5f);
    if (cx < 1) cx = 1;
    if (cy < 1) cy = 1;
    if (cx > _width - 2) cx = _width - 2;
    if (cy > _height - 2) cy = _height - 2;

    int sum = 0;
    for (int yy = cy - 1; yy <= cy + 1; ++yy) {
        for (int xx = cx - 1; xx <= cx + 1; ++xx) {
            sum += gray[yy*_width + xx];
        }
    }
    return static_cast<uint8_t>(sum / 9);
}

bool MarkerDetector::decodeCandidate(const uint8_t* gray,
                                     const Point2f corners[4],
                                     int threshold, int expected_id,
                                     int* rotation_out, int* hamming_out,
                                     float* border_score_out) const {
    float h[9];
    if (!solveUnitSquareHomography(corners, h)) return false;

    bool cells[6][6] = {};
    int black_border = 0;
    int border_cells = 0;

    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            const float u = (col + 0.5f) / 6.0f;
            const float v = (row + 0.5f) / 6.0f;
            const float den = h[6]*u + h[7]*v + 1.0f;
            if (fabsf(den) < 1e-7f) return false;

            const float x = (h[0]*u + h[1]*v + h[2]) / den;
            const float y = (h[3]*u + h[4]*v + h[5]) / den;
            const bool white = sampleGray(gray, x, y) >= threshold;
            cells[row][col] = white;

            if (row == 0 || row == 5 || col == 0 || col == 5) {
                ++border_cells;
                if (!white) ++black_border;
            }
        }
    }

    const float border_score =
        border_cells ? static_cast<float>(black_border) / border_cells : 0.0f;
    if (border_score_out) *border_score_out = border_score;
    if (black_border < 18) return false; // 20 border cells total.

    uint16_t payload = 0;
    for (int row = 1; row <= 4; ++row) {
        for (int col = 1; col <= 4; ++col) {
            payload = static_cast<uint16_t>((payload << 1) |
                                            (cells[row][col] ? 1u : 0u));
        }
    }

    return aruco4x4::match(payload, expected_id, appcfg::kMaxHammingError,
                          rotation_out, hamming_out);
}

bool MarkerDetector::detect(const uint8_t* gray, const RectI& input_roi,
                            int expected_id, MarkerObservation& out) {
    out.valid = false;
    if (!gray || !_visited || !_queue) return false;

    const RectI roi = clampRect(input_roi);
    if (roi.w < appcfg::kMinMarkerSidePx ||
        roi.h < appcfg::kMinMarkerSidePx) {
        return false;
    }

    memset(_visited, 0, static_cast<size_t>(_width) * _height);
    const int threshold = otsuThreshold(gray, roi);

    Candidate best;

    for (int sy = roi.y; sy < roi.y + roi.h; ++sy) {
        for (int sx = roi.x; sx < roi.x + roi.w; ++sx) {
            const uint32_t start = static_cast<uint32_t>(sy*_width + sx);
            if (_visited[start] || gray[start] >= threshold) continue;

            size_t head = 0;
            size_t tail = 0;
            _queue[tail++] = start;
            _visited[start] = 1;

            int area = 0;
            int min_x = sx, max_x = sx, min_y = sy, max_y = sy;

            float min_sum = 1e30f, max_sum = -1e30f;
            float min_diff = 1e30f, max_diff = -1e30f;
            Point2f tl{}, tr{}, br{}, bl{};

            while (head < tail) {
                const uint32_t idx = _queue[head++];
                const int y = static_cast<int>(idx / _width);
                const int x = static_cast<int>(idx - y*_width);
                ++area;

                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;

                const float sum = static_cast<float>(x + y);
                const float diff = static_cast<float>(x - y);
                if (sum < min_sum) { min_sum = sum; tl = {(float)x,(float)y}; }
                if (sum > max_sum) { max_sum = sum; br = {(float)x,(float)y}; }
                if (diff > max_diff) { max_diff = diff; tr = {(float)x,(float)y}; }
                if (diff < min_diff) { min_diff = diff; bl = {(float)x,(float)y}; }

                const int nx[4] = {x-1, x+1, x, x};
                const int ny[4] = {y, y, y-1, y+1};
                for (int k = 0; k < 4; ++k) {
                    const int xx = nx[k];
                    const int yy = ny[k];
                    if (xx < roi.x || xx >= roi.x + roi.w ||
                        yy < roi.y || yy >= roi.y + roi.h) {
                        continue;
                    }
                    const uint32_t ni = static_cast<uint32_t>(yy*_width + xx);
                    if (_visited[ni] || gray[ni] >= threshold) continue;
                    _visited[ni] = 1;
                    _queue[tail++] = ni;
                }
            }

            if (area < appcfg::kMinBlackComponentAreaPx) continue;

            const int bw = max_x - min_x + 1;
            const int bh = max_y - min_y + 1;
            if (bw < appcfg::kMinMarkerSidePx || bh < appcfg::kMinMarkerSidePx ||
                bw > appcfg::kMaxMarkerSidePx || bh > appcfg::kMaxMarkerSidePx) {
                continue;
            }

            const float aspect = static_cast<float>(bw) / bh;
            if (aspect < 0.48f || aspect > 2.08f) continue;

            const float fill = static_cast<float>(area) / (bw*bh);
            if (fill < 0.12f || fill > 0.92f) continue;

            Point2f corners[4] = {tl, tr, br, bl};
            if (!pointsDistinct(corners)) continue;

            const float top = dist(tl, tr);
            const float right = dist(tr, br);
            const float bottom = dist(br, bl);
            const float left = dist(bl, tl);
            const float side = 0.25f*(top + right + bottom + left);
            if (side < appcfg::kMinMarkerSidePx ||
                side > appcfg::kMaxMarkerSidePx) continue;

            const float quad_area = polygonArea4(corners);
            if (quad_area < 0.35f * bw * bh) continue;

            int rotation = -1;
            int hamming = 99;
            float border_score = 0.0f;
            if (!decodeCandidate(gray, corners, threshold, expected_id,
                                 &rotation, &hamming, &border_score)) {
                continue;
            }

            // Prefer clean border geometry and larger markers. Hamming error is
            // heavily penalized so an exact expected ID wins.
            const float score =
                2.0f*border_score + 0.003f*quad_area - 1.5f*hamming;

            if (score > best.score) {
                best.valid = true;
                best.score = score;
                best.area = area;
                best.side_px = side;
                best.rotation = rotation;
                best.hamming = hamming;
                best.threshold = threshold;
                for (int i = 0; i < 4; ++i) best.corners[i] = corners[i];
            }
        }
    }

    if (!best.valid) return false;

    out.valid = true;
    out.id = expected_id;
    out.rotation = best.rotation;
    out.hamming = best.hamming;
    out.side_px = best.side_px;
    out.quality = fmaxf(0.0f, fminf(1.0f,
        1.0f - 0.25f*best.hamming));

    for (int i = 0; i < 4; ++i) out.corners[i] = best.corners[i];
    return true;
}
