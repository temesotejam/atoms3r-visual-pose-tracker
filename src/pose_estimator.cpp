#include "pose_estimator.h"

#include <math.h>
#include <string.h>

#include "app_config.h"

namespace {
constexpr float kRadToDeg = 57.29577951308232f;

struct Vec3 {
    float x, y, z;
};

float norm3(const Vec3& v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float dot3(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 scale3(const Vec3& v, float s) {
    return {v.x*s, v.y*s, v.z*s};
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
    return {a.x-b.x, a.y-b.y, a.z-b.z};
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

float edgeLength(const Point2f& a, const Point2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return sqrtf(dx*dx + dy*dy);
}

float relDiff(float a, float b) {
    const float mean = 0.5f * (fabsf(a) + fabsf(b));
    if (mean < 1e-6f) return 0.0f;
    return fabsf(a - b) / mean;
}
}

bool PoseEstimator::solveHomography(const Point2f src[4], const Point2f dst[4],
                                    float h[9]) const {
    float a[8][9] = {};

    for (int i = 0; i < 4; ++i) {
        const float u = src[i].x;
        const float v = src[i].y;
        const float x = dst[i].x;
        const float y = dst[i].y;

        const int r0 = 2*i;
        const int r1 = r0 + 1;

        a[r0][0] = u; a[r0][1] = v; a[r0][2] = 1.0f;
        a[r0][6] = -u*x; a[r0][7] = -v*x;
        a[r0][8] = x;

        a[r1][3] = u; a[r1][4] = v; a[r1][5] = 1.0f;
        a[r1][6] = -u*y; a[r1][7] = -v*y;
        a[r1][8] = y;
    }

    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        float pivot_abs = fabsf(a[pivot][col]);
        for (int row = col + 1; row < 8; ++row) {
            const float v = fabsf(a[row][col]);
            if (v > pivot_abs) {
                pivot_abs = v;
                pivot = row;
            }
        }
        if (pivot_abs < 1e-7f) return false;

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
            if (fabsf(f) < 1e-10f) continue;
            for (int j = col; j < 9; ++j) a[row][j] -= f * a[col][j];
        }
    }

    for (int i = 0; i < 8; ++i) h[i] = a[i][8];
    h[8] = 1.0f;
    return true;
}

bool PoseEstimator::estimate(const Point2f canonical_corners[4],
                             MarkerObservation& out) const {
    // Always compute the robust image-domain measurements first. These remain
    // the preferred visual observations for the current mechanism because the
    // markers move mainly horizontally and stay close to fronto-parallel.
    out.center_x_px = 0.25f * (
        canonical_corners[0].x + canonical_corners[1].x +
        canonical_corners[2].x + canonical_corners[3].x);
    out.center_y_px = 0.25f * (
        canonical_corners[0].y + canonical_corners[1].y +
        canonical_corners[2].y + canonical_corners[3].y);

    const float top = edgeLength(canonical_corners[0], canonical_corners[1]);
    const float right = edgeLength(canonical_corners[1], canonical_corners[2]);
    const float bottom = edgeLength(canonical_corners[2], canonical_corners[3]);
    const float left = edgeLength(canonical_corners[3], canonical_corners[0]);
    out.side_px = 0.25f * (top + right + bottom + left);

    const float dx = canonical_corners[1].x - canonical_corners[0].x;
    const float dy = canonical_corners[1].y - canonical_corners[0].y;
    out.image_angle_deg = atan2f(dy, dx) * kRadToDeg;

    // Mechanism-friendly constrained position estimate. For an approximately
    // fronto-parallel square, apparent side length is a much more stable depth
    // cue than decomposing a noisy planar homography into full 6DoF. Because
    // the marker may be rotated ~90 deg in the image, use an effective focal
    // length rather than assigning fx/fy to specific marker edges.
    if (out.side_px > 1.0f) {
        const float f_eff = sqrtf(_fx * _fy);
        const float z = f_eff * _marker_side / out.side_px;
        out.constrained_z_m = z;
        out.constrained_x_m = (out.center_x_px - _cx) * z / _fx;
        out.constrained_y_m = (out.center_y_px - _cy) * z / _fy;
    }

    // Opposite-edge asymmetry is a simple indicator of how much perspective
    // excitation exists. Near zero means the marker is almost fronto-parallel,
    // where out-of-plane roll/pitch are weakly observable and extremely
    // sensitive to sub-pixel corner noise.
    const float tb_asym = relDiff(top, bottom);
    const float lr_asym = relDiff(left, right);
    out.perspective_asymmetry = fmaxf(tb_asym, lr_asym);
    out.tilt_reliable =
        appcfg::kCameraIntrinsicsCalibrated &&
        out.corner_refined &&
        out.side_px >= appcfg::kTiltMinMarkerSidePx &&
        out.perspective_asymmetry >= appcfg::kTiltMinPerspectiveAsymmetry;

    // Normal control does not use the raw homography 6DoF. Avoid the 8x8
    // solve + decomposition unless a dedicated diagnostic build enables it.
    if (!appcfg::kEnableRawHomographyPose) return true;

    const float half = 0.5f * _marker_side;
    Point2f object_xy[4] = {
        {-half, -half},
        { half, -half},
        { half,  half},
        {-half,  half},
    };

    float h[9];
    if (!solveHomography(object_xy, canonical_corners, h)) {
        // Keep the detection usable: constrained/image-domain measurements are
        // still valid even if the diagnostic raw 6DoF decomposition fails.
        return true;
    }

    // K^-1 H columns.
    Vec3 b1 = {
        (h[0] - _cx*h[6]) / _fx,
        (h[3] - _cy*h[6]) / _fy,
        h[6]
    };
    Vec3 b2 = {
        (h[1] - _cx*h[7]) / _fx,
        (h[4] - _cy*h[7]) / _fy,
        h[7]
    };
    Vec3 b3 = {
        (h[2] - _cx) / _fx,
        (h[5] - _cy) / _fy,
        1.0f
    };

    const float n1 = norm3(b1);
    const float n2 = norm3(b2);
    if (n1 < 1e-6f || n2 < 1e-6f) return true;

    const float lambda = 2.0f / (n1 + n2);
    Vec3 r1 = scale3(b1, lambda);
    Vec3 r2 = scale3(b2, lambda);
    Vec3 t  = scale3(b3, lambda);

    const float r1n = norm3(r1);
    if (r1n < 1e-6f) return true;
    r1 = scale3(r1, 1.0f / r1n);

    r2 = sub3(r2, scale3(r1, dot3(r1, r2)));
    const float r2n = norm3(r2);
    if (r2n < 1e-6f) return true;
    r2 = scale3(r2, 1.0f / r2n);

    Vec3 r3 = cross3(r1, r2);
    const float r3n = norm3(r3);
    if (r3n < 1e-6f) return true;
    r3 = scale3(r3, 1.0f / r3n);

    if (t.z < 0.0f) {
        t = scale3(t, -1.0f);
        r1 = scale3(r1, -1.0f);
        r2 = scale3(r2, -1.0f);
        r3 = cross3(r1, r2);
    }

    out.x_m = t.x;
    out.y_m = t.y;
    out.z_m = t.z;

    const float r20 = r1.z;
    const float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -r20)));
    const float cp = cosf(pitch);

    float roll = 0.0f;
    float yaw = 0.0f;
    if (fabsf(cp) > 1e-5f) {
        yaw  = atan2f(r1.y, r1.x);
        roll = atan2f(r2.z, r3.z);
    } else {
        yaw = atan2f(-r2.x, r2.y);
        roll = 0.0f;
    }

    out.roll_deg = roll * kRadToDeg;
    out.pitch_deg = pitch * kRadToDeg;
    out.yaw_deg = yaw * kRadToDeg;

    return true;
}
