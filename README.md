# AtomS3R Visual Pose Tracker

A deliberately small vision pipeline for **M5Stack AtomS3R-CAM**.

The first milestone tracks two known **OpenCV ArUco `DICT_4X4_50` markers
(ID 0 and ID 1)** while preserving a separate **200 Hz IMU/control task**.
The real mounting has now confirmed that both markers move mainly
**horizontally**: marker A stays in an upper image band and marker B in a lower
image band. The firmware therefore uses two horizontal acquisition bands and
switches to a predicted local ROI after lock.

## What is implemented

- GC0308 at **QVGA 320×240**
- native **grayscale/Y8** capture
- two independent marker trackers
- upper/lower horizontal acquisition bands
- predicted local ROI after acquisition
- ultra-light horizontal-only 1D tracking as the normal fast path
- only three Y sample rows, X-only search, and no per-frame corner re-fit
- two-level local block tracking retained as a safety fallback
- ArUco fallback when both local trackers are not trustworthy
- lane-wide acquisition duty-cycled while fully lost to leave CPU headroom
- expanding recovery ROI after misses
- fallback to band acquisition instead of immediate full-frame search
- minimal two-ID ArUco dictionary (`DICT_4X4_50`, IDs 0 and 1)
- 4-corner extraction and homography-based cell sampling
- lightweight edge-line corner refinement
- constrained fronto-parallel position estimate for the mainly-horizontal mechanism
- raw homography 6DoF kept as an optional diagnostic build path (off by default)
- tilt-observability telemetry (`perspective_asymmetry`, `tilt_reliable`)
- BMI270 high-rate task at **200 Hz**
- control integration hook that is intentionally independent of vision
- JSON telemetry at **921600 baud**
- GitHub Actions PlatformIO compile test
- GitHub Pages + ESP Web Tools browser flasher
- versioned browser-flash BIN paths to avoid stale cached firmware
- Web Serial monitor at 921600 baud with one-click log copy
- printable test markers

## Important: current pose accuracy

For the current mechanism, `center_x_px` is the primary image-domain motion
measurement. `center_y_px` should remain nearly constant and is useful as a
consistency check. `image_angle_deg` is also stable enough for bring-up.

The new `constrained_x_m / constrained_y_m / constrained_z_m` fields use marker
center, mean apparent side length, the configured physical marker size, and the
current approximate focal length. For this mostly fronto-parallel mechanism,
that constrained estimate is intentionally preferred over raw planar
homography roll/pitch.

The current `x_m/y_m/z_m` and raw 3D Euler angles still use approximate camera
intrinsics in `include/app_config.h`. They are **not calibration-grade yet**.
The latest hardware logs show that roll/pitch can jump strongly even while
marker center and size are almost stationary. Therefore `tilt_reliable` remains
false until the real GC0308 intrinsics are calibrated and the frame also has
enough perspective asymmetry.

## Default marker setup

- Dictionary: OpenCV `DICT_4X4_50`
- Marker A: ID `0`, upper horizontal band
- Marker B: ID `1`, lower horizontal band
- Printed black-square side: `6 mm`

The Pages site includes a printable marker sheet.

## Configuration

Edit `include/app_config.h`.

The values most likely to change first are:

```cpp
kMarkerSideM

kLaneAX
kLaneAY
kLaneAW
kLaneAH

kLaneBX
kLaneBY
kLaneBW
kLaneBH

kFxPx
kFyPx
kCxPx
kCyPx
```

The current acquisition geometry is approximately:

```text
320 px wide
+--------------------------------------+
|           Marker A band              |
|     <------ horizontal travel ---->   |
+--------------------------------------+
|                                      |
+--------------------------------------+
|           Marker B band              |
|     <------ horizontal travel ---->   |
+--------------------------------------+
```

After a marker is found, the normal path uses the mechanism constraint directly:
the marker is assumed to translate only in X over a short range while Y, scale,
and image rotation stay effectively fixed. Three rows through the known marker
are sampled and matched only along X around the velocity prediction. A successful
1D match translates the already-validated marker geometry without re-fitting four
corners or solving a homography.

If that 1D match is not trustworthy, the previous two-level X/Y block tracker is
used as a safety fallback, and only then is ArUco decoding attempted. Once fully
lost, expensive lane-wide ArUco acquisition is duty-cycled rather than run on
every camera frame.

## Serial output

Set the monitor to **921600 baud**.

Every ~100 ms the firmware emits one JSON object containing:

- camera/frame counters and `frame_dt_us`
- total vision processing time
- max observed vision time
- `motion_axis: "horizontal"`
- 200 Hz IMU loop count
- IMU deadline misses
- max IMU step execution time
- acceleration / gyro
- marker validity/state and tracking source (`1d` / `pyramid` / `aruco` / `none`)
- local tracking SAD and cumulative 1D/flow/decode/reacquire counters
- pixel position and image-plane angle
- constrained XYZ estimate
- perspective asymmetry / tilt reliability
- raw approximate camera-relative XYZ and Euler angles

## Build locally

```bash
python -m pip install platformio
pio run -e atoms3r_cam
```

The CI workflow runs the same PlatformIO build on GitHub.

## Browser flashing

The `pages.yml` workflow builds the firmware, merges bootloader + partition
table + application into one ESP32-S3 image, and deploys the `web/` directory
to GitHub Pages.

The page uses ESP Web Tools. Chrome or Edge is recommended.

To enter AtomS3R-CAM download mode, hold Reset for about two seconds until the
internal green LED lights, then release it.

## Architecture

```text
Core 0 / high priority
    BMI270 -> 200 Hz task -> future EKF -> controlStep()

Core 1 / non-deadline vision
    GC0308 frame
      -> Marker A / Marker B
      -> constrained 1D X tracking in normal operation
      -> two-level local motion tracking only as fallback
      -> ArUco acquire/reacquire only when needed
      -> constrained horizontal position measurement
      -> optional raw pose diagnostics
      -> future timestamped EKF correction
```

Vision is not allowed to block the 200 Hz loop. A slow or missed frame should
reduce visual update rate, not stall control.

## Hardware test

1. Print ID 0 and ID 1 at the configured physical size.
2. Flash from the Pages installer.
3. Open serial at 921600 baud.
4. Keep both markers still and confirm `valid=true`; the initial lock should report `source:"aruco"`.
5. Move each marker horizontally and confirm successful following frames normally report `source:"1d"`.
6. Watch `track_sad`, `track1d_fail`, `flow_fail`, and `reacquire` while increasing motion speed.
7. Confirm `cx_px` follows the motion while `cy_px` remains comparatively constant.
8. Cover one marker and confirm recovery returns through ArUco acquisition.
9. Watch `imu.misses` and `imu.max_step_us` while doing all of the above.
10. Use the Pages serial monitor to copy the full test log for analysis.
11. Calibrate the real camera before enabling metric visual corrections in the EKF.

## Sources used for the initial hardware assumptions

- M5Stack AtomS3R-CAM documentation for GC0308 pins, 8 MB PSRAM, camera power
  enable, QVGA examples, and supported development environments.
- Espressif `esp32-camera` GC0308 implementation for native grayscale/Y8
  support on ESP32-S3.
- OpenCV's predefined `DICT_4X4_50` bytes for IDs 0 and 1.
- ESP Web Tools documentation for browser flashing.
