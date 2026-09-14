# AtomS3R Visual Pose Tracker

A deliberately small vision pipeline for **M5Stack AtomS3R-CAM**.

The first milestone tracks two known **OpenCV ArUco `DICT_4X4_50` markers
(ID 0 and ID 1)** while preserving a separate **200 Hz IMU/control task**.
It is designed for mechanisms where each marker moves mainly in a known
vertical lane, so the firmware does not pay the cost of a general full-frame
marker search on every image.

## What is implemented

- GC0308 at **QVGA 320×240**
- native **grayscale/Y8** capture
- two independent marker trackers
- initial lane acquisition
- predicted local ROI after acquisition
- expanding recovery ROI after misses
- fallback to lane acquisition instead of immediate full-frame search
- minimal two-ID ArUco dictionary (`DICT_4X4_50`, IDs 0 and 1)
- 4-corner extraction and homography-based cell sampling
- lightweight planar pose estimate
- BMI270 high-rate task at **200 Hz**
- control integration hook that is intentionally independent of vision
- JSON telemetry at **921600 baud**
- GitHub Actions PlatformIO compile test
- GitHub Pages + ESP Web Tools browser flasher
- printable test markers

## Important: current pose accuracy

`center_y_px` and `image_angle_deg` are the most useful bring-up signals.

The current `x_m/y_m/z_m` and 3D Euler angles use approximate camera
intrinsics in `include/app_config.h`. They are **not calibration-grade yet**.
Before feeding metric camera pose into an EKF, calibrate the actual GC0308
module and replace `fx/fy/cx/cy`.

The corner detector is also intentionally lightweight. The first real-hardware
test should determine whether motion blur and the black-border connected
component are clean enough before adding more complexity.

## Default marker setup

- Dictionary: OpenCV `DICT_4X4_50`
- Marker A: ID `0`
- Marker B: ID `1`
- Printed black-square side: `50 mm`

The Pages site includes a printable marker sheet.

## Configuration

Edit `include/app_config.h`.

The values most likely to change first are:

```cpp
kMarkerSideM

kLaneAX
kLaneAW

kLaneBX
kLaneBW

kFxPx
kFyPx
kCxPx
kCyPx
```

The initial lane defaults overlap slightly:

```text
0                                      319
|------------- A -------------|
                  |------------- B -------------|
```

After a marker is found, only a predicted ROI around its previous
position/velocity is searched. After several misses the tracker returns to its
configured lane.

## Serial output

Set the monitor to **921600 baud**.

Every ~100 ms the firmware emits one JSON object containing:

- camera/frame counters
- total vision processing time
- max observed vision time
- 200 Hz IMU loop count
- IMU deadline misses
- max IMU step execution time
- acceleration / gyro
- marker validity/state
- pixel position
- pixel angle
- approximate camera-relative XYZ and Euler angles

Example fields:

```json
{
  "vision_total_us": 8300,
  "imu": {
    "loops": 2450,
    "misses": 0,
    "max_step_us": 740
  },
  "marker_a": {
    "valid": true,
    "state": "track",
    "cy_px": 122.4,
    "image_angle_deg": 1.7,
    "z_m": 0.42
  }
}
```

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
      -> Marker A predicted ROI
      -> Marker B predicted ROI
      -> homography / ID check / pose
      -> future timestamped EKF correction
```

Vision is not allowed to block the 200 Hz loop. A slow or missed frame should
reduce visual update rate, not stall control.

## First hardware test

1. Print ID 0 and ID 1 at the configured physical size.
2. Flash from the Pages installer.
3. Open serial at 921600 baud.
4. Keep both markers still and confirm `valid=true`.
5. Move each marker only vertically and confirm the tracker stays in `track`.
6. Cover one marker and confirm `track -> recover -> acquire`.
7. Watch `imu.misses` and `imu.max_step_us` while doing all of the above.
8. Only after this passes, tune exposure / marker size and perform camera
   calibration.

## Sources used for the initial hardware assumptions

- M5Stack AtomS3R-CAM documentation for GC0308 pins, 8 MB PSRAM, camera power
  enable, QVGA examples, and supported development environments.
- Espressif `esp32-camera` GC0308 implementation for native grayscale/Y8
  support on ESP32-S3.
- OpenCV's predefined `DICT_4X4_50` bytes for IDs 0 and 1.
- ESP Web Tools documentation for browser flashing.
