# AtomS3R White Marker 1D Tracker

A very small vision pipeline for **M5Stack AtomS3R-CAM**.

The current hardware no longer needs visual marker IDs. Marker A always moves in
the upper image band, marker B always moves in the lower image band, and the two
paths do not cross. The runtime therefore tracks two **plain white markers on a
black body** using sparse horizontal scan lines only.

## Current runtime

- GC0308 at **QVGA 320×240**
- native **grayscale/Y8** capture
- marker A = upper white marker
- marker B = lower white marker
- no ArUco decode
- no connected-component search
- no previous-frame image buffer
- no template matching
- no pyramid/local optical-flow fallback
- no Otsu threshold in the normal runtime
- current-frame-only X measurement
- separate BMI270 / future-control task at **200 Hz**
- JSON telemetry at **921600 baud**
- browser flashing through GitHub Pages

The old ArUco/template code is still present in the repository for rollback and
comparison, but it is not used by the current main runtime.

## White-marker detector

For each marker, the firmware samples five horizontal rows through the white
marker and three rows through the nearby black body. At every X pixel it forms a
brightness contrast:

    contrast(X) = white-marker-band brightness
                  - black-body reference brightness

The contrast profile is smoothed over five X pixels. The strongest peak is then
refined with a weighted centroid over a ±35 px window.

The current rows are:

    Marker A:
      white rows      58, 62, 66, 70, 74
      black reference 38, 42, 46

    Marker B:
      white rows      152, 156, 160, 164, 168
      black reference 182, 186, 190

These values were evaluated offline on
WIN_20260921_17_14_06_Pro.mp4 (320×240, 262 frames). The same sparse-line
algorithm detected both markers in **262 / 262 frames**.

The offline test used:

    minimum peak contrast = 55 gray levels
    centroid baseline     = 35 gray levels
    centroid half-window  = 35 px

The minimum observed peak contrast in that video was about 132 for A and 124 for
B, leaving substantial margin above the threshold.

## Why plain white markers

The mechanism already provides marker identity:

    upper lane -> A
    lower lane -> B

So the internal bit pattern of an ArUco marker is unnecessary. During fast
horizontal motion the ArUco pattern can blur enough that ID decoding fails even
though the object is still clearly visible. A plain white region remains a
bright horizontal blob, and its X centroid can still be measured.

## Serial output

Set the monitor to **921600 baud**.

Every ~100 ms the firmware emits one JSON object containing:

- frame number and frame_dt_us
- camera size / failure count
- vision_mode: "white_sparse_1d"
- total and max vision processing time
- 200 Hz IMU loop count / deadline misses / max step time
- IMU acceleration / gyro
- marker A / B:
  - valid
  - source: "white1d" or "none"
  - cx_px
  - fixed nominal cy_px
  - peak_x_px
  - peak_contrast
  - weight_sum
  - bright_width_px
  - cumulative detect_ok / detect_fail
  - per-marker vision_us

For the current mechanism, **cx_px is the primary visual measurement**.

## Configuration

Edit include/app_config.h.

The main white-marker settings are:

    kWhiteMarkerARows
    kWhiteReferenceARows
    kWhiteMarkerBRows
    kWhiteReferenceBRows

    kWhitePeakMinContrast
    kWhiteCentroidBaseline
    kWhiteMinWeightSum
    kWhiteCentroidHalfWindowPx

## Build locally

    python -m pip install platformio
    pio run -e atoms3r_cam

GitHub Actions runs the same PlatformIO build.

## Browser flashing

The Pages workflow builds the firmware, creates the browser-flashable ESP32-S3
image, and deploys the installer.

Chrome or Edge is recommended. To enter AtomS3R-CAM download mode, hold Reset
for about two seconds until the internal green LED lights, then release it.

## Architecture

    Core 0 / high priority
        BMI270 -> 200 Hz task -> future EKF -> controlStep()

    Core 1 / non-deadline vision
        GC0308 current Y8 frame
          -> A sparse rows -> white-vs-black X profile -> weighted X centroid
          -> B sparse rows -> white-vs-black X profile -> weighted X centroid
          -> JSON telemetry

No previous camera frame is copied or retained by the tracker.

## Hardware test

1. Attach the plain white markers to the black body in the tested locations.
2. Flash the newest firmware from the Pages installer.
3. Open serial at 921600 baud.
4. Move through the full mechanism travel repeatedly.
5. Confirm both markers normally remain valid=true with source:"white1d".
6. Check that cx_px follows the physical horizontal motion continuously.
7. Watch peak_contrast; values should remain comfortably above 55.
8. Watch detect_fail; ideally it remains zero.
9. Check vision_total_us and the individual marker vision_us.
10. Confirm imu.misses remains zero while moving at the fastest expected speed.
11. Copy the full log from the Pages serial monitor for analysis.

## Legacy implementation

Earlier revisions used OpenCV-style DICT_4X4_50 markers, local 1-D tracking,
wide template recovery, a two-level local tracker, and ArUco reacquisition.
Those files remain available in repository history and source for comparison,
but the current runtime intentionally avoids them.
