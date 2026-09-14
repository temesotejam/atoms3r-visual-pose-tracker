# Hardware test procedure

1. Flash the firmware produced from this branch.
2. Hold both markers nearly stationary for 10 seconds at about the same range used in the previous hardware log.
3. Move both markers slowly through the normal vertical travel while keeping them visible.
4. Save serial JSON.

Check:
- `refined` acceptance rate for marker A/B.
- A/B valid and both-valid tracking rate.
- Vision processing time and IMU deadline misses.
- Standard deviation/range of `roll_deg`, `pitch_deg`, `yaw_deg` during the stationary interval.
- Compare those values against `cx_px`, `cy_px`, `side_px`, and `image_angle_deg` to distinguish true motion from pose-solver corner jitter.
