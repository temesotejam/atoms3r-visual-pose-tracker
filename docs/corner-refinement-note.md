# Corner refinement validation target

Hardware logs after the tracking-ROI fix show that two-marker tracking is now stable enough for the next milestone, but pose roll/pitch still jump by roughly 10–30 degrees while center and apparent marker size change only slightly.

This branch therefore refines only the already-decoded winning marker candidate:

1. Sample the black/white transition along each outer edge.
2. Fit one line per edge from multiple samples.
3. Intersect adjacent fitted lines to recover four refined corners.
4. Re-decode the marker and accept refined corners only when ID/rotation remain consistent and Hamming error does not worsen.
5. Fall back to the original extrema corners whenever refinement validation fails.

Telemetry exposes `refined:true/false` for each marker so the hardware run can measure acceptance rate and compare pose jitter directly.
