# Lightweight edge-line refinement

The detector still uses the existing connected-component extrema for candidate discovery and ArUco ID decoding. Refinement runs only after the best candidate has already been accepted.

For each of the four marker edges, the implementation samples several points away from the corners and searches a short distance along the inward normal for the strongest bright-to-dark transition. A total-least-squares line is fitted to those transition points. Adjacent lines are intersected to obtain four refined corners.

The refined quadrilateral is rejected if the fitted edges are poorly aligned, a corner moves too far from its coarse location, area changes excessively, or re-decoding changes marker rotation / worsens Hamming error. In every rejection case the original detected corners are used.
