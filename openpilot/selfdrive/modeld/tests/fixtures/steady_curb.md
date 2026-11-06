# Steady curb boundary trace

`steady_curb.json` contains 100 consecutive model frames from 4–9 s of the
supplied `000004e6--d60f5fff86` segment 2. Times are relative to the start of this
excerpt. Coordinates are vehicle-frame lateral metres at the first 18
`ModelConstants.X_IDXS` stations (0–54.1875 m), rounded to five decimals.

Each row contains time, ego speed in m/s, left paint probability, right edge
standard deviation, left painted-line y coordinates and right road-edge y
coordinates. There are no GPS coordinates, images, device IDs or vehicle IDs.

The original policy qualifies the curb, then drops it despite continued valid
geometry because inward width caps feed back into its continuity reference.
After the first second, it is available in 65 of 80 frames. The corrected filter
remains available in all 80. The test retains the original geometric/confidence
thresholds and exercises the actual boundary filter, without a steering or
vehicle-response simulation.
