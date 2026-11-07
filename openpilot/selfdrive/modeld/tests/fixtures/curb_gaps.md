# Curb opening and width-noise traces

The daylight fixtures contain boundary-only observations from the supplied route
`000004e7--05f29b7d3e`, recorded on `e3959e6`:

- `curb_opening.json`: 140 frames, segment 13 at 55–60 s followed by segment 14
  at 0–2 s. It includes an established right curb, an outward opening, recovery,
  and subsequent deterioration of the painted-line support.
- `curb_width_noise.json`: 220 frames, segment 23 at 46–57 s. A steady curb has
  isolated width fluctuations that repeatedly restarted the former qualification
  window despite a confident opposite painted line.

`curb_opening_night.json` adds 80 frames from the earlier route
`000004e6--d60f5fff86`, segment 2 at 12–16 s. It protects the preceding steady-curb
fix through a subsequent opening: the lower corridor width must be compared with
the pre-gap conservative boundary, not with a wider median from the same valid
curved corridor. Its extended-hold reference stays frozen during the gap.

Rows contain relative camera frame time, ego speed (m/s), left paint probability,
right edge standard deviation, left painted-line y and right edge y. Coordinates
are lateral metres at the first 18 `ModelConstants.X_IDXS` stations, through
54.1875 m, rounded to six decimals. There are no images, GPS coordinates, device
IDs or vehicle IDs. The fixtures exercise qualification and bounded continuity,
not a vehicle-response simulation.
