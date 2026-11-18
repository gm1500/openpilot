# Speed-dependent curve-reversal pause POC

Based on `batmobile` at `cf97ef9297b1dae54d215918b190288ab870c663`, including
the shared road-edge qualification and bounded painted-line confidence pauses.
The main branch retains its fixed 0.15 s curve-reversal pause.

This POC linearly interpolates the curve-reversal pause from 0.30 s at 0 km/h
to 0.05 s at 100 km/h, clamping outside that range. The 20 Hz model loop rounds
the requested duration to the nearest 0.05 s frame:

| Vehicle speed | Pause | Model frames |
| --- | ---: | ---: |
| 0 km/h | 0.30 s | 6 |
| 20 km/h | 0.25 s | 5 |
| 40 km/h | 0.20 s | 4 |
| 60 km/h | 0.15 s | 3 |
| 80 km/h | 0.10 s | 2 |
| 100 km/h and above | 0.05 s | 1 |

The speed is sampled when a qualifying correction reversal starts the pause.
An existing countdown cannot be extended or shortened by later speed changes
or sign chatter. The most recent reversal direction is still remembered.

The curve threshold, reversal deadband, separate 0.35 s E2E turn-direction guard,
centering gains, correction limits, motion history and edge confidence rules are
unchanged. This is a temporary release of the centering correction on a curve
reversal, not a steering-actuator delay or an edge-detection hold timer.

Validation of the updated endpoint: 142 focused road-edge and painted-width
memory tests pass, including exact pause durations, speed clamping, monotonic
interpolation, and speed/sign changes during a pause. Lint, compilation and
whitespace checks pass. This change affects only the curve-reversal pause;
recorded-input selection tests do not establish improved vehicle handling.
