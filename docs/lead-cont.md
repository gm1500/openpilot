# lead-cont: preserve established duplicate-lead history

Based on `brake-map` at `43e9ef63912723510fd7dae20538817f3979393f`.

An established distance track can be shared by the model's two lead hypotheses.
A brief disagreement beyond the grouping threshold previously split that group,
assigned its warm history to one hypothesis, and forced the other back to model
velocity. The lead could remain accepted throughout this discontinuity.

This change adds grouping hysteresis:

| Situation | Maximum distance difference | Maximum lateral difference |
| --- | ---: | ---: |
| First joining hypotheses | 1.5 m | 0.35 m |
| Retaining a ready, already shared track | 3.0 m | 0.5 m |

The wider release limits apply only when both observations still independently
pass the existing association check against that shared track. A separation
beyond either release limit, an association failure, or an unready track uses
the original grouping behavior. Independently established tracks do not gain
the wider joining limits. A retained pair is still one correlated observation
and receives one Kalman update.

The model braking/closing-time guards and normal fallback remain in place.
Lead acceptance, raw published distance and acceleration, filtering gains,
planner policy, actuator tuning, and the opendbc revision are unchanged.

## Recorded-input comparison

Replay of all 7,200 radar/model frames from the six supplied highway segments
reproduces the following changes in segment 12:

| Time | Previous one-frame speed change | With continuity hysteresis |
| --- | ---: | ---: |
| 56.45 s | -6.290 km/h | -0.032 km/h |
| 57.15 s | -6.366 km/h | -0.023 km/h |

The first event exceeded both original grouping gates (2.424 m longitudinal
and 0.438 m lateral). The second exceeded the distance gate. Both remain in
distance mode with the change, removing four fallback frames in total.

The segment-15 reset after a much larger gap jump still occurs. Its later
steady-following brake cycles have the same lead-speed output within recorded
float rounding. This patch targets the sharp split/fallback discontinuities;
it does not claim to resolve all slower following oscillation.

35 focused estimator/adaptive unit tests pass, including five new regressions
covering temporary splits, original joining limits, release on true separation,
failed association, and immediate model braking/closing-time guards:

```sh
python3 -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive
```

Replay holds the recorded vehicle motion and model observations fixed. It
validates the changed lead-speed input, not a new planner/brake trajectory or
road performance. No native MPC or full-stack vehicle simulation was run.
