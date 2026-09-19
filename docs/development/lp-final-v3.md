# lp-final-v3: delay-aware approach timing

Parent: lp-final-v2, `33afce3c788c0763c9bf23f343ee9c57a9e39365`.

## Delay source and existing compensation

`lagd` publishes `lateralDelay` at 4 Hz. Only a valid, fresh message with status
`estimated` and delay in 0.15–0.65 s is accepted by the new scheduler. Freshness
uses the message's monotonic timestamp, with a maximum age of 1 second, plus
SubMaster's seen, valid, and alive checks. Raw lateralDelayEstimate is not used.

The existing model action horizon already includes lateralDelay and frame/action
latency. The torque controller uses lateralDelay to align the requested and
measured lateral acceleration. Both existing uses are unchanged.

## Added schedule

    target = clip(0.20 + 0.25 * (estimated_delay - 0.30), 0.17, 0.25)

This is an empirical parameter schedule for the additional lane correction, not
another addition of the full actuator delay. The reference matches the roughly
0.302 s delay in route 245, where the working 0.20 s anticipation felt good.

| Estimated delay | Target anticipation |
| --- | --- |
| 0.15 s | 0.170 s |
| 0.20 s | 0.175 s |
| 0.30 s | 0.200 s |
| 0.302 s | 0.2005 s |
| 0.40 s | 0.225 s |
| 0.50–0.65 s | 0.250 s |

Start at 0.20 s. An unavailable, invalid, unestimated, stale, non-finite, or
out-of-range estimate targets 0.20 s. Parameter changes use a 2 s first-order
response and a maximum rate of 0.01 s of anticipation per second, updated on
successful model outputs at the nominal 20 Hz rate. Thus lost estimates return
gradually to baseline. This scheduling state survives lane-policy resets.
Lane geometry and output curvature receive no new filter or waiting timer.

The 35% offset-reduction cap, centering strength, confidence behavior, one-line
hold, fast release, E2E fallback and curve guards are retained. Every 5 seconds,
`lp-final-v3 timing` logs the delay, approach time, and estimated/baseline source.

## Validation and limits

29 isolated policy tests passed, including all 21 previous tests. Added checks
cover reference behavior, range bounds, stale/invalid/non-finite data, jitter,
loss/recovery slew limits, state lifetime, function argument propagation, and
fallback/correction bounds. The actual policy and action function definitions
were executed with hardware imports isolated; this is not a complete device
startup or model-runtime integration test. Normal runtime test command:

    python -m unittest openpilot.selfdrive.modeld.tests.test_lane_policy

Python compilation and git diff whitespace checks passed.

Recorded-input comparison on all ten route 245 segments covered 11,999 frames.
The anticipation ranged from 0.200000 to 0.200492 s. Relative to v2 on the same
reconstructed E2E inputs, the 95th-percentile absolute curvature difference was
3.20e-8 1/m, maximum 1.20e-5 1/m (threshold/state effects can exceed the direct
parameter difference). No active-state differences occurred; all 1,959 blinker
frames returned exact E2E, and outputs stayed finite and within correction caps.
Raw E2E is reconstructed, not independently logged; replay does not establish
closed-loop improvement or exclude alternative reconstructions. Initial state
and missing route intervals are not fully observable.

The range and gain are conservative trial values, not a vehicle-specific optimum
or a validated universal tune. Other vehicles and actual on-road behavior have
not been validated. This revision intentionally stays very close to v2 for the
reference truck and does not implement confidence-loss handoff changes.
