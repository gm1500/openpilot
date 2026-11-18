# Lead identity and history ownership POC

This change gives vision tracks process-local IDs and history generations, and
uses those identities to prevent a newly observed lead from borrowing another
vehicle's speed history. Identity uncertainty does not delay lead acceptance:
a new track immediately publishes its current accepted model speed and range.

## Behavior

- IDs follow associated observations across model-slot swaps. Duplicate model
  predictions can reference the same track and do not count as two measurements.
- IDs are never reused within a process. Reinitializing an estimator in place
  advances its generation, invalidating consumers of its discarded history.
- Association transports relative position with ego motion and, when fresh
  calibrated device motion is available, compensates for ego yaw. Missing
  optional motion does not affect radar health or lead acceptance.
- A joint range and model-speed discontinuity rejects an old association even
  inside the ordinary five-metre gate. Range changes alone or model-speed changes
  alone do not automatically declare a new vehicle.
- Near-tied competing assignments start fresh histories instead of arbitrarily
  giving one branch a mature track or copying that history into both branches.
- New IDs do not inherit the previous published speed through split bridges or
  uncertain handoffs. Opening-speed recovery and retained range/future-motion
  estimators reset when their owner changes or becomes ambiguous. Future and
  confidence filters also verify the history generation.
- Existing lead presence, raw acceleration, radar IDs and planner limits retain
  their existing owners. No lead is suppressed because its identity is unknown.
- The existing `lead-pre` diagnostic includes track ID, generation and association
  reason. It is still rate-limited to 1 Hz; offline replay captures every frame.

## Validation, 2026-11-18

Baseline: `e5e83e4266571b9595f739e3437d282c5e68413f`.

The host suite ran 266 tests: 265 passed and one optional private approach
fixture was unavailable. Tests use production Python estimators and cereal,
with host substitutes for unavailable IPC/parameter-store components. Targeted
coverage includes slot swaps, duplicate heads, ambiguous splits/merges, nearby
cut-ins, continuous braking, missing observations, gyro causality, generation
resets and revocation of downstream state. Changed Python files pass Ruff.

The full estimator was reconstructed from raw recorded model messages at logged
radar publication times, then fed to independent native MPC instances at logged
planner publications, with full available segment warmup. Eleven recorded
windows and four known-identity synthetic sequences completed without solver
failures. This is a fixed-ego comparison, not a friction-actuator replay.

### Known cut-in: old lead 30 m/s at 75 m, new lead 18 m/s at 72 m

| Sequence | First-frame published speed, baseline / POC | Time to requested acceleration below -0.5 m/s², baseline / POC |
| --- | --- | --- |
| Both slots move to new lead | 29.928 / 18.000 m/s | 0.15 / 0.00 s |
| New closer lead, old lead retained in other slot | 30.000 / 18.000 m/s | 0.70 / 0.00 s |

Pure slot swaps and continuously braking single-lead sequences are unchanged.

A separate simple closed-loop simulation uses the actual estimator and native
MPC with 0.5 s command delay, 0.2 s lag and 15 s after lead appearance:

| Sequence | Minimum true gap, baseline / POC | Peak applied acceleration, baseline / POC |
| --- | --- | --- |
| New single lead | 32.579 / 33.398 m | -3.221 / -3.135 m/s² |
| Split cut-in | 28.392 / 33.398 m | -3.427 / -3.135 m/s² |

These two constructed cases support the intended earlier response and reduced
late braking. They are not real-world gap measurements. Initial requested
acceleration can change more sharply; the split case increases peak command
jerk even though the lagged simulated plant has lower peak braking.

### Recorded comparison limits and regressions

- The recorded semi cut-in, steady following, clean return and departure events
  A/B/D have unchanged requested acceleration in the compared windows.
- Departure C has 3.56% more sampled deceleration area; the largest additional
  requested deceleration is 0.036 m/s².
- The clean lead-disappearing event has 11.23% more sampled deceleration area
  (0.6216 to 0.6914 m/s), with up to 0.075 m/s² additional requested slowing.
- The lead-exit/reveal event has unchanged negative area but a brief 0.058 m/s²
  downward command difference. A separate no-departure bookmark has less slowing.
- The older slower-lead approach was recorded on commit `8129eaa177fb1df06034851aea0b31ad996982f9`.
  Rebuilding the current baseline does not reproduce that original drive's
  estimator output; it is only a comparison between today's baseline and this
  candidate on those observations.

No recorded response window changes the derived-below-55-km/h classification,
but the replay does not implement the complete E2E/CEM policy. These results do
not demonstrate friction-brake elimination or uniformly improved comfort.

## Identity limits

An ID represents an association hypothesis, not proof of physical vehicle
identity. Simultaneous model range/speed corrections can cause a new ID for the
same car; a distinct vehicle with sufficiently similar geometry can retain an
old association. Association uses a best-versus-runner-up margin, not a calibrated
identity probability. Predicted motion and direct model velocity are correlated
evidence. Yaw compensation applies to association, not the entire range Kalman
model. Camera-labelled identity accuracy and broader cut-in validation remain
necessary before treating this as a general solution.
