# boat-anchor

Based on comma master `ec95db3f1fa19f76940497fedfdb62e09ea19912`.
Seven feature commits carry the working tune and policies onto that base.

| Category | Behavior |
| --- | --- |
| Sierra Tuning | Preserve the Sierra/Silverado torque controller, fingerprint-specific road load, 100 Nm bounded coast allowance, final-stop calibration, Ki 0.050 at 40–60 km/h and 0.5 s longitudinal delay. The unused pitch input is removed; torque requests are unchanged. Other platforms retain a wheel-radius default. |
| Nudgeless | A valid lane-change request can proceed without a steering nudge; existing speed, blinker and blind-spot checks remain. |
| Lane policy | Blend a bounded centering correction from confident inner lane lines with the model path. Retain brief learned-width support with one line, and release for lane changes, invalid geometry or the per-drive toggle. Shared UI support covers comma 3/3X and comma 4. |
| [VOACC fixes](voacc.md) | Start with model lead velocity, blend in a bounded early closing cue, then use mature range-derived velocity. Reset inconsistent cut-in history without hiding current range. Preserve urgent braking and stopped-lead constraints. E2E stop targets use a stationary virtual lead, a 1.5 m earlier target, and a persistent rectangular stop marker. |
| SLC | Match fresh GPS/heading to OSM road geometry. Use legal limits first and advisory speeds when no legal limit exists. Keep the existing cruise target when map data is missing or ambiguous. |
| [CEM](conditional-experimental.md) | Separate conditional mode enables E2E for junctions, missing map speed, or sustained model stops. No-speed fallback acknowledges its SET change and recovery. |
| [C3x UI](c3x-ui.md) | Camera zoom/crossfade in all modes, brake and blind-spot border feedback, and rounded real-lead triangles. |

E2E slowing assistance can qualify when the model-estimated lead speed is below
55 km/h and E2E requests sustained braking. The threshold controls entry only;
crossing it does not clear an existing latch. Confirmed go/pull-away conditions
release assistance, with a bounded return to acceleration. Lead loss and invalid
data retain their guards; driver override clears immediately. Stronger regular
braking always wins. This does not enable Experimental Mode or change SLC targets.
The path shows the selected deceleration colour while assistance contributes.

The SLC sign toggle defaults on where openpilot controls cruise speed. SET on
engagement uses the higher of the fresh map speed and current driving speed;
RES restores the previous setting. While engaged, short SET/− and RES/+ presses
adjust by one step and pause map tracking until a different limit qualifies.
The same limit finishing qualification or returning after a dropout cannot undo
a manual selection. Long presses retain 5 km/h or 5 mph steps. Automatic changes
require two seconds of stable, advancing map data. Advisory signs are yellow.

The green pulse follows an automatic speed adjustment until measured speed
settles within ±1 km/h for one second. It is hidden while a fresh plan selects
a real lead or E2E lead assistance as the limiting constraint; a lead merely
being present is insufficient. It can resume if cruise takes over while the
adjustment remains pending. Manual speed changes, pedal intervention,
disengagement or map-off cancel it. This feedback never changes control output.

OSM matching walks connected roads using CAN distance and calibrated measured
steering, with GPS positions correcting the match. When receiver course is poor,
a recent GPS displacement checked against vehicle motion can recover heading.
Up to six branches compete at a junction; sustained motion and score separation
commit the selected branch so an old ramp cannot win later just by being nearer.
An established path tolerates bounded GPS/lane offset, retains heading at stops,
and releases when motion contradicts it. Acquisition, freshness, direction and
connectivity checks still apply. This does not navigate indefinitely without GPS
beyond the bounded saved parking position/heading restart support. Speed values are never blended.
Recently missed exit branches can recover from sustained motion evidence.
An untagged ramp can look ahead along one directed continuation to its merge
limit using eight seconds of travel distance, clamped to 40–250 metres. Forks, missing
connectivity and intervening speed restrictions stop this lookup. Both lower and
higher targets can apply before the merge after normal qualification, allowing
speed matching on highway entry. `mapSpeedLimit.distanceAhead` records the
distance to the upcoming limit; manual speed adjustments keep their usual priority.
Other unknown limits remain unknown. The cache stays in memory; deprecated
message ordinals remain reserved for old logs.

Focused tests cover lead/E2E transitions, map matching, SLC and pulse behavior.
Offline replays and isolated planner/UI checks do not establish stopping distance
or replace a full device build and closed-loop validation.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).

The `boat-anchor-dynamic-zones` development branch adds location-based civil-time
evaluation for explicitly mapped school/playground schedules and dated temporary
limits. See [dynamic speed zones](dynamic-speed-zones.md) for supported conditions,
data coverage and the additional device dependency provisioning requirement.
