# Dynamic speed zones

This implementation evaluates explicitly mapped OSM conditional speed limits
and verified municipal restrictions at the vehicle's current location. There is no hard-coded home city,
school timetable, timezone or UTC offset. It uses the existing road matcher,
speed-limit display and SLC qualification/driver-override behavior.

## Time and location

- Fresh GPS supplies UTC time. Elapsed time is carried forward with the monotonic
  clock for up to ten minutes; a parked position alone cannot establish time.
- `timezonefinder` resolves current coordinates against worldwide timezone
  polygons offline. Python `zoneinfo` converts UTC to local civil time, including
  daylight-saving changes and fractional-hour offsets.
- The timezone is looked up at the current position rather than cached by city
  name or a broad radius. Samples around the position's accuracy allowance must
  agree on the current offset. This is a bounded border check, not a guarantee
  that GPS or administrative boundaries are exact.
- Missing clock/timezone data or a contradictory GPS clock leaves conditional
  limits unknown. Ordinary unconditional limits remain usable.
- Re-evaluation happens on local minute boundaries, including while stopped.
  A clock change is not treated as new motion evidence for road matching.

## Supported map rules

| OSM conditional value | Interpretation |
| --- | --- |
| `30 @ (07:30-21:00)` | Every day, local time, including weekends |
| `30 @ (Mo-Fr 08:00-09:30,14:30-16:00)` | Two weekday windows |
| `40 @ (Fr 22:00-06:00)` | Friday evening through Saturday morning |
| `50 @ (2026 Oct 10-2026 Oct 12)` | Dated temporary limit, through the last date |
| `50 @ (2026 Oct 10-2026 Oct 12 08:00-18:00)` | Temporary daily work hours |

The normal `maxspeed` applies outside a fully understood condition. Numeric km/h
and mph, directional conditional tags and advisory conditions are supported.
Contradictory overlapping conditions, malformed syntax and unsupported qualifiers
stay unknown. An unresolved legal condition cannot fall through to advisory cruise.

This is a strict subset of OSM opening-hours syntax. `school_days`, public/school
holiday calendars (`PH`/`SH`), flashing lights, children present, weather and
sunrise/sunset are not inferred. Dates need an explicit year and English
three-letter month. Conditions must exist on the matched road section; proximity
to a school or construction point does not establish a speed limit.

## Coverage and temporary construction

Location/time evaluation works across cities. Data completeness is separate:
this does not discover every city's municipal school calendar or live roadwork
feed. Construction is supported when an explicit numeric limit and date/time
condition are mapped. Legacy `temporary:*` speed tags remain unsupported. City disruption
notices alone cannot supply a posted speed, travel direction or sign boundaries.
The City of Edmonton Speed Zones feed now supplements OSM. It supplies each
playground segment's geometry, numeric speed, travel direction and published
hours. The provider is selected geographically; its rules are applied only to
matching road names, geometry and alignment inside the segment endpoints.
The provider's documented IANA timezone supplies civil time for those segments,
including DST, without requiring timezonefinder. Other cities continue to use
OSM; additional municipal feeds require verified adapters. No city timetable is
used as a worldwide default.

Downloads share the background map worker with independent refresh/backoff.
Each response is capped at 2 MB and fewer than 1,000 records; cache lifetime is
ten minutes, with movement-based prefetch. No files or route history accumulate.
A known matched zone with missing time, unsupported hours, conflicting limits or
expired cached data remains unresolved. A municipal limit cannot relax a lower
OSM limit or resolve an unknown OSM legal condition.

The school bookmark on route 2ea/11 re-evaluates from 40 to 30 km/h with the
retrieved public road and municipal geometry. This is an offline map replay,
not a closed-loop vehicle test. The construction bookmark's driver-confirmed
40 km/h is still absent from the queried sources: no geographic override is
invented from a point or expanded over the whole construction project.

Active restrictions display CONSTRUCTION on orange or SCHOOL on neon green
inside the existing 34-unit MAXIMUM row. The numeric speed layout is unchanged.
`mapSpeedLimit.zoneType` records the active label and `zoneSource` identifies a
municipal record consulted for the matched road; old logs default to no zone.

Source: https://data.edmonton.ca/Transportation-Infrastructure/Speed-Zones/6vjs-shqe
No roadwork-to-speed guesses or language-model decisions run onroad.

Edmonton's official daily playground hours are an example, not a global default:
https://www.edmontonpolice.ca/TrafficVehicles/TrafficSafety/SchoolZones

Map semantics:
https://wiki.openstreetmap.org/wiki/Key:maxspeed:conditional

## Packaging and validation

The Python project and lockfile include `timezonefinder==8.2.0` and `tzdata`.
Timezone polygons are approximately 53 MB compressed. Existing AGNOS device
environments do not automatically install new Python packages when switching
branches. They require dependency provisioning for worldwide OSM timezone resolution;
absent dependencies leave those conditional limits unknown without stopping
ordinary mapd. Municipal records with a verified IANA timezone use system
zoneinfo and do not require the polygon package.
No automatic package install or additional network time service runs onroad.

PC tests: `python -m pytest openpilot/selfdrive/mapd/tests` in the repository's
provisioned environment. Tests exercise city/timezone changes, DST, weekday and
date boundaries, reverse-direction restrictions, invalid clocks, missing
dependencies, road ambiguity and existing map behavior. Device installation and
live construction/school coverage still require validation. SLC retains its
existing qualification delay; this change does not add advance braking before
zone entry or establish closed-loop driving safety.

Timezone data and code licensing/attribution are distributed with timezonefinder:
https://github.com/jannikmi/timezonefinder

The endpoint-stability POC was removed from the control path's diagnostic work:
no sample deque, travel integration, spread calculation or stability telemetry.
The current endpoint detector, freshness checks, approach behavior and confirmed
stop tracking remain. Comparing the old and new detector/target pipeline on
3,600 recorded model frames from 2ea/9–11 produced identical values in all eleven
control-state fields (1,325 active-target frames and 50 approach frames).

Validation for the municipal integration: 165 focused map/stop tests and 72
subtests passed, the live municipal request returned the expected segments, and
zone metadata serialized through the complete log schema. A full device build
and on-device performance validation were not run.
