# Dynamic speed zones

This initial implementation evaluates explicitly mapped OSM conditional speed
limits at the vehicle's current location. There is no hard-coded home city,
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
condition are mapped. Untimed temporary tags remain unsupported. City disruption
notices alone cannot supply a posted speed, travel direction or sign boundaries.
Verified municipal sources can be added later as providers of explicit road
restrictions; no roadwork-to-speed guesses or language-model decisions run onroad.

Edmonton's official daily playground hours are an example, not a global default:
https://www.edmontonpolice.ca/TrafficVehicles/TrafficSafety/SchoolZones

Map semantics:
https://wiki.openstreetmap.org/wiki/Key:maxspeed:conditional

## Packaging and validation

The Python project and lockfile include `timezonefinder==8.2.0` and `tzdata`.
Timezone polygons are approximately 53 MB compressed. Existing AGNOS device
environments do not automatically install new Python packages when switching
branches. They require dependency provisioning before timed zones work; absent
dependencies leave conditional limits unknown without stopping ordinary mapd.
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
