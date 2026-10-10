using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct MapSpeedLimit @0x81c2f05a394cf4af {
  speedLimit @0 :Float32;  # current/merge-approach legal limit in m/s; zero if unavailable, requires Event.valid
  gpsMonoTime @1 :UInt64; # timestamp of the matched GPS fix, nanoseconds
  headingValid @2 :Bool;
  # Retain old ordinals for recorded-log compatibility; no runtime publisher.
  displaySpeedLimitDEPRECATED @3 :Float32;
  displayValidDEPRECATED @4 :Bool;
  displayGpsMonoTimeDEPRECATED @5 :UInt64;
  displayIsAdvisoryDEPRECATED @6 :Bool;
  advisorySpeed @7 :Float32; # fresh matched recommendation in m/s; fallback when speedLimit is zero
  distanceAhead @8 :Float32; # metres to a unique ramp-merge limit; zero on the current road
  positionEstimated @9 :Bool; # bounded wheel/steering projection, including a recent parked restart
  positionMonoTime @10 :UInt64; # observation time of the projected position; never rewrite gpsMonoTime
}

struct MapCruiseState @0xaedffd8f31e7b55d {
  state @0 :State;
  targetSpeed @1 :Float32; # qualified map target in m/s, zero if unavailable
  pendingSpeedDEPRECATED @2 :Float32;
  adjustingSpeed @3 :Float32; # UI pulse target until ego speed settles; zero if cancelled/complete
  enum State {
    off @0;
    armed @1;
    active @2;
    paused @3;
    waiting @4;
    unsupported @5;
  }
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}

struct ConditionalExperimentalState {
  state @0 :State;
  reason @1 :Text;
  targetId @2 :UInt64;
  targetDistance @3 :Float32;
  activationDistance @4 :Float32;
  modelStopDistance @5 :Float32; # -1 if no sustained stop in the model horizon
  regularAcceleration @6 :Float32; # boat-anchor output before junction assistance
  modelAcceleration @7 :Float32;
  modelShouldStop @8 :Bool;
  modelSlowing @9 :Bool;
  mapValid @10 :Bool;
  armed @11 :Bool;
  contributing @12 :Bool; # actual additional junction slowing/stop constraint
  activationSpeed @13 :Float32; # current valid cruise set speed used for range, m/s; zero when unavailable
  e2eEnabled @14 :Bool; # qualified conditional model candidate, independent of contribution
  enum State {
    off @0;
    ready @1;
    inRange @2;
    assisting @3; # legacy slowing-only state
    active @4;
  }
}


struct StopTargetState {
  active @0 :Bool;
  distance @1 :Float32; # remaining ego travel to the tracked target, -1 when inactive
  holding @2 :Bool;
  horizonCandidate @3 :Bool;
  horizonStable @4 :Bool;
  horizonDistance @5 :Float32; # unshifted candidate stop travel, -1 when absent
  horizonRemaining @6 :Float32; # seconds of compact trajectory after endpoint onset
  horizonStableTime @7 :Float32; # observed span within the endpoint spread tolerance
  horizonSpread @8 :Float32; # motion-compensated endpoint range over the confirmation window
}


struct ModelStopState {
  active @0 :Bool;
  margin @1 :Float32;
  modelDistance @2 :Float32;
  targetDistance @3 :Float32;
  holdingStop @4 :Bool;
  predictionGrace @5 :Bool;
}
