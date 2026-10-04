using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct MapSpeedLimit @0x81c2f05a394cf4af {
  speedLimit @0 :Float32;  # legal limit in m/s, valid only with Event.valid
  gpsMonoTime @1 :UInt64; # timestamp of the matched GPS fix, nanoseconds
  headingValid @2 :Bool;
  # Display can briefly outlive a confirmed match. NEVER use for control.
  displaySpeedLimit @3 :Float32;
  displayValid @4 :Bool;
  displayGpsMonoTime @5 :UInt64;
  displayIsAdvisory @6 :Bool; # recommended speed: yellow sign, not a legal maximum
}

struct MapCruiseState @0xaedffd8f31e7b55d {
  state @0 :State;
  targetSpeed @1 :Float32; # qualified map target in m/s, zero if unavailable
  pendingSpeed @2 :Float32; # UI only: candidate automatic change in m/s, zero if none
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
