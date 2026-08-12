# Where the C++ optimisation flags come from, 2026-08-11

**Verdict: `pixi run build` passes `-DCMAKE_BUILD_TYPE=Release`, and it changes
nothing measurable.** An empty `CMAKE_BUILD_TYPE` in `build/*/CMakeCache.txt`
looks like an unoptimised robot and is not one — which is the thing worth
recording here.

Empty means CMake appends no `CMAKE_CXX_FLAGS_<CONFIG>`; it does not mean `-O0`,
because `CMAKE_CXX_FLAGS` is seeded from the environment's `CXXFLAGS` and the
conda compiler packages export one — `-O2` on linux-64, **`-O3` on
linux-aarch64**. mote-01's real compile lines carried `-O3` before this change.
`kinematic_icp` was never in question either: its own CMakeLists does
`set(CMAKE_BUILD_TYPE Release)` (`ros/CMakeLists.txt:26`), shadowing the cache,
so its objects already had `-O3 -DNDEBUG`.

So the flag adds `-DNDEBUG` on the robot and takes linux-64 from `-O2` to `-O3`.
Its point is that the level is now stated by the repo rather than inherited from
whichever toolchain the solve picked, and survives a build outside pixi.
**Not `RelWithDebInfo`**: config flags are appended *after* `CXXFLAGS`, so its
`-O2` would beat the Pi's `-O3` and lower optimisation.

## Measured

Replaying mapping bag `20260729_160009` into `localization_launch.py`
(`use_sim_time:=true`) drives the real ICP at the recorded 10.0 Hz, identical
input each run, so only the binary differs. Container `utime + stime` over a
120 s window, 3 reps a side, rebuild between:

| | mean CPU-s / 120 s | mean ms per scan |
| --- | --- | --- |
| before | 2.207 | 1.837 |
| after | 2.200 | 1.831 |

−0.3% against a ±0.7% within-group spread: no effect, as expected from flags
that barely moved. Also worth knowing: the whole container costs **1.8% of one
core**, so it is not where a nav mission's load goes.

## `-ffp-contract=off` survives Release

`odom_tf_relay`'s bit-identity against the Python it replaced depends on that
flag (`mote_nav/CMakeLists.txt:60`), and Release now stacks `-O3` over it.
Target options are emitted last, so it still wins — counted in the shipped
aarch64 objects, not inferred: `libodom_tf_relay_component.so` holds **0** fused
multiply-adds before and after, while `libicp_odom_gate_component.so` (same
`-O3`, no flag) holds **11** both times, so the zero is the flag and not a
target without FMA. `test_odom_tf_relay.cpp` cannot catch this — it compares the
relay against reference arithmetic in the same TU, under the same flag.
