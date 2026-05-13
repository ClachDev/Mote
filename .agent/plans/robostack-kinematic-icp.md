# Plan: Add ros-jazzy-kinematic-icp to robostack-jazzy

## Context

The auldbot project (ROS2 Jazzy differential drive robot) currently builds `kinematic_icp` from source as a git submodule at `third_party/kinematic_icp`. It is used for lidar-wheel odometry fusion: it reads the raw `odom → base_footprint` TF from `diff_drive_controller` and the `/scan_filtered` topic, and publishes a corrected `odom_lidar → base_footprint` TF plus `/kinematic_icp/lidar_odometry`.

The goal is to get it into the robostack-jazzy conda channel so it can be installed via `pixi add ros-jazzy-kinematic-icp`, removing the submodule.

The project uses pixi for package management (`pixi.toml` at repo root).

## About kinematic_icp

- **Upstream repo:** https://github.com/PRBonn/kinematic-icp (PRBonn lab, ICRA 2025)
- **Release tag to target:** `v0.1.1`
- **`package.xml` location:** `ros/package.xml` (the repo has `cpp/` and `ros/` subdirs; we build the `ros/` package)
- **Build entry point:** `ros/CMakeLists.txt` — pass `$SRC_DIR/ros` as the cmake source dir
- **License:** MIT

## Dependency Challenge: FetchContent

`kinematic_icp`'s CMake fetches `kiss-icp v1.2.0` at configure time via FetchContent:

```cmake
# cpp/kinematic_icp/kiss_icp/kiss-icp.cmake
FetchContent_Declare(kiss_icp
  URL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v1.2.0.tar.gz
  SOURCE_SUBDIR cpp/kiss_icp)
FetchContent_MakeAvailable(kiss_icp)
```

`kiss-icp` itself fetches `Sophus`, `TBB`, `Eigen`, and `tsl_robin_map`, but it already has `USE_SYSTEM_<DEP>` flags in `cpp/kiss_icp/3rdparty/find_dependencies.cmake` to skip those fetches and use system packages instead.

**Solution:** Declare `kiss-icp` as a second source entry in the recipe. Use `FETCHCONTENT_SOURCE_DIR_KISS_ICP` to redirect FetchContent to the pre-extracted directory, eliminating the network dependency during the build. Pass `USE_SYSTEM_*=ON` for all transitive deps so they come from conda-forge.

All transitive deps exist in conda-forge:
| Dep | conda-forge package |
|-----|-------------------|
| Sophus | `sophus` |
| TBB | `tbb-devel` |
| Eigen3 | `eigen` |
| tsl_robin_map | `tsl_robin_map` |

## Compute sha256 Values

```bash
# kinematic_icp v0.1.1
curl -sL https://github.com/PRBonn/kinematic-icp/archive/refs/tags/v0.1.1.tar.gz | sha256sum

# kiss-icp v1.2.0
curl -sL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v1.2.0.tar.gz | sha256sum
```

## Recipe

Fork https://github.com/RoboStack/ros-jazzy and create:
`recipes/ros-jazzy-kinematic-icp/meta.yaml`

```yaml
{% set name = "ros-jazzy-kinematic-icp" %}
{% set version = "0.1.1" %}
{% set kiss_icp_version = "1.2.0" %}

package:
  name: {{ name }}
  version: {{ version }}

source:
  - url: https://github.com/PRBonn/kinematic-icp/archive/refs/tags/v{{ version }}.tar.gz
    sha256: <fill in>
    folder: src
  - url: https://github.com/PRBonn/kiss-icp/archive/refs/tags/v{{ kiss_icp_version }}.tar.gz
    sha256: <fill in>
    folder: kiss-icp-src

build:
  number: 0
  script: |
    cmake -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DFETCHCONTENT_SOURCE_DIR_KISS_ICP=$SRC_DIR/kiss-icp-src \
      -DUSE_SYSTEM_SOPHUS=ON \
      -DUSE_SYSTEM_TBB=ON \
      -DUSE_SYSTEM_EIGEN3=ON \
      -DUSE_SYSTEM_TSL_ROBIN_MAP=ON \
      $SRC_DIR/src/ros
    cmake --build . --parallel $CPU_COUNT
    cmake --install .

requirements:
  build:
    - {{ compiler('cxx') }}
    - cmake
    - ninja
    - pkg-config
  host:
    - ros-jazzy-ament-cmake
    - ros-jazzy-rclcpp
    - ros-jazzy-rclcpp-components
    - ros-jazzy-rcutils
    - ros-jazzy-geometry-msgs
    - ros-jazzy-nav-msgs
    - ros-jazzy-sensor-msgs
    - ros-jazzy-laser-geometry
    - ros-jazzy-std-msgs
    - ros-jazzy-std-srvs
    - ros-jazzy-tf2-ros
    - ros-jazzy-visualization-msgs
    - ros-jazzy-rosbag2-cpp
    - ros-jazzy-rosbag2-storage
    - ros-jazzy-ros2launch
    - eigen
    - sophus
    - tbb-devel
    - tsl_robin_map
  run:
    - ros-jazzy-rclcpp
    - ros-jazzy-rclcpp-components
    - ros-jazzy-geometry-msgs
    - ros-jazzy-nav-msgs
    - ros-jazzy-sensor-msgs
    - ros-jazzy-laser-geometry
    - ros-jazzy-std-msgs
    - ros-jazzy-std-srvs
    - ros-jazzy-tf2-ros
    - ros-jazzy-visualization-msgs
    - ros-jazzy-rosbag2-cpp
    - ros-jazzy-rosbag2-storage
    - sophus
    - tbb-devel
    - tsl_robin_map

test:
  commands:
    - test -f ${PREFIX}/lib/kinematic_icp/kinematic_icp_online_node

about:
  home: https://github.com/PRBonn/kinematic-icp
  license: MIT
  license_file: src/LICENSE
  summary: Tightly-coupled kinematic-constrained ICP for lidar-wheel odometry fusion (ICRA 2025)

extra:
  recipe-maintainers:
    - mjohnson459
```

**Key cmake flag notes:**
- `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` — required because vendored Sophus/tessil have old `cmake_minimum_required` values that CMake 4 rejects without this flag (confirmed needed from local build)
- `-DFETCHCONTENT_SOURCE_DIR_KISS_ICP` — the variable name is derived from the `FetchContent_Declare` name `kiss_icp` uppercased. Redirects to the pre-extracted tarball; no internet needed at build time
- `USE_SYSTEM_*` flags are defined in `kiss-icp`'s `3rdparty/find_dependencies.cmake` and bypass the sub-FetchContent calls

## PR to RoboStack

Open PR to `RoboStack/ros-jazzy`:
- Title: `Add ros-jazzy-kinematic-icp 0.1.1`
- CI must pass for both `linux-64` and `linux-aarch64` (Pi 5 is aarch64)

## Final Step: Transition auldbot

Once `pixi search ros-jazzy-kinematic-icp` returns a result:

1. Add to `pixi.toml` `[dependencies]`:
   ```toml
   ros-jazzy-kinematic-icp = ">=0.1.1"
   ```

2. Remove from `pixi.toml` `[dependencies]` (no longer needed at build time):
   ```toml
   ros-jazzy-rosbag2-cpp = ">=0.26.0"
   ros-jazzy-rosbag2-storage = ">=0.26.0"
   ```
   (these become transitive deps of kinematic-icp)

3. Remove the submodule:
   ```bash
   git submodule deinit -f third_party/kinematic_icp
   git rm -f third_party/kinematic_icp
   rm -rf .git/modules/third_party/kinematic_icp
   ```

4. Remove `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` from the `build` task in `pixi.toml` if no other submodule needs it.

5. If no other submodules remain, remove the `submodules` task and `depends-on = ["submodules"]` from `pixi.toml`.

## Critical Files (in auldbot repo)

- `third_party/kinematic_icp/ros/package.xml` — dependency list
- `third_party/kinematic_icp/cpp/kinematic_icp/kiss_icp/kiss-icp.cmake` — declares the `kiss_icp` FetchContent (fetch name determines the `FETCHCONTENT_SOURCE_DIR_*` variable name)
- `third_party/kinematic_icp/` ← in build dir: `build/kinematic_icp/_deps/kiss_icp-src/cpp/kiss_icp/3rdparty/find_dependencies.cmake` — `USE_SYSTEM_*` flag definitions
- `pixi.toml` — add dep, clean up build flags when done
- `.gitmodules` — `git rm` handles removing the entry automatically
