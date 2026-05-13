# Plan: Add ros-jazzy-sllidar-ros2 to robostack-jazzy

## Context

The auldbot project (ROS2 Jazzy differential drive robot) currently builds `sllidar_ros2` from source as a git submodule at `third_party/sllidar_ros2`. The goal is to get it into the robostack-jazzy conda channel so it can be installed via `pixi add ros-jazzy-sllidar-ros2`, removing the submodule entirely.

The project uses pixi for package management (`pixi.toml` at repo root). The robostack-jazzy channel is how ROS2 packages are distributed in this ecosystem.

## About sllidar_ros2

- **Upstream repo:** https://github.com/Slamtec/sllidar_ros2
- **Current submodule commit:** `3430009` (HEAD of main, no tags)
- **`package.xml` version:** `1.0.1`
- **Build system:** `ament_cmake`
- **Runtime deps:** `rclcpp`, `sensor_msgs`, `std_srvs` — all standard ROS2, all already in robostack-jazzy
- **No FetchContent, no vendored C++ deps** — this is a straightforward recipe
- **License:** BSD-3-Clause (check LICENSE file in repo)

## Blocker: No Release Tag

The upstream repo has no git tags. The conda recipe needs a stable tarball URL. Before the recipe can be written:

1. Open an issue on https://github.com/Slamtec/sllidar_ros2 requesting a `v1.0.1` tag on the current HEAD
2. Alternatively, open a PR that adds a tag (maintainers often just need a nudge)
3. Wait for the tag to be created

Once the tag exists at `https://github.com/Slamtec/sllidar_ros2/archive/refs/tags/v1.0.1.tar.gz`, compute the sha256:
```bash
curl -sL https://github.com/Slamtec/sllidar_ros2/archive/refs/tags/v1.0.1.tar.gz | sha256sum
```

## Recipe

Fork https://github.com/RoboStack/ros-jazzy and create:
`recipes/ros-jazzy-sllidar-ros2/meta.yaml`

```yaml
{% set name = "ros-jazzy-sllidar-ros2" %}
{% set version = "1.0.1" %}

package:
  name: {{ name }}
  version: {{ version }}

source:
  url: https://github.com/Slamtec/sllidar_ros2/archive/refs/tags/v{{ version }}.tar.gz
  sha256: <fill in after tag is cut>

build:
  number: 0

requirements:
  build:
    - {{ compiler('cxx') }}
    - cmake
    - ninja
    - pkg-config
  host:
    - ros-jazzy-ament-cmake
    - ros-jazzy-rclcpp
    - ros-jazzy-sensor-msgs
    - ros-jazzy-std-srvs
  run:
    - ros-jazzy-rclcpp
    - ros-jazzy-sensor-msgs
    - ros-jazzy-std-srvs

test:
  commands:
    - test -f ${PREFIX}/lib/sllidar_ros2/sllidar_ros2_node

about:
  home: https://github.com/Slamtec/sllidar_ros2
  license: BSD-3-Clause
  license_file: LICENSE
  summary: ROS2 driver for Slamtec RPLIDAR series (A1, A2, A3, C1, S1, S2, S3)

extra:
  recipe-maintainers:
    - mjohnson459
```

## PR to RoboStack

Open PR to `RoboStack/ros-jazzy`:
- Title: `Add ros-jazzy-sllidar-ros2 1.0.1`
- CI must pass for both `linux-64` and `linux-aarch64` (the Pi 5 target is aarch64)

## Final Step: Transition auldbot

Once `pixi search ros-jazzy-sllidar-ros2` returns a result:

1. Add to `pixi.toml` `[dependencies]`:
   ```toml
   ros-jazzy-sllidar-ros2 = ">=1.0.1"
   ```

2. Remove the submodule:
   ```bash
   git submodule deinit -f third_party/sllidar_ros2
   git rm -f third_party/sllidar_ros2
   rm -rf .git/modules/third_party/sllidar_ros2
   ```

3. If no other submodules remain, also remove the `submodules` task from `pixi.toml` and the `depends-on = ["submodules"]` from the `build` task.

## Critical Files (in auldbot repo)

- `third_party/sllidar_ros2/package.xml` — version and dep reference
- `third_party/sllidar_ros2/LICENSE` — verify license text
- `pixi.toml` — add dep, remove submodule task when done
- `.gitmodules` — `git rm` handles removing the entry automatically
