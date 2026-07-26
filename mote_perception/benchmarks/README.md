# Inference benchmarks

Latency/throughput for the off-board inference servers, so the dedicated
gaming-PC (NVIDIA/CUDA) tier can be compared against the earlier CPU and
ROCm-iGPU tiers. Numbers here are the **full socket round trip** measured with
`pixi run inference-bench` unless noted (that is what the robot actually pays —
compress → send → infer → receive), plus the server's own per-frame model time
where it isolates GPU-only cost.

Regenerate the CUDA numbers with, from the robot or a LAN machine:

```bash
pixi run inference-bench --host mote-gpu --image sample.jpg --frames 200 \
    --out mote_perception/benchmarks/depth_cuda_lan.json
pixi run inference-bench --service detect --host mote-gpu --labels "red box" \
    --frames 50 --out mote_perception/benchmarks/detect_cuda_lan.json
```

Commit the resulting `*.json` alongside this file and fill the tables below.

## Depth (Depth-Anything-V2-Small, 640×480)

| Tier | Machine | Device | ~ms/frame | ~fps | Source |
|---|---|---|---:|---:|---|
| CPU (idle) | Linux dev | CPU (physical-core threads) | ~330 | ~3 | #152, `depth_server.py` comment (measured with `depth_bag_eval.py`) |
| CPU (oversubscribed) | Linux dev | CPU (SMT threads) | ~460 | ~2 | #152, same |
| CPU (under load) | Linux dev / Pi-class | CPU while RViz+ROS+node run | ~1000–2000 | ~0.5–1 | #152 prose baseline |
| ROCm iGPU | Linux dev | Radeon 780M (gfx1103, fp32) | ~330, **flat under load** | ~3 | #152 — ties idle CPU, stays flat where CPU degrades |
| **CUDA (LAN)** | **container on the inference machine** | **NVIDIA (fp32)** | _measure_ | _measure_ | **`inference-bench` → `depth_cuda_lan.json`** |
| CUDA (fp16) | container | NVIDIA (`--fp16`) | _optional_ | _optional_ | optional, if fp16 is stable on the card |

Measure with the server warm: the first frame after an idle release includes the
model load (see `--idle-timeout` in `docs/inference-server.md`), which `--warmup`
already excludes by default.

The CUDA tier is expected to land well under the ROCm/CPU ~330 ms — a discrete
NVIDIA card runs this small ViT in tens of ms — so the depth rate is bounded by
the camera/publish rate, not the model. The point of the number is to *confirm*
that and to quantify the LAN overhead (round trip minus the server's reported
`served … in N ms`).

## Detect (OWLv2-base, 640×480)

OWLv2 is much heavier than depth and was CPU-only before this task; the
container's CUDA torch plus the `--device` support in `detect_server.py` let it
run on the GPU too. Measure once the inference machine is up:

| Tier | Machine | Device | ~ms/frame | ~fps | Source |
|---|---|---|---:|---:|---|
| CPU | Linux dev | CPU (all cores) | _prior_ | _prior_ | pre-CUDA baseline, if recorded |
| **CUDA (LAN)** | **container** | **NVIDIA** | _measure_ | _measure_ | **`inference-bench --service detect` → `detect_cuda_lan.json`** |

Detection is a per-command burst (the task layer asks for a label, not a stream),
so its latency matters for fetch responsiveness, not sustained fps.

## End-to-end (robot → server → `/camera_obstacles`)

The socket round trip above is the dominant term, but the full pipeline also
includes JPEG capture on the Pi, lidar rescale, back-projection, and publish. To
capture the whole leg over the real LAN, with the robot running `pixi run
perception` pointed at the gaming PC:

- read the depth node's cloud publish rate on `/camera_obstacles`
  (`ros2 topic hz /camera_obstacles`), and
- compare against `inference-bench`'s server round trip to attribute the rest to
  on-robot processing.

Record the observed `/camera_obstacles` Hz and the `inference-bench` summary here
once measured on hardware.
