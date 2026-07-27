# Mote inference server: depth + open-vocabulary detection on an NVIDIA GPU.
#
# One image, three homes: a Windows gaming PC (Docker Desktop), a Linux box, or a
# cloud GPU instance. The host needs only an NVIDIA driver and a container runtime
# with GPU support — nothing is installed on it, which is the point: the inference
# machine is not a development environment and should never need the repo, a
# toolchain, or a checkout.
#
# Base is python:3.12-slim, NOT nvidia/cuda: the pytorch.org cu128 wheels bundle
# the CUDA runtime libraries they need, so only the host *driver* has to match.
# That saves several GB over a full CUDA base image, which matters both for CI
# disk limits and for how long a pull takes on a home connection.
#
# Model weights are baked in at build time (see the prefetch step), so a fresh
# container serves its first frame without reaching out to the HuggingFace hub —
# no first-run stall, and it works on an air-gapped or flaky network.
#
# Build (normally done by CI, see .github/workflows/inference-image.yml):
#   docker build -f mote_perception/deploy/Dockerfile -t mote-inference \
#       --build-arg MOTE_VERSION=$(git describe --always --dirty --tags) .
FROM python:3.12-slim

# Baked into the health blob so `pixi run inference-health` on the robot can
# report which build the server is running and warn on version skew. There is no
# git in the image, so this is the only source of that answer.
ARG MOTE_VERSION=unknown
ENV MOTE_VERSION=${MOTE_VERSION}

# Keep model weights inside the image rather than a volume, so the container is
# self-contained and a `docker run` with no volumes is a complete deployment.
ENV HF_HOME=/models \
    PYTHONUNBUFFERED=1

WORKDIR /app

# torch from the CUDA wheel index; everything else from PyPI. --no-cache-dir
# keeps the layer from carrying a second copy of the ~2.5 GB wheel.
RUN pip install --no-cache-dir \
        --extra-index-url https://download.pytorch.org/whl/cu128 \
        "torch>=2.8,<3" \
        "transformers>=4,<5" \
        "numpy>=1.26,<3" \
        "pillow>=10,<13" \
        "scipy>=1,<2"

# Only what the servers actually need: the wire protocol package and the tools.
# The ROS packages, sim, and robot config have no business in this image.
COPY mote_perception/mote_perception/__init__.py \
     mote_perception/mote_perception/depth_wire.py \
     mote_perception/mote_perception/detect_wire.py \
     /app/mote_perception/
COPY mote_perception/tools/depth_server.py \
     mote_perception/tools/detect_server.py \
     mote_perception/tools/inference_server.py \
     mote_perception/tools/model_host.py \
     mote_perception/tools/prefetch_models.py \
     mote_perception/tools/probe.py \
     /app/tools/

# Bake the weights. Runs on the build machine's CPU — no GPU needed to build.
RUN python /app/tools/prefetch_models.py

# Depth, detect. Published to the host with -p; the supervisor binds 0.0.0.0
# inside the container, which is a private namespace, not the LAN.
EXPOSE 5601 5602

# Liveness only — the sentinel is answered before the model has ever loaded, so
# it says "the process is serving", not "this build works". The update pipeline
# (deploy/inference-deploy.sh) runs the same probe *with* a real frame against a
# candidate on a shadow port, which is what a health gate has to mean here.
HEALTHCHECK --interval=60s --timeout=20s --start-period=30s --retries=3 \
    CMD python /app/tools/probe.py --no-infer --timeout 10 || exit 1

# The supervisor runs every service in SERVICES and exits if any dies, so the
# container's restart policy (not a bespoke supervisor loop) is what brings it
# back — one less moving part than the host-side runner this replaces.
ENTRYPOINT ["python", "-u", "/app/tools/inference_server.py"]
