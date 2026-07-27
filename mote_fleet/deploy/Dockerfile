# The Mote fleet server: the fleet API, the registry, and the operator UI.
#
# The fleet box is ordinary server infrastructure — a VPS, a home server, a
# spare Pi — so unlike the robot it is not fleet-managed and does not update
# through the prefix.dev channel (fleet.md, "The other pipelines"). It is a
# container image and a compose file, versioned in this repo.
#
# python:3.12-slim with one dependency: paho, which the server needs only on the
# mediated-dispatch path (BrokerLink). Everything else is stdlib, which is why
# the fleet box needs neither ROS nor a checkout, and why this image is tens of
# megabytes and builds in seconds — on the box itself if GHCR is unreachable.
#
# Build (normally CI, see .github/workflows/fleet-image.yml):
#   docker build -f mote_fleet/deploy/Dockerfile \
#       --build-arg MOTE_VERSION=$(git describe --always --dirty --tags) .
FROM python:3.12-slim

# Reported by /healthz, so an update can gate on the new version being the one
# that answered rather than on something answering. Same convention as the
# inference image's MOTE_VERSION.
ARG MOTE_VERSION=unknown
ENV MOTE_VERSION=${MOTE_VERSION} \
    PYTHONUNBUFFERED=1

# The fleet's memory of who is in it — registry rows and tokens — and, until M4
# moves map distribution to the registry, the site bundles the UI's basemaps are
# served from ($MOTE_FLEET_HOME/sites). A named volume is mounted here by the
# compose file, so redeploying the *code* replaces the container around this
# directory and never the state inside it. Same split as the robot's MOTE_HOME.
ENV MOTE_FLEET_HOME=/var/lib/mote-fleet

RUN pip install --no-cache-dir "paho-mqtt>=2,<3"

# The layout the server already expects: fleet_server.py puts its parent's
# parent on sys.path and imports `mote_fleet.protocol` from there, and finds the
# UI beside itself — so /app/mote_fleet/protocol.py + /app/server/*.py needs no
# code change.
WORKDIR /app
COPY mote_fleet/mote_fleet/__init__.py \
     mote_fleet/mote_fleet/protocol.py \
     /app/mote_fleet/
COPY mote_fleet/server/fleet_server.py \
     mote_fleet/server/registry.py \
     mote_fleet/server/fleetctl.py \
     /app/server/
# The dashboard is static files served by the API, so it ships in the same
# image and cannot be a version behind the routes it calls.
COPY mote_fleet/server/ui/ /app/server/ui/

# Not root: this process is reachable by every robot on the tailnet, and it has
# no reason to own anything but its state directory.
RUN useradd --system --uid 10001 --home-dir /var/lib/mote-fleet mote \
    && mkdir -p /var/lib/mote-fleet \
    && chown -R mote:mote /var/lib/mote-fleet
USER mote

EXPOSE 8080

# The same route an operator would curl, asked from inside the container so it
# tests the process rather than the host's port mapping. `docker compose up
# --wait` and the deploy script's health gate both read this.
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
    CMD python -c "import json,urllib.request; \
assert json.load(urllib.request.urlopen('http://127.0.0.1:8080/healthz', timeout=4))['ok']"

# Args (--broker-host, --broker-ws-port, ...) come from the compose file, which
# is where the deployment's knobs belong.
ENTRYPOINT ["python", "-u", "/app/server/fleet_server.py", "--host", "0.0.0.0", "--port", "8080"]
