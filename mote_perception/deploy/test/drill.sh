#!/usr/bin/env bash
# Exercise the inference deploy pipeline end to end, without a GPU.
#
# Builds three stub images — two that serve and one that answers health but
# rejects every frame — and runs the real `inference-deploy.sh` against them:
# first deploy, a good update, a bad update that must be caught on the shadow
# port, and a rollback. Nothing here is mocked except the model (deploy/test/
# stub_server.py speaks the real wire protocol and the image carries the real
# probe), so what passes is the pipeline that runs on the GPU box.
#
#   ./drill.sh            # ~1 minute, needs docker and nothing else
#
# Ports and container name are deliberately not the production ones, so this is
# safe to run on a machine that is also serving inference.
set -euo pipefail

cd "$(dirname "$0")"
REPO="$(cd ../../.. && pwd)"
DEPLOY="$REPO/mote_perception/deploy/inference-deploy.sh"

export IMAGE=mote-inference-stub
export NAME=mote-inference-drill
export GPUS=none
export DEPTH_PORT=15601 DETECT_PORT=15602
export SHADOW_DEPTH_PORT=15611 SHADOW_DETECT_PORT=15612
export STATE_DIR="${TMPDIR:-/tmp}/mote-inference-drill"
export NO_PULL=1 PROBE_WAIT=60

pass() { printf '\n  PASS  %s\n' "$*"; }

# Captured, not piped into grep: `grep -q` exits on the first match, and under
# `set -o pipefail` the SIGPIPE that gives the deploy script fails the whole
# check for the wrong reason.
serving() { "$DEPLOY" status; }
fail() {
    printf '\n  FAIL  %s\n' "$*" >&2
    exit 1
}

cleanup() {
    docker rm -f "$NAME" "$NAME-green" >/dev/null 2>&1 || true
    rm -rf "$STATE_DIR"
}
trap cleanup EXIT
cleanup

echo "== building stub images"
for spec in "v1:serve" "v2:serve" "bad:reject"; do
    tag="${spec%%:*}"
    mode="${spec##*:}"
    docker build -q -f "$REPO/mote_perception/deploy/test/Dockerfile" \
        --build-arg "MOTE_VERSION=stub-$tag" --build-arg "STUB_MODE=$mode" \
        -t "$IMAGE:$tag" "$REPO" >/dev/null
done

echo "== 1. first deploy"
TAG=v1 "$DEPLOY" up
case "$(serving)" in *stub-v1*) ;; *) fail "v1 is not what is deployed" ;; esac
pass "deployed and probed"

echo "== 2. update to a good build"
"$DEPLOY" update "$IMAGE:v2"
case "$(serving)" in *stub-v2*) ;; *) fail "v2 did not take the served ports" ;; esac
pass "cut over to v2"

echo "== 3. update to a build that answers health but cannot infer"
if "$DEPLOY" update "$IMAGE:bad"; then
    fail "the bad build was deployed"
fi
case "$(serving)" in *stub-v2*) ;; *) fail "the serving version changed anyway" ;; esac
case "$(docker ps --format '{{.Names}}')" in *"$NAME-green"*) fail "green was left behind" ;; esac
pass "rejected on the shadow port; v2 still serving"

echo "== 4. rollback"
"$DEPLOY" rollback
case "$(serving)" in *stub-v1*) ;; *) fail "rollback did not restore v1" ;; esac
pass "back on v1"

echo
echo "all four checks passed"
