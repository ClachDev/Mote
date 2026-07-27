# A stand-in for the inference image, for exercising the deploy pipeline.
#
# Same layout as mote_perception/deploy/Dockerfile — /app/tools/probe.py, the
# wire package, an entrypoint that binds 5601/5602 — but with a stub in place of
# torch, so it builds in seconds anywhere instead of pulling a CUDA wheel and
# baking model weights. See deploy/test/drill.sh.
FROM python:3.12-slim

ARG MOTE_VERSION=stub
ARG STUB_MODE=serve
ENV MOTE_VERSION=${MOTE_VERSION} \
    STUB_MODE=${STUB_MODE} \
    PYTHONUNBUFFERED=1

RUN pip install --no-cache-dir "numpy>=1.26,<3" "pillow>=10,<13"

WORKDIR /app
COPY mote_perception/mote_perception/__init__.py \
     mote_perception/mote_perception/depth_wire.py \
     mote_perception/mote_perception/detect_wire.py \
     /app/mote_perception/
# The real probe, unmodified: the point of the drill is that the gate which
# runs in production is the gate under test.
COPY mote_perception/tools/probe.py /app/tools/
COPY mote_perception/deploy/test/stub_server.py /app/tools/

EXPOSE 5601 5602

HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python /app/tools/probe.py --no-infer --timeout 5 || exit 1

ENTRYPOINT ["sh", "-c", "exec python -u /app/tools/stub_server.py --mode $STUB_MODE \"$@\"", "--"]
