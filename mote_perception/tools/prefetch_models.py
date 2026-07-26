r"""Download the inference models into the local HuggingFace cache ahead of time.

The servers pull their weights via `from_pretrained` on first use, which means
the very first depth/detect request after a fresh install blocks on a multi-
hundred-MB download. Running this once at setup time (part of the inference-PC
guide) makes the servers start serving immediately and confirms the machine can
reach the HF hub before you rely on it.

    pixi run inference-prefetch          # CPU/dev env
    pixi run inference-prefetch-rocm     # AMD ROCm dev env

The container image runs this at build time, so a deployed server ships with its
weights already cached and never downloads at runtime.

Weights land in the standard HF cache (~/.cache/huggingface, or %USERPROFILE%\
.cache\huggingface on Windows); set HF_HOME to relocate it. Safe to re-run —
already-cached files are skipped.
"""

import sys

DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Small-hf"
DETECT_MODEL = "google/owlv2-base-patch16-ensemble"


def main():
    models = sys.argv[1:] or [DEPTH_MODEL, DETECT_MODEL]
    from transformers import (
        AutoImageProcessor,
        AutoModelForDepthEstimation,
        Owlv2ForObjectDetection,
        Owlv2Processor,
    )

    for model in models:
        print(f"fetching {model} ...", flush=True)
        if "owlv2" in model.lower():
            Owlv2Processor.from_pretrained(model)
            Owlv2ForObjectDetection.from_pretrained(model)
        else:
            AutoImageProcessor.from_pretrained(model)
            AutoModelForDepthEstimation.from_pretrained(model)
        print(f"  done: {model}", flush=True)
    print("all models cached")


if __name__ == "__main__":
    main()
