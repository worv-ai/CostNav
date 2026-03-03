#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "${SCRIPT_DIR}/.env" ]; then
    PRE_FIXER_SOURCE_PATH="${FIXER_SOURCE_PATH-}"
    PRE_FIXER_CONTAINER_WORKDIR="${FIXER_CONTAINER_WORKDIR-}"
    PRE_FIXER_CONTAINER_INPUT_DIR="${FIXER_CONTAINER_INPUT_DIR-}"
    PRE_FIXER_MODEL_PATH="${FIXER_MODEL_PATH-}"
    PRE_FIXER_INPUT_DIR="${FIXER_INPUT_DIR-}"
    PRE_FIXER_OUT_DIR="${FIXER_OUT_DIR-}"
    PRE_FIXER_VIDEO_DIR="${FIXER_VIDEO_DIR-}"
    PRE_CAPTURE_DIR="${CAPTURE_DIR-}"
    set -a
    source "${SCRIPT_DIR}/.env"
    set +a
    if [ -z "${FIXER_SOURCE_PATH:-}" ] && [ -n "${PRE_FIXER_SOURCE_PATH:-}" ]; then
        export FIXER_SOURCE_PATH="${PRE_FIXER_SOURCE_PATH}"
    fi
    if [ -z "${FIXER_CONTAINER_WORKDIR:-}" ] && [ -n "${PRE_FIXER_CONTAINER_WORKDIR:-}" ]; then
        export FIXER_CONTAINER_WORKDIR="${PRE_FIXER_CONTAINER_WORKDIR}"
    fi
    if [ -z "${FIXER_CONTAINER_INPUT_DIR:-}" ] && [ -n "${PRE_FIXER_CONTAINER_INPUT_DIR:-}" ]; then
        export FIXER_CONTAINER_INPUT_DIR="${PRE_FIXER_CONTAINER_INPUT_DIR}"
    fi
    if [ -z "${FIXER_MODEL_PATH:-}" ] && [ -n "${PRE_FIXER_MODEL_PATH:-}" ]; then
        export FIXER_MODEL_PATH="${PRE_FIXER_MODEL_PATH}"
    fi
    if [ -z "${FIXER_INPUT_DIR:-}" ] && [ -n "${PRE_FIXER_INPUT_DIR:-}" ]; then
        export FIXER_INPUT_DIR="${PRE_FIXER_INPUT_DIR}"
    fi
    if [ -z "${FIXER_OUT_DIR:-}" ] && [ -n "${PRE_FIXER_OUT_DIR:-}" ]; then
        export FIXER_OUT_DIR="${PRE_FIXER_OUT_DIR}"
    fi
    if [ -z "${FIXER_VIDEO_DIR:-}" ] && [ -n "${PRE_FIXER_VIDEO_DIR:-}" ]; then
        export FIXER_VIDEO_DIR="${PRE_FIXER_VIDEO_DIR}"
    fi
    if [ -z "${CAPTURE_DIR:-}" ] && [ -n "${PRE_CAPTURE_DIR:-}" ]; then
        export CAPTURE_DIR="${PRE_CAPTURE_DIR}"
    fi
fi

INPUT_DIR="${FIXER_INPUT_DIR:-${CAPTURE_DIR:-}}"
OUTPUT_DIR="${FIXER_OUT_DIR:-}"
VIDEO_DIR="${FIXER_VIDEO_DIR:-}"
FIXER_FPS="${FIXER_FPS:-24}"
FIXER_TIMESTEP="${FIXER_TIMESTEP:-250}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --input) INPUT_DIR="$2"; shift 2;;
        --output) OUTPUT_DIR="$2"; shift 2;;
        --fps) FIXER_FPS="$2"; shift 2;;
        --timestep) FIXER_TIMESTEP="$2"; shift 2;;
        --video-dir) VIDEO_DIR="$2"; shift 2;;
        -h|--help)
            echo "Usage: $0 --input <frames_dir> [--output <out_dir>] [--video-dir <dir>] [--fps 24] [--timestep 250]"
            exit 0;;
        *) echo "Unknown arg: $1"; exit 1;;
    esac
  done

if [ -z "$INPUT_DIR" ]; then
    echo "[Error] FIXER_INPUT_DIR or CAPTURE_DIR is not set. Use --input or set in .env."
    exit 1
fi

# Resolve trial structure if INPUT_DIR is a base dir or a trial root
if [ -n "$INPUT_DIR" ] && [ -d "$INPUT_DIR" ]; then
    if [ -d "$INPUT_DIR/capture" ]; then
        trial_root="$INPUT_DIR"
        INPUT_DIR="$trial_root/capture"
        if [ -z "$OUTPUT_DIR" ]; then
            OUTPUT_DIR="$trial_root/fixer"
        fi
        if [ -z "$VIDEO_DIR" ]; then
            VIDEO_DIR="$trial_root"
        fi
    fi

    latest_trial=$(ls -dt "$INPUT_DIR"/teleop_* 2>/dev/null | head -n 1)
    if [ -n "$latest_trial" ] && [ -d "$latest_trial/capture" ]; then
        INPUT_DIR="$latest_trial/capture"
        if [ -z "$OUTPUT_DIR" ]; then
            OUTPUT_DIR="$latest_trial/fixer"
        fi
        if [ -z "$VIDEO_DIR" ]; then
            VIDEO_DIR="$latest_trial"
        fi
    fi
fi

if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="$INPUT_DIR/output"
fi
if [ -z "$VIDEO_DIR" ]; then
    VIDEO_DIR="$OUTPUT_DIR"
fi

# Fixer env
SOURCE_PATH="${FIXER_SOURCE_PATH:-}"
FIXER_CONTAINER_WORKDIR="${FIXER_CONTAINER_WORKDIR:-}"
FIXER_CONTAINER_INPUT_DIR="${FIXER_CONTAINER_INPUT_DIR:-}"
FIXER_MODEL_PATH="${FIXER_MODEL_PATH:-}"
FIXER_DOCKER_IMAGE="${FIXER_DOCKER_IMAGE:-fixer-cosmos-env}"
AUTO_FIXER_SETUP="${AUTO_FIXER_SETUP:-1}"
FIXER_GIT_URL="${FIXER_GIT_URL:-}"
FIXER_GIT_REF="${FIXER_GIT_REF:-}"
FIXER_MODELS_DIR="${FIXER_MODELS_DIR:-}"
FIXER_MODEL_HOST_PATH="${FIXER_MODEL_HOST_PATH:-}"
FIXER_HF_REPO="${FIXER_HF_REPO:-}"
FIXER_DOCKER_CONTEXT="${FIXER_DOCKER_CONTEXT:-}"
FIXER_DOCKERFILE="${FIXER_DOCKERFILE:-}"

if [ -z "$SOURCE_PATH" ] || [ -z "$FIXER_CONTAINER_WORKDIR" ] || [ -z "$FIXER_CONTAINER_INPUT_DIR" ] || [ -z "$FIXER_MODEL_PATH" ]; then
    echo "[Error] FIXER_SOURCE_PATH, FIXER_CONTAINER_WORKDIR, FIXER_CONTAINER_INPUT_DIR, FIXER_MODEL_PATH must be set."
    exit 1
fi

if [ -z "$FIXER_MODELS_DIR" ]; then
    FIXER_MODELS_DIR="${SOURCE_PATH}/models"
fi

if [ -z "$FIXER_MODEL_HOST_PATH" ]; then
    FIXER_MODEL_HOST_PATH="${FIXER_MODELS_DIR}/pretrained/pretrained_fixer.pkl"
fi

if [ -z "$FIXER_DOCKER_CONTEXT" ]; then
    FIXER_DOCKER_CONTEXT="${SOURCE_PATH}"
fi

if [ -z "$FIXER_DOCKERFILE" ]; then
    FIXER_DOCKERFILE="Dockerfile.cosmos"
fi

ensure_fixer_repo() {
    if [ -d "${SOURCE_PATH}/.git" ]; then
        return 0
    fi

    if [ -z "$FIXER_GIT_URL" ]; then
        echo "[Error] FIXER_GIT_URL is not set and repo is missing at ${SOURCE_PATH}."
        exit 1
    fi

    echo "[Setup] Cloning Fixer repo..."
    mkdir -p "$(dirname "${SOURCE_PATH}")"
    git clone "${FIXER_GIT_URL}" "${SOURCE_PATH}"
    if [ -n "$FIXER_GIT_REF" ]; then
        (cd "${SOURCE_PATH}" && git checkout "${FIXER_GIT_REF}")
    fi
}

ensure_fixer_models() {
    if [ -f "${FIXER_MODEL_HOST_PATH}" ]; then
        return 0
    fi

    if [ -z "$FIXER_HF_REPO" ]; then
        echo "[Error] FIXER_HF_REPO is not set and model file is missing."
        exit 1
    fi

    if ! command -v hf >/dev/null 2>&1; then
        echo "[Setup] hf CLI not found. Installing huggingface_hub..."
        if command -v python3 >/dev/null 2>&1; then
            python3 -m pip install -U huggingface_hub
        elif command -v python >/dev/null 2>&1; then
            python -m pip install -U huggingface_hub
        else
            echo "[Error] python not found. Cannot install huggingface_hub."
            exit 1
        fi
    fi

    if ! command -v hf >/dev/null 2>&1; then
        echo "[Error] hf CLI still not found after install. Check PATH."
        exit 1
    fi

    echo "[Setup] Downloading Fixer models from Hugging Face..."
    mkdir -p "${FIXER_MODELS_DIR}"
    hf download "${FIXER_HF_REPO}" --local-dir "${FIXER_MODELS_DIR}"
}

ensure_fixer_docker() {
    if docker image inspect "${FIXER_DOCKER_IMAGE}" >/dev/null 2>&1; then
        return 0
    fi

    local dockerfile_path="${FIXER_DOCKER_CONTEXT}/${FIXER_DOCKERFILE}"
    if [ ! -f "${dockerfile_path}" ]; then
        echo "[Error] Dockerfile not found: ${dockerfile_path}"
        exit 1
    fi

    echo "[Setup] Building Fixer docker image..."
    docker build -t "${FIXER_DOCKER_IMAGE}" -f "${dockerfile_path}" "${FIXER_DOCKER_CONTEXT}"
}

if [ "${AUTO_FIXER_SETUP}" = "1" ]; then
    ensure_fixer_repo
    ensure_fixer_models
    ensure_fixer_docker
fi

mkdir -p "$OUTPUT_DIR"
mkdir -p "$VIDEO_DIR"

# Run Fixer

docker run --rm --gpus=all --ipc=host \
    -u $(id -u):$(id -g) \
    -v "${SOURCE_PATH}:${FIXER_CONTAINER_WORKDIR}" \
    -v "${INPUT_DIR}:${FIXER_CONTAINER_INPUT_DIR}" \
    -v "${OUTPUT_DIR}:${OUTPUT_DIR}" \
    --entrypoint python \
    "${FIXER_DOCKER_IMAGE}" \
    "${FIXER_CONTAINER_WORKDIR}/src/inference_pretrained_model.py" \
    --model "${FIXER_MODEL_PATH}" \
    --input "${FIXER_CONTAINER_INPUT_DIR}" \
    --output "${OUTPUT_DIR}" \
    --timestep "${FIXER_TIMESTEP}"

# Encode video
find "$INPUT_DIR" -maxdepth 1 -type f \( -iname "*.png" -o -iname "*.jpg" \) | sort -V | sed "s/^/file '/;s/$/'/" > before_list.txt
find "$OUTPUT_DIR" -maxdepth 1 -type f \( -iname "*.png" -o -iname "*.jpg" \) | sort -V | sed "s/^/file '/;s/$/'/" > after_list.txt

if [ -s after_list.txt ]; then
    ffmpeg -y -f concat -safe 0 -i after_list.txt \
        -vf "settb=AVTB,setpts=N/${FIXER_FPS}/TB" \
        -r "${FIXER_FPS}" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "${VIDEO_DIR}/after.mp4"

    ffmpeg -y -f concat -safe 0 -i before_list.txt \
        -vf "settb=AVTB,setpts=N/${FIXER_FPS}/TB" \
        -r "${FIXER_FPS}" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "${VIDEO_DIR}/before.mp4"

    ffmpeg -y -f concat -safe 0 -i before_list.txt \
        -f concat -safe 0 -i after_list.txt \
        -filter_complex "[0:v][1:v]hstack,settb=AVTB,setpts=N/${FIXER_FPS}/TB" \
        -r "${FIXER_FPS}" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "${VIDEO_DIR}/comparison.mp4"
fi

rm -f before_list.txt after_list.txt

echo "[Done] Fixer output: ${OUTPUT_DIR}"
