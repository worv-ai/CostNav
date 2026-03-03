#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
FIXER_RAN_FLAG="${SCRIPT_DIR}/.fixer_ran"
rm -f "${FIXER_RAN_FLAG}"

if [ -f "${SCRIPT_DIR}/.env" ]; then
    PRE_FIXER_SOURCE_PATH="${FIXER_SOURCE_PATH-}"
    PRE_FIXER_CONTAINER_WORKDIR="${FIXER_CONTAINER_WORKDIR-}"
    PRE_FIXER_CONTAINER_INPUT_DIR="${FIXER_CONTAINER_INPUT_DIR-}"
    PRE_FIXER_MODEL_PATH="${FIXER_MODEL_PATH-}"
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
fi

ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-${ISAAC_PYTHON:-}}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-}"
ROS_SETUP_ZSH="${ROS_SETUP_ZSH:-}"
ROS_PYTHON_SITE_PACKAGES="${ROS_PYTHON_SITE_PACKAGES:-}"

RUN_FIXER_TO_VIDEO="${RUN_FIXER_TO_VIDEO:-0}"
FIXER_INPUT_ARG=""
FIXER_OUTPUT_ARG=""
FIXER_VIDEO_ARG=""
FIXER_FPS_ARG=""
FIXER_TIMESTEP_ARG=""
FIXER_SOURCE_ARG=""
CAPTURE_DIR_ARG=""

REMAINING_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-fixer)
            RUN_FIXER_TO_VIDEO=1
            export RUN_FIXER_TO_VIDEO=1
            shift
            ;;
        --fixer-input)
            FIXER_INPUT_ARG="$2"
            shift 2
            ;;
        --fixer-output)
            FIXER_OUTPUT_ARG="$2"
            shift 2
            ;;
        --fixer-video-dir)
            FIXER_VIDEO_ARG="$2"
            shift 2
            ;;
        --fixer-fps)
            FIXER_FPS_ARG="$2"
            shift 2
            ;;
        --fixer-timestep)
            FIXER_TIMESTEP_ARG="$2"
            shift 2
            ;;
        --fixer-source-path)
            FIXER_SOURCE_ARG="$2"
            export FIXER_SOURCE_PATH="$2"
            shift 2
            ;;
        --capture-dir)
            CAPTURE_DIR_ARG="$2"
            export CAPTURE_DIR="$2"
            REMAINING_ARGS+=("$1" "$2")
            shift 2
            ;;
        *)
            REMAINING_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ -z "$ISAAC_SIM_PYTHON" ]; then
    echo "[Error] ISAAC_SIM_PYTHON not set. Configure it in .env."
    exit 1
fi

if [ -z "$ROS_SETUP_SCRIPT" ]; then
    echo "[Error] ROS_SETUP_SCRIPT not set. Configure it in .env."
    exit 1
fi

if [[ "$SHELL" == *"zsh"* ]] && [ -n "$ROS_SETUP_ZSH" ]; then
    echo "[Pipeline] Zsh detected. Sourcing ROS setup: $ROS_SETUP_ZSH"
    source "$ROS_SETUP_ZSH"
else
    echo "[Pipeline] Bash detected. Sourcing ROS setup: $ROS_SETUP_SCRIPT"
    source "$ROS_SETUP_SCRIPT"
fi

if [ -n "$ROS_PYTHON_SITE_PACKAGES" ]; then
    export PYTHONPATH
    PYTHONPATH=$(echo "$PYTHONPATH" | sed -e "s|${ROS_PYTHON_SITE_PACKAGES}||g")
fi

if [ -f "$ISAAC_SIM_PYTHON" ]; then
    "$ISAAC_SIM_PYTHON" "${SCRIPT_DIR}/run_pipeline.py" "${REMAINING_ARGS[@]}"
else
    echo "[Error] Isaac Sim launcher not found at: $ISAAC_SIM_PYTHON"
    exit 1
fi

if [ "${RUN_FIXER_TO_VIDEO}" = "1" ] && [ ! -f "${FIXER_RAN_FLAG}" ]; then
    if [ -x "${SCRIPT_DIR}/fixer_to_video.sh" ]; then
        FIXER_ARGS=()
        if [ -n "${FIXER_INPUT_ARG}" ]; then
            FIXER_ARGS+=(--input "${FIXER_INPUT_ARG}")
        elif [ -n "${CAPTURE_DIR_ARG}" ]; then
            FIXER_ARGS+=(--input "${CAPTURE_DIR_ARG}")
        fi
        [ -n "${FIXER_OUTPUT_ARG}" ] && FIXER_ARGS+=(--output "${FIXER_OUTPUT_ARG}")
        [ -n "${FIXER_VIDEO_ARG}" ] && FIXER_ARGS+=(--video-dir "${FIXER_VIDEO_ARG}")
        [ -n "${FIXER_FPS_ARG}" ] && FIXER_ARGS+=(--fps "${FIXER_FPS_ARG}")
        [ -n "${FIXER_TIMESTEP_ARG}" ] && FIXER_ARGS+=(--timestep "${FIXER_TIMESTEP_ARG}")
        "${SCRIPT_DIR}/fixer_to_video.sh" "${FIXER_ARGS[@]}"
    else
        echo "[Warn] fixer_to_video.sh not found or not executable."
    fi
elif [ "${RUN_FIXER_TO_VIDEO}" = "1" ] && [ -f "${FIXER_RAN_FLAG}" ]; then
    echo "[Pipeline] Fixer already ran on timeline stop. Skipping post-run fixer."
fi
