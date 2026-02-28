#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "${SCRIPT_DIR}/.env" ]; then
    set -a
    source "${SCRIPT_DIR}/.env"
    set +a
fi

ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-${ISAAC_PYTHON:-}}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-}"
ROS_SETUP_ZSH="${ROS_SETUP_ZSH:-}"
ROS_PYTHON_SITE_PACKAGES="${ROS_PYTHON_SITE_PACKAGES:-}"

RUN_FIXER_TO_VIDEO="${RUN_FIXER_TO_VIDEO:-0}"
FIXER_INPUT_ARG=""
FIXER_OUTPUT_ARG=""
FIXER_FPS_ARG=""
FIXER_TIMESTEP_ARG=""

REMAINING_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-fixer)
            RUN_FIXER_TO_VIDEO=1
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
        --fixer-fps)
            FIXER_FPS_ARG="$2"
            shift 2
            ;;
        --fixer-timestep)
            FIXER_TIMESTEP_ARG="$2"
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

if [ "${RUN_FIXER_TO_VIDEO}" = "1" ]; then
    if [ -x "${SCRIPT_DIR}/fixer_to_video.sh" ]; then
        FIXER_ARGS=()
        [ -n "${FIXER_INPUT_ARG}" ] && FIXER_ARGS+=(--input "${FIXER_INPUT_ARG}")
        [ -n "${FIXER_OUTPUT_ARG}" ] && FIXER_ARGS+=(--output "${FIXER_OUTPUT_ARG}")
        [ -n "${FIXER_FPS_ARG}" ] && FIXER_ARGS+=(--fps "${FIXER_FPS_ARG}")
        [ -n "${FIXER_TIMESTEP_ARG}" ] && FIXER_ARGS+=(--timestep "${FIXER_TIMESTEP_ARG}")
        "${SCRIPT_DIR}/fixer_to_video.sh" "${FIXER_ARGS[@]}"
    else
        echo "[Warn] fixer_to_video.sh not found or not executable."
    fi
fi
