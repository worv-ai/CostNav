# === Base stage with shared deps ===
FROM ubuntu:24.04 AS base

# uv handles Python installs
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# System packages
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY pyproject.toml ./
COPY README.md ./

# Core dev extras
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system -e ".[dev]"

# === Isaac Sim image ===
FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS isaac-sim

# Reuse uv binary
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /workspace

ENV ACCEPT_EULA=Y
ENV ISAAC_PATH=/isaac-sim
ENV ISAAC_SIM_PATH=/isaac-sim
ENV CARB_APP_PATH=/isaac-sim/kit
ENV EXP_PATH=/isaac-sim/apps
ENV PYTHON_BIN="/isaac-sim/kit/python/bin/python3"

# Set PYTHONPATH to include all Isaac Sim, Omni Kit, and extension directories
ENV PYTHONPATH="\
${ISAAC_PATH}/kit/kernel/py:\
${ISAAC_PATH}/kit/exts:\
${ISAAC_PATH}/kit/extscore:\
${ISAAC_PATH}/exts:\
${ISAAC_PATH}/extscache:\
${ISAAC_PATH}/extsDeprecated:\
${ISAAC_PATH}/extsUser:\
${ISAAC_PATH}/kit/python/lib/python3.11/site-packages:\
${PYTHONPATH}"

# Set LD_LIBRARY_PATH for binary interfaces
ENV LD_LIBRARY_PATH="\
${ISAAC_PATH}/kit/lib:\
${ISAAC_PATH}/kit/plugins:\
${ISAAC_PATH}/kit/exts:\
${LD_LIBRARY_PATH}"

COPY pyproject.toml ./
COPY README.md ./

# Copy custom extensions to Isaac Sim extensions folder
COPY extensions/ /isaac-sim/extsUser/

# python -> python3 shim
RUN ln -sf /isaac-sim/kit/python/bin/python3 /isaac-sim/kit/python/bin/python

# Isaac Sim extras
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system -e ".[isaac-sim,dev]"

# Keep container idle
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "sleep infinity"]

# === Isaac Lab image ===
FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS isaac-lab

# uv + git for installs
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

USER root
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENV ACCEPT_EULA=Y
ENV ISAAC_PATH=/isaac-sim
ENV ISAAC_SIM_PATH=/isaac-sim
ENV CARB_APP_PATH=/isaac-sim/kit
ENV EXP_PATH=/isaac-sim/apps
ENV PYTHON_BIN="/isaac-sim/kit/python/bin/python3"

# Set PYTHONPATH to include all Isaac Sim, Omni Kit, and extension directories
ENV PYTHONPATH="\
${ISAAC_PATH}/kit/kernel/py:\
${ISAAC_PATH}/kit/exts:\
${ISAAC_PATH}/kit/extscore:\
${ISAAC_PATH}/exts:\
${ISAAC_PATH}/extscache:\
${ISAAC_PATH}/extsDeprecated:\
${ISAAC_PATH}/extsUser:\
${ISAAC_PATH}/kit/python/lib/python3.11/site-packages:\
${PYTHONPATH}"

# Set LD_LIBRARY_PATH for binary interfaces
ENV LD_LIBRARY_PATH="\
${ISAAC_PATH}/kit/lib:\
${ISAAC_PATH}/kit/plugins:\
${ISAAC_PATH}/kit/exts:\
${LD_LIBRARY_PATH}"

# python -> python3 shim
RUN ln -sf /isaac-sim/kit/python/bin/python3 /isaac-sim/kit/python/bin/python

# PyTorch CUDA 12.8
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system \
    torch==2.7.0 \
    torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128

# Isaac Sim rl_games fork
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system \
    git+https://github.com/isaac-sim/rl_games.git@python3.11

# Isaac Lab + extras
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system \
    "isaaclab[isaacsim,all]==2.3.0" \
    --extra-index-url https://pypi.nvidia.com

COPY pyproject.toml ./
COPY README.md ./
COPY costnav_isaaclab/ ./costnav_isaaclab/

# Copy custom extensions to Isaac Sim extensions folder
COPY extensions/ /isaac-sim/extsUser/

# CostNav Isaac Lab deps
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system -e ".[isaac-lab,dev]"

# Template project install
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python="${PYTHON_BIN}" --system -e costnav_isaaclab/source/costnav_isaaclab

RUN printf '%s\n' '#!/usr/bin/env bash' \
    'exec /isaac-sim/python.sh "$@"' > /usr/local/bin/python && \
    chmod +x /usr/local/bin/python && \
    ln -sf /usr/local/bin/python /usr/local/bin/python3

# Keep container idle
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "sleep infinity"]

# === Dev-only stage ===
FROM base AS dev

# Extra sim deps can be added ad-hoc
