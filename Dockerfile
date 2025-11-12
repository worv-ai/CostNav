# ==============================================================================
# Base stage: Common dependencies for all simulation environments
# ==============================================================================
FROM ubuntu:22.04 AS base

# Install uv for fast Python package management
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Install common system dependencies
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy project files
COPY pyproject.toml ./
COPY README.md ./
COPY costnav/ ./costnav/

# Install base dependencies only (no simulation-specific deps)
RUN uv pip install --system -e ".[dev]"

# ==============================================================================
# Isaac Sim stage: For NVIDIA Isaac Sim environment (base for Isaac Lab)
# ==============================================================================
FROM nvcr.io/nvidia/isaac-sim:5.0.0 AS isaac-sim

# Install uv for faster package management
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /workspace

# Set environment variables for Isaac Sim
ENV ACCEPT_EULA=Y
ENV ISAAC_SIM_PATH=/isaac-sim
ENV PYTHONPATH="${ISAAC_SIM_PATH}/python:${PYTHONPATH}"
ENV PYTHON_BIN="/isaac-sim/kit/python/bin/python3"

# Copy project files
COPY pyproject.toml ./
COPY README.md ./
COPY costnav/ ./costnav/

# Create symlink for python -> python3
RUN ln -sf /isaac-sim/kit/python/bin/python3 /isaac-sim/kit/python/bin/python

# Install with Isaac Sim dependencies (minimal, since Isaac Sim already has most deps)
RUN uv pip install --python="${PYTHON_BIN}" --system -e ".[isaac-sim,dev]"

# Use bash as entrypoint to prevent Isaac Sim auto-start
# The devcontainer will override this with its own command
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "sleep infinity"]

# ==============================================================================
# Isaac Lab stage: Isaac Lab on top of Isaac Sim
# ==============================================================================
FROM nvcr.io/nvidia/isaac-sim:5.0.0 AS isaac-lab

# Install uv for faster package management and git for cloning repos
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Install git (needed for installing packages from GitHub)
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Set environment variables for Isaac Sim
ENV ACCEPT_EULA=Y
ENV ISAAC_SIM_PATH=/isaac-sim
ENV PYTHONPATH="${ISAAC_SIM_PATH}/python:${PYTHONPATH}"
ENV PYTHON_BIN="/isaac-sim/kit/python/bin/python3"

# Create symlink for python -> python3
RUN ln -sf /isaac-sim/kit/python/bin/python3 /isaac-sim/kit/python/bin/python

# Install PyTorch with CUDA 12.8 support (required for Isaac Lab)
RUN uv pip install --python="${PYTHON_BIN}" --system \
    torch==2.7.0 \
    torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128

# Install rl_games from Isaac Sim's fork (Python 3.11 compatible)
RUN uv pip install --python="${PYTHON_BIN}" --system \
    git+https://github.com/isaac-sim/rl_games.git@python3.11

# Install Isaac Lab with all extensions
# Using Isaac Sim's Python to install globally in the container (no venv needed)
RUN uv pip install --python="${PYTHON_BIN}" --system \
    "isaaclab[isaacsim,all]==2.2.0" \
    --extra-index-url https://pypi.nvidia.com

# Copy project files
COPY pyproject.toml ./
COPY README.md ./
COPY costnav/ ./costnav/
COPY costnav_isaaclab/ ./costnav_isaaclab/

# Install costnav with Isaac Lab dependencies
RUN uv pip install --python="${PYTHON_BIN}" --system -e ".[isaac-lab,dev]"

# Install costnav_isaaclab project (from template)
RUN uv pip install --python="${PYTHON_BIN}" --system -e costnav_isaaclab/source/costnav_isaaclab

RUN printf '%s\n' '#!/usr/bin/env bash' \
    'exec /isaac-sim/python.sh "$@"' > /usr/local/bin/python && \
    chmod +x /usr/local/bin/python && \
    ln -sf /usr/local/bin/python /usr/local/bin/python3

# Use bash as entrypoint to prevent Isaac Sim auto-start
# The devcontainer will override this with its own command
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "sleep infinity"]

# ==============================================================================
# Development stage: Minimal setup for development without heavy simulation deps
# ==============================================================================
FROM base AS dev

# This stage only has base dependencies
# Developers can install specific simulation deps as needed
