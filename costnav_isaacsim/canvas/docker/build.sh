#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-canvas}"
IMAGE_TAG="${IMAGE_TAG:-latest}"

echo "Building ${IMAGE_NAME}:${IMAGE_TAG} ..."

docker build \
    -f "${REPO_ROOT}/docker/Dockerfile" \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    --target official \
    "${REPO_ROOT}"
