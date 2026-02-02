#!/bin/bash
# Start local Omniverse Nucleus server for CostNav assets
#
# Prerequisites:
#   1. Docker and Docker Compose installed
#   2. NGC CLI configured (for pulling NVIDIA images)
#   3. Assets downloaded via: python assets/download_assets_hf.py
#
# Usage:
#   ./assets/nucleus/start_nucleus.sh
#
# After starting, assets will be available at:
#   omniverse://localhost/Users/worv/...

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSETS_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "CostNav Local Nucleus Server"
echo "============================================================"

# Check if assets exist
if [ ! -d "$ASSETS_DIR/Users" ]; then
    echo "ERROR: Assets not found at $ASSETS_DIR/Users"
    echo ""
    echo "Please download assets first:"
    echo "  python assets/download_assets_hf.py"
    exit 1
fi

echo "Assets directory: $ASSETS_DIR/Users"
echo ""

# Check Docker
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed."
    exit 1
fi

# Check if logged into NGC
echo "Checking NGC authentication..."
if ! docker pull nvcr.io/nvidia/omniverse/nucleus:2023.2.9 2>/dev/null; then
    echo ""
    echo "ERROR: Cannot pull Nucleus image from NGC."
    echo ""
    echo "Please login to NGC first:"
    echo "  docker login nvcr.io"
    echo ""
    echo "Use your NGC API key as the password."
    echo "Get your API key at: https://ngc.nvidia.com/setup/api-key"
    exit 1
fi

cd "$SCRIPT_DIR"

echo ""
echo "Starting Nucleus server..."
docker compose up -d

echo ""
echo "============================================================"
echo "Nucleus server starting..."
echo ""
echo "Web UI:     http://localhost:8080"
echo "Omniverse:  omniverse://localhost"
echo ""
echo "Default credentials:"
echo "  Username: omniverse"
echo "  Password: costnav123 (or set NUCLEUS_PASSWORD env var)"
echo ""
echo "To stop:    docker compose -f $SCRIPT_DIR/docker-compose.yml down"
echo "To logs:    docker compose -f $SCRIPT_DIR/docker-compose.yml logs -f"
echo "============================================================"

