#!/bin/bash
# Setup and start local Omniverse Nucleus server for CostNav assets
#
# This script follows the official NVIDIA Nucleus Enterprise installation guide.
# See: https://docs.omniverse.nvidia.com/nucleus/latest/enterprise/installation/install-ove-nucleus.html
#
# Prerequisites:
#   1. Docker and Docker Compose installed
#   2. NGC account with Omniverse Enterprise access
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
NUCLEUS_INSTALL_DIR="/opt/ove"
NUCLEUS_DATA_DIR="/var/lib/omni/nucleus-data"

echo "============================================================"
echo "CostNav Local Nucleus Server Setup"
echo "============================================================"

# Check if running as root for system directories
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo for system directory access."
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

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
    echo "Please install Docker first: https://docs.docker.com/engine/install/"
    exit 1
fi

# Check if nucleus-stack is already installed
if [ ! -d "$NUCLEUS_INSTALL_DIR/base_stack" ]; then
    echo "============================================================"
    echo "Nucleus stack not found at $NUCLEUS_INSTALL_DIR"
    echo "============================================================"
    echo ""
    echo "You need to download the Nucleus stack from NGC first:"
    echo ""
    echo "Option 1 - Using NGC CLI:"
    echo "  ngc registry resource download-version 'nvidia/omniverse/nucleus-stack:2023.2.9'"
    echo ""
    echo "Option 2 - Manual download:"
    echo "  1. Go to: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/omniverse/collections/nucleus"
    echo "  2. Click 'Nucleus Compose Stack' -> 'File Browser' -> 'Download'"
    echo ""
    echo "After downloading, extract it:"
    echo "  sudo mkdir -p $NUCLEUS_INSTALL_DIR"
    echo "  sudo tar xzvf nucleus-stack-*.tar.gz -C $NUCLEUS_INSTALL_DIR --strip-components=1"
    echo ""
    echo "Then run this script again."
    exit 1
fi

cd "$NUCLEUS_INSTALL_DIR/base_stack"

# Check if already configured
if [ ! -f "nucleus-stack.env.configured" ]; then
    echo "Configuring Nucleus stack..."

    # Backup original env file
    cp nucleus-stack.env nucleus-stack.env.backup 2>/dev/null || true

    # Configure nucleus-stack.env
    sed -i 's/^#ACCEPT_EULA=.*/ACCEPT_EULA=1/' nucleus-stack.env
    sed -i 's/^#SECURITY_REVIEWED=.*/SECURITY_REVIEWED=1/' nucleus-stack.env
    sed -i 's/^SERVER_IP_OR_HOST=.*/SERVER_IP_OR_HOST=localhost/' nucleus-stack.env
    sed -i 's/^#SERVER_IP_OR_HOST=.*/SERVER_IP_OR_HOST=localhost/' nucleus-stack.env
    sed -i 's/^DATA_ROOT=.*/DATA_ROOT=\/var\/lib\/omni\/nucleus-data/' nucleus-stack.env
    sed -i 's/^#DATA_ROOT=.*/DATA_ROOT=\/var\/lib\/omni\/nucleus-data/' nucleus-stack.env
    sed -i 's/^MASTER_PASSWORD=.*/MASTER_PASSWORD=costnav123/' nucleus-stack.env
    sed -i 's/^#MASTER_PASSWORD=.*/MASTER_PASSWORD=costnav123/' nucleus-stack.env
    sed -i 's/^SERVICE_PASSWORD=.*/SERVICE_PASSWORD=costnav123/' nucleus-stack.env
    sed -i 's/^#SERVICE_PASSWORD=.*/SERVICE_PASSWORD=costnav123/' nucleus-stack.env

    # Generate secrets if script exists
    if [ -f "generate-sample-insecure-secrets.sh" ]; then
        echo "Generating secrets..."
        ./generate-sample-insecure-secrets.sh
    fi

    touch nucleus-stack.env.configured
    echo "Configuration complete."
fi

# Create data directory
mkdir -p "$NUCLEUS_DATA_DIR"

# Copy assets to Nucleus data directory
echo ""
echo "Copying assets to Nucleus data directory..."
cp -r "$ASSETS_DIR/Users" "$NUCLEUS_DATA_DIR/"
echo "Assets copied to $NUCLEUS_DATA_DIR/Users"

# Check NGC login
echo ""
echo "Checking NGC authentication..."
if ! docker pull nvcr.io/nvidia/omniverse/nucleus-api:2023.2.9 2>/dev/null; then
    echo ""
    echo "Please login to NGC first:"
    echo "  docker login nvcr.io"
    echo ""
    echo "Use \$oauthtoken as username and your NGC API key as password."
    echo "Get your API key at: https://ngc.nvidia.com/setup/api-key"
    exit 1
fi

# Pull all containers
echo ""
echo "Pulling Nucleus containers..."
docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml pull

# Start the stack
echo ""
echo "Starting Nucleus server..."
docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml up -d

echo ""
echo "============================================================"
echo "Nucleus server started!"
echo ""
echo "Web UI:     http://localhost:8080"
echo "Omniverse:  omniverse://localhost"
echo ""
echo "Default credentials:"
echo "  Username: omniverse"
echo "  Password: costnav123"
echo ""
echo "Assets available at:"
echo "  omniverse://localhost/Users/worv/costnav/..."
echo ""
echo "Commands:"
echo "  Stop:  cd $NUCLEUS_INSTALL_DIR/base_stack && docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml down"
echo "  Logs:  cd $NUCLEUS_INSTALL_DIR/base_stack && docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml logs -f"
echo "============================================================"

