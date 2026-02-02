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
REPO_ROOT="$(dirname "$ASSETS_DIR")"
NUCLEUS_INSTALL_DIR="/opt/ove"
NUCLEUS_DATA_DIR="/var/lib/omni/nucleus-data"
NUCLEUS_WEB_PORT="${NUCLEUS_WEB_PORT:-8080}"
NUCLEUS_PORT="${NUCLEUS_PORT:-3009}"

wait_for_port() {
    local host="$1"
    local port="$2"
    local timeout="$3"
    local start_ts
    start_ts="$(date +%s)"
    while true; do
        if (echo >"/dev/tcp/${host}/${port}") >/dev/null 2>&1; then
            return 0
        fi
        if [ $(( "$(date +%s)" - start_ts )) -ge "$timeout" ]; then
            return 1
        fi
        sleep 2
    done
}

upsert_env_var() {
    local file="$1"
    local key="$2"
    local value="$3"
    if grep -qE "^${key}=" "$file"; then
        sed -i "s/^${key}=.*/${key}=${value}/" "$file"
    elif grep -qE "^#${key}=" "$file"; then
        sed -i "s/^#${key}=.*/${key}=${value}/" "$file"
    else
        echo "${key}=${value}" >> "$file"
    fi
}

echo "============================================================"
echo "CostNav Local Nucleus Server Setup"
echo "============================================================"

# Check if running as root for system directories
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo for system directory access."
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

# Load .env file for NGC credentials
if [ -f "$REPO_ROOT/.env" ]; then
    echo "Loading credentials from .env file..."
    set -a
    source "$REPO_ROOT/.env"
    set +a
else
    echo "ERROR: .env file not found at $REPO_ROOT/.env"
    echo ""
    echo "Please create .env file with NGC credentials:"
    echo "  cp .env.example .env"
    echo "  # Edit .env and set NGC_PASS to your NGC API key"
    exit 1
fi

# Check NGC credentials
if [ -z "$NGC_PASS" ]; then
    echo "ERROR: NGC_PASS not set in .env file"
    echo ""
    echo "Please set your NGC API key in .env:"
    echo "  NGC_PASS=your_ngc_api_key_here"
    echo ""
    echo "Get your API key at: https://ngc.nvidia.com/setup/api-key"
    exit 1
fi

NGC_USER="${NGC_USER:-\$oauthtoken}"
echo "NGC User: $NGC_USER"
echo ""

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

# Stop any existing Nucleus containers before start
if docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml ps -q | grep -q .; then
    echo "Existing Nucleus containers detected. Stopping..."
    docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml down
fi

# Check if already configured
if [ ! -f "nucleus-stack.env.configured" ]; then
    echo "Configuring Nucleus stack..."

    # Backup original env file
    cp nucleus-stack.env nucleus-stack.env.backup 2>/dev/null || true

    # Configure nucleus-stack.env
    upsert_env_var nucleus-stack.env "ACCEPT_EULA" "1"
    upsert_env_var nucleus-stack.env "SECURITY_REVIEWED" "1"
    upsert_env_var nucleus-stack.env "SERVER_IP_OR_HOST" "localhost"
    upsert_env_var nucleus-stack.env "DATA_ROOT" "/var/lib/omni/nucleus-data"
    upsert_env_var nucleus-stack.env "MASTER_PASSWORD" "costnav123"
    upsert_env_var nucleus-stack.env "SERVICE_PASSWORD" "costnav123"

    # Generate secrets if script exists
    if [ -f "generate-sample-insecure-secrets.sh" ]; then
        echo "Generating secrets..."
        ./generate-sample-insecure-secrets.sh
    fi

    touch nucleus-stack.env.configured
    echo "Configuration complete."
fi

# Ensure required env vars even if already configured
upsert_env_var nucleus-stack.env "ACCEPT_EULA" "1"
upsert_env_var nucleus-stack.env "SECURITY_REVIEWED" "1"

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
echo "Logging into nvcr.io..."
if ! echo "$NGC_PASS" | docker login nvcr.io -u "$NGC_USER" --password-stdin; then
    echo ""
    echo "NGC login failed."
    echo "Use \$oauthtoken as username and your NGC API key as password."
    echo "Get your API key at: https://ngc.nvidia.com/setup/api-key"
    exit 1
fi
echo "Docker NGC login successful."

# Pull all containers
echo ""
echo "Pulling Nucleus containers..."
docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml pull

# Start the stack
echo ""
echo "Starting Nucleus server..."
docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml up -d

echo ""
echo "Waiting for Nucleus services to become available..."
if ! wait_for_port "127.0.0.1" "$NUCLEUS_WEB_PORT" 120; then
    echo "ERROR: Web UI did not become available on port ${NUCLEUS_WEB_PORT} within 120s."
    echo "Try: cd $NUCLEUS_INSTALL_DIR/base_stack && docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml logs -f"
    exit 1
fi
if ! wait_for_port "127.0.0.1" "$NUCLEUS_PORT" 120; then
    echo "ERROR: Nucleus service did not become available on port ${NUCLEUS_PORT} within 120s."
    echo "If your compose uses a different port, set NUCLEUS_PORT and re-run:"
    echo "  NUCLEUS_PORT=3009 sudo ./assets/nucleus/start_nucleus.sh"
    exit 1
fi

echo ""
echo "============================================================"
echo "Nucleus server started!"
echo ""
echo "Web UI:     http://localhost:${NUCLEUS_WEB_PORT}"
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
echo "  Test:  python assets/nucleus/test_nucleus_connection.py"
echo "============================================================"
