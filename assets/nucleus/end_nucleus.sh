#!/bin/bash
# Stop local Omniverse Nucleus server stack
#
# Usage:
#   ./assets/nucleus/end_nucleus.sh

set -e

NUCLEUS_INSTALL_DIR="/opt/ove"

echo "============================================================"
echo "Stopping CostNav Nucleus Server"
echo "============================================================"

# Check if running as root for system directories
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo for system directory access."
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

if [ ! -d "$NUCLEUS_INSTALL_DIR/base_stack" ]; then
    echo "ERROR: Nucleus stack not found at $NUCLEUS_INSTALL_DIR/base_stack"
    echo "Nothing to stop."
    exit 1
fi

cd "$NUCLEUS_INSTALL_DIR/base_stack"

echo "Stopping Nucleus containers..."
docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml down

echo ""
echo "============================================================"
echo "Nucleus server stopped."
echo "============================================================"
