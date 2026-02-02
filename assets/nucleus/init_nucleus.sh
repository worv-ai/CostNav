#!/bin/bash
# Initialize Omniverse Nucleus Server for CostNav
#
# This script automates:
#   1. NGC CLI installation
#   2. nucleus-compose-stack download from NGC
#   3. Extraction to /opt/ove
#
# Prerequisites:
#   - .env file with NGC_USER and NGC_PASS set
#   - sudo access
#
# Usage:
#   ./assets/nucleus/init_nucleus.sh
#
# After running this script, run start_nucleus.sh to start the server.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
NUCLEUS_INSTALL_DIR="/opt/ove"
NGC_CLI_VERSION="3.41.4"

echo "============================================================"
echo "CostNav Nucleus Server Initialization"
echo "============================================================"

# Load .env file
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

# Check if running as root for system directories
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo for system directory access."
    echo "Re-running with sudo (preserving environment)..."
    exec sudo -E "$0" "$@"
fi

# Step 1: Install NGC CLI if not present
echo "============================================================"
echo "Step 1: NGC CLI Installation"
echo "============================================================"

if command -v ngc &> /dev/null && [ -x /usr/local/ngc-cli/ngc ]; then
    echo "NGC CLI already installed: $(ngc --version 2>/dev/null || echo 'unknown version')"
else
    echo "Installing NGC CLI v${NGC_CLI_VERSION}..."
    
    cd /tmp
    
    # Download NGC CLI
    wget -q --show-progress --content-disposition \
        "https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/${NGC_CLI_VERSION}/files/ngccli_linux.zip" \
        -O ngccli_linux.zip
    
    # Unzip
    unzip -q -o ngccli_linux.zip
    
    # Install full CLI bundle so libpython can be found
    chmod +x ngc-cli/ngc
    rm -rf /usr/local/ngc-cli
    mv ngc-cli /usr/local/ngc-cli

    # Wrapper ensures the executable resolves from /usr/local/ngc-cli
    cat > /usr/local/bin/ngc <<'EOF'
#!/bin/bash
NGC_CLI_DIR="/usr/local/ngc-cli"
export LD_LIBRARY_PATH="${NGC_CLI_DIR}:${LD_LIBRARY_PATH:-}"
exec "${NGC_CLI_DIR}/ngc" "$@"
EOF
    chmod +x /usr/local/bin/ngc
    
    # Clean up
    rm -rf ngccli_linux.zip
    
    echo "NGC CLI installed successfully."
fi

# Configure NGC CLI
echo ""
echo "Configuring NGC CLI..."
mkdir -p ~/.ngc
cat > ~/.ngc/config <<EOF
[CURRENT]
apikey = ${NGC_PASS}
format_type = ascii
org = nvidia
EOF
echo "NGC CLI configured."

# Step 2: Download nucleus-compose-stack
echo ""
echo "============================================================"
echo "Step 2: Download Nucleus Stack"
echo "============================================================"

if [ -d "$NUCLEUS_INSTALL_DIR/base_stack" ]; then
    echo "Nucleus stack already exists at $NUCLEUS_INSTALL_DIR"
    echo "Skipping download. Delete $NUCLEUS_INSTALL_DIR to re-download."
else
    echo "Downloading nucleus-compose-stack from NGC..."
    
    cd /tmp
    
    # Download using NGC CLI
    ngc registry resource download-version "nvidia/omniverse/nucleus-compose-stack:2023.2.9" --dest /tmp
    
    # Create install directory
    mkdir -p "$NUCLEUS_INSTALL_DIR"
    
    # Extract
    echo "Extracting to $NUCLEUS_INSTALL_DIR..."
    STACK_DIR="/tmp/nucleus-compose-stack_v2023.2.9"
    TARBALL_PATH="$(find "$STACK_DIR" -maxdepth 2 -type f \( -name "*.tar.gz" -o -name "*.tgz" \) | head -n 1)"
    if [ -z "$TARBALL_PATH" ]; then
        echo "ERROR: Could not locate nucleus stack tarball in $STACK_DIR"
        echo "Contents:"
        ls -la "$STACK_DIR"
        exit 1
    fi
    tar xzf "$TARBALL_PATH" -C "$NUCLEUS_INSTALL_DIR" --strip-components=1
    
    # Clean up
    rm -rf /tmp/nucleus-compose-stack_v2023.2.9
    
    echo "Nucleus stack installed to $NUCLEUS_INSTALL_DIR"
fi

# Login to Docker NGC registry
echo ""
echo "============================================================"
echo "Step 3: Docker NGC Login"
echo "============================================================"
echo "Logging into nvcr.io..."
echo "$NGC_PASS" | docker login nvcr.io -u "$NGC_USER" --password-stdin
echo "Docker NGC login successful."

echo ""
echo "============================================================"
echo "Initialization Complete!"
echo "============================================================"
echo ""
echo "Next steps:"
echo "  1. Download assets:  python assets/download_assets_hf.py"
echo "  2. Start Nucleus:    ./assets/nucleus/start_nucleus.sh"
echo "  3. Update paths:     python assets/update_asset_paths.py --target localhost"
echo ""
