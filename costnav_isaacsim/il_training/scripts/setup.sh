#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# setup.sh — one-stop setup for il_training
#
# 1. Initialises git submodules (if needed)
# 2. Patches diffusion_policy with a missing __init__.py
#    (upstream ships as a namespace package, but its setup.py uses
#     find_packages() which requires __init__.py)
# 3. Runs `uv sync` to create / update the virtual-env
#
# Usage:
#   cd costnav_isaacsim/il_training && bash scripts/setup.sh
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
DIFFUSION_POLICY_INIT="$PROJECT_ROOT/third_party/diffusion_policy/diffusion_policy/__init__.py"

# Export PROJECT_ROOT for use by Python scripts
export PROJECT_ROOT

# ── 1. git submodules (only the ones il_training needs) ──────────────────────
echo "▸ Initialising git submodules …"
git -C "$PROJECT_ROOT" submodule update --init third_party/diffusion_policy third_party/visualnav-transformer

# ── 2. patch diffusion_policy ────────────────────────────────────────────────
if [ ! -f "$DIFFUSION_POLICY_INIT" ]; then
    touch "$DIFFUSION_POLICY_INIT"
    echo "▸ Created missing $DIFFUSION_POLICY_INIT"
else
    echo "▸ $DIFFUSION_POLICY_INIT already exists — skipping"
fi

# ── 3. uv sync ──────────────────────────────────────────────────────────────
# --reinstall-package forces a rebuild of diffusion-policy so the editable
# finder picks up the __init__.py we just created (uv may have cached the
# old build where it was missing).
echo "▸ Running uv sync …"
cd "$SCRIPT_DIR/.."
uv sync --reinstall-package diffusion-policy --quiet
echo "✔ Done"
