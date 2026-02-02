# CostNav Assets

This directory contains Omniverse USD assets used by CostNav and scripts to manage them.

## For Open Source Users (Recommended)

### 1. Download Assets from Hugging Face

```bash
python assets/download_assets_hf.py
```

This downloads all required assets to `assets/Users/`.

### 2. Start Local Nucleus Server

**Prerequisites:**
- Docker and Docker Compose installed
- NGC account with Omniverse Enterprise access
- Downloaded nucleus-stack from NGC

**Step 1:** Download nucleus-stack from NGC (one-time setup):
```bash
# Option A: Using NGC CLI
ngc registry resource download-version "nvidia/omniverse/nucleus-stack:2023.2.9"

# Option B: Manual download from
# https://catalog.ngc.nvidia.com/orgs/nvidia/teams/omniverse/collections/nucleus
```

**Step 2:** Extract to /opt/ove:
```bash
sudo mkdir -p /opt/ove
sudo tar xzvf nucleus-stack-*.tar.gz -C /opt/ove --strip-components=1
```

**Step 3:** Run the setup script:
```bash
./assets/nucleus/start_nucleus.sh
```

The script will configure Nucleus and copy assets. After starting, assets will be served at `omniverse://localhost`.

### 3. Update Codebase Paths

```bash
# Update all code references to use localhost
python assets/update_asset_paths.py --target localhost

# Dry run to preview changes
python assets/update_asset_paths.py --target localhost --dry-run
```

## For Internal Developers

### Download from Internal Omniverse Server

```bash
docker compose --profile isaac-lab run --rm isaac-lab \
    python /workspace/assets/download_omniverse_assets.py
```

### Upload to Hugging Face

```bash
# Login to Hugging Face first
huggingface-cli login

# Upload assets
python assets/upload_assets_hf.py
```

## Directory Structure

```
assets/
├── README.md                      # This file
├── download_assets_hf.py          # Download from Hugging Face
├── upload_assets_hf.py            # Upload to Hugging Face
├── download_omniverse_assets.py   # Download from internal Omniverse server
├── update_asset_paths.py          # Update codebase paths
├── nucleus/                       # Local Nucleus server setup
│   ├── docker-compose.yml
│   └── start_nucleus.sh
└── Users/                         # Downloaded assets (git-ignored)
    └── worv/
        └── costnav/
            └── ...
```

## Asset Locations

| Asset | Path |
|-------|------|
| Street Sidewalk Map | `Users/worv/costnav/Street_sidewalk.usd` |
| Segway E1 Map | `Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd` |
| Popcorn Food Asset | `Users/worv/costnav/foods/popcorn/popcorn.usd` |
