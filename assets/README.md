# CostNav Assets

This directory contains Omniverse USD assets used by CostNav.

## For Open Source Users (Recommended)

### 1. Download Assets from Hugging Face

```bash
make download-assets-hf
```

This downloads all required assets to `assets/Users/`.

### 2. Initialize and Start Local Nucleus Server

**Prerequisites:**
- Docker and Docker Compose installed
- `.env` file with NGC credentials

**Step 1:** Set up NGC credentials in `.env`:
```bash
cp .env.example .env
# Edit .env and set NGC_PASS to your NGC API key
# Get your API key at: https://org.ngc.nvidia.com/setup/api-keys
```

**Step 2:** Run the nucleus server:
```bash
make start-nucleus
```

The script will configure Nucleus and copy assets. After starting, assets will be served at `omniverse://localhost`.

Modify OMNI_USER and OMNI path into our default credentials

Default credentials:
  Username: omniverse
  Password: costnav123

### 3. Update Codebase Paths

```bash
# Update all code references to use localhost
python scripts/assets/update_asset_paths.py --target localhost

# Dry run to preview changes
python scripts/assets/update_asset_paths.py --target localhost --dry-run
```

## For Internal Developers

### Download from Internal Omniverse Server

```bash
make download-assets-omniverse
```

### Upload to Hugging Face

```bash
# Login to Hugging Face first
huggingface-cli login

# Upload assets
python scripts/assets/upload_assets_hf.py
```
