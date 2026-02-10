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


### 3. Set OMNI_URL in dotenv

```bash
OMNI_USER=omniverse
OMNI_PASS=costnav123
OMNI_URL=omniverse://localhost
```

## For Internal Developers

### Download from Internal Omniverse Server

```bash
make download-assets-omniverse
```

### Upload to Hugging Face

```bash
make upload-assets-hf
```
