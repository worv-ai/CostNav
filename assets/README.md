### download using hf

python assets/download_assets_hf.py

### download command from omniverse

docker compose --profile isaac-lab run --rm isaac-lab python /workspace/assets/download_omniverse_assets.py

### upload into hf

login into hf

huggingface-cli login

python assets/upload_assets_hf.py

