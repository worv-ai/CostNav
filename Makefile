.PHONY: build-isaac-sim build-isaac-lab build-dev build-all fetch-third-party build-ros2 build-vint run-ros2 run-isaac-sim run-isaac-sim-raw run-nav2 run-teleop run-vint start-mission start-mission-record run-rosbag stop-rosbag run-eval-nav2 run-eval-teleop run-eval-vint download-assets-omniverse download-assets-hf upload-assets-hf start-nucleus stop-nucleus

# Load environment variables from .env file if it exists
# Variables can still be overridden from command line
ifneq (,$(wildcard .env))
include .env
export
endif

DOCKERFILE ?= Dockerfile
DOCKER_BUILD ?= docker build
DOCKER_COMPOSE ?= docker compose

# Version defaults (can be overridden by .env or command line)
ISAAC_SIM_VERSION ?= 5.1.0
ISAAC_LAB_VERSION ?= 2.2.0
COSTNAV_VERSION ?= 0.1.0

# ROS configuration
ROS_DISTRO ?= jazzy
UBUNTU_VERSION ?= 24.04
SIM_ROBOT ?= segway_e1
NUM_PEOPLE ?= 20
FOOD ?= True
TUNED ?= True
AMCL ?= False
GOAL_IMAGE ?= False

# Joystick settings for teleop (always reads from .env)
XBOX_ID := $(shell grep '^XBOX_ID=' .env 2>/dev/null | cut -d= -f2)

# model checkpoint path
MODEL_CHECKPOINT ?= checkpoints/vint.pth

ISAAC_SIM_IMAGE ?= costnav-isaacsim-$(ISAAC_SIM_VERSION):$(COSTNAV_VERSION)
ISAAC_LAB_IMAGE ?= costnav-isaaclab-$(ISAAC_SIM_VERSION)-$(ISAAC_LAB_VERSION):$(COSTNAV_VERSION)
COSTNAV_DEV_IMAGE ?= costnav-dev:$(COSTNAV_VERSION)
COSTNAV_ROS2_IMAGE ?= costnav-ros2-jazzy:$(COSTNAV_VERSION)

build-isaac-sim:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-sim \
		--build-arg ISAAC_SIM_VERSION=$(ISAAC_SIM_VERSION) \
		-t $(ISAAC_SIM_IMAGE) .

build-isaac-lab:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-lab \
		--build-arg ISAAC_SIM_VERSION=$(ISAAC_SIM_VERSION) \
		--build-arg ISAAC_LAB_VERSION=$(ISAAC_LAB_VERSION) \
		-t $(ISAAC_LAB_IMAGE) .

build-dev:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target dev -t $(COSTNAV_DEV_IMAGE) .

build-all: build-isaac-sim build-isaac-lab build-dev

# =============================================================================
# Third-party Fetch
# =============================================================================

fetch-third-party:
	@./scripts/fetch_third_party.sh

# =============================================================================
# ROS2 Workspace Build and Runtime Targets
# =============================================================================

# Build the ROS2 runtime Docker image
build-ros2:
	$(DOCKER_COMPOSE) --profile ros2 build ros2

# Run the ROS2 runtime container
run-ros2:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile ros2 up ros2

# =============================================================================
# Isaac Sim and Navigation Targets
# =============================================================================

# Run the Isaac Sim container with launch.py (includes RViz)
# TODO: down and up every time takes a long time. Can we avoid it?
# However, healthcheck does not work if we don't do this...
# Usage: make run-isaac-sim NUM_PEOPLE=5 SIM_ROBOT=nova_carter FOOD=True GOAL_IMAGE=True
run-isaac-sim:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile isaac-sim down
	NUM_PEOPLE=$(NUM_PEOPLE) SIM_ROBOT=$(SIM_ROBOT) FOOD=$(FOOD) GOAL_IMAGE=$(GOAL_IMAGE) $(DOCKER_COMPOSE) --profile isaac-sim up

# Run the Isaac Sim container with the native Isaac Sim GUI (no launch.py)
# Useful for opening the editor directly to inspect scenes, create assets, etc.
# Usage: make run-isaac-sim-raw
run-isaac-sim-raw:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile isaac-sim down
	$(DOCKER_COMPOSE) --profile isaac-sim run --rm \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.Xauthority \
		-e OMNI_KIT_ALLOW_ROOT=1 \
		isaac-sim /isaac-sim/isaac-sim.sh

# Run both Isaac Sim and ROS2 Nav2 navigation together (using combined 'nav2' profile)
# Usage: make run-nav2 NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True TUNED=True AMCL=False
run-nav2:
	@if ! docker image inspect $(ISAAC_SIM_IMAGE) >/dev/null 2>&1; then \
		echo "==> Missing Isaac Sim image ($(ISAAC_SIM_IMAGE)); building..."; \
		$(MAKE) build-isaac-sim; \
	fi
	@if ! docker image inspect $(COSTNAV_ROS2_IMAGE) >/dev/null 2>&1; then \
		echo "==> Missing ROS2 image ($(COSTNAV_ROS2_IMAGE)); building..."; \
		$(MAKE) build-ros2; \
	fi
	xhost +local:docker 2>/dev/null || true
	SIM_ROBOT=$(SIM_ROBOT) $(DOCKER_COMPOSE) --profile nav2 down
	NUM_PEOPLE=$(NUM_PEOPLE) SIM_ROBOT=$(SIM_ROBOT) FOOD=$(FOOD) TUNED=$(TUNED) AMCL=$(AMCL) $(DOCKER_COMPOSE) --profile nav2 up

# Trigger mission start (manual)
start-mission:
	@container=""; \
	names=$$(docker ps --format '{{.Names}}'); \
	for svc in costnav-ros2-nav2 costnav-ros2-teleop costnav-ros2-vint costnav-ros2; do \
		container=$$(echo "$$names" | grep -m1 -E "^$${svc}($$|-run-)" || true); \
		[ -n "$$container" ] && break; \
	done; \
	if [ -z "$$container" ]; then \
		echo "No ROS2 container running (expected costnav-ros2-nav2, costnav-ros2-teleop, costnav-ros2-vint, costnav-ros2)."; \
		exit 1; \
	fi; \
	echo "Calling /start_mission via $$container"; \
	docker exec "$$container" /ros_entrypoint.sh ros2 service call /start_mission std_srvs/srv/Trigger {}

# Start ROS bag recording then trigger a mission
start-mission-record:
	$(MAKE) run-rosbag
	$(MAKE) start-mission
	@echo ""
	@echo "Run 'make stop-rosbag' after recording is done."


# Run Isaac Sim + RViz, then launch teleop node interactively (curses UI visible)
# Usage: make run-teleop NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True GOAL_IMAGE=True
run-teleop:
	@if [ "$(SIM_ROBOT)" != "nova_carter" ] && [ "$(SIM_ROBOT)" != "segway_e1" ]; then \
		echo "Unsupported robot: $(SIM_ROBOT). Use nova_carter or segway_e1."; \
		exit 1; \
	fi
	xhost +local:docker 2>/dev/null || true
	@# Bring down any previous teleop session
	SIM_ROBOT=$(SIM_ROBOT) $(DOCKER_COMPOSE) --profile teleop down
	@# Start Isaac Sim and RViz in background
	@echo "Waiting for Isaac Sim to become healthy (tip: run 'docker logs -f costnav-isaac-sim' in another terminal to monitor)..."
	NUM_PEOPLE=$(NUM_PEOPLE) SIM_ROBOT=$(SIM_ROBOT) FOOD=$(FOOD) GOAL_IMAGE=$(GOAL_IMAGE) \
		$(DOCKER_COMPOSE) --profile teleop up -d
	@# Wait for Isaac Sim to become healthy
	@timeout 600 bash -c 'while [ "$$(docker inspect -f "{{.State.Health.Status}}" costnav-isaac-sim 2>/dev/null)" != "healthy" ]; do sleep 5; done' \
		|| { echo "ERROR: Isaac Sim did not become healthy within 10 minutes."; exit 1; }
	@echo "Isaac Sim is healthy. Starting teleop node..."
	@# Resize terminal to minimum size for curses UI (58x35)
	@cols=$$(tput cols 2>/dev/null || echo 0); rows=$$(tput lines 2>/dev/null || echo 0); \
	if [ "$$cols" -lt 58 ] || [ "$$rows" -lt 35 ]; then \
		printf '\033[8;35;58t'; \
		sleep 0.3; \
	fi
	@# Run teleop interactively so the curses UI is visible
	@# On Ctrl+C: stop teleop, then tear down the whole profile.
	@# Ignore further SIGINTs during teardown so a second Ctrl+C doesn't interrupt cleanup.
	@SIM_ROBOT=$(SIM_ROBOT) XBOX_ID=$(XBOX_ID) \
		$(DOCKER_COMPOSE) --profile teleop run --rm ros2-teleop; \
	echo ""; \
	echo "Teleop stopped. Tearing down containers (please wait, do not press Ctrl+C again)..."; \
	trap '' INT; \
	SIM_ROBOT=$(SIM_ROBOT) $(DOCKER_COMPOSE) --profile teleop down; \
	echo "Done."

# =============================================================================
# IL Baselines (ViNT) Targets
# =============================================================================

# Build the ViNT Docker image
build-vint:
	$(DOCKER_COMPOSE) --profile vint build ros2-vint

# Run Isaac Sim with ViNT policy node and trajectory follower for IL baseline evaluation
# Set MODEL_CHECKPOINT environment variable to specify model weights (default: checkpoints/vint.pth)
# Goal image publishing is enabled by default for ViNT ImageGoal mode
# Example: MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint
run-vint:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile vint down
	GOAL_IMAGE=True MODEL_CHECKPOINT=$(MODEL_CHECKPOINT) $(DOCKER_COMPOSE) --profile vint up

# =============================================================================
# ROS Bag Recording Targets
# =============================================================================

# Start ROS bag recording in the background (run before run-teleop)
# Records all ROS messages to ./rosbags/ directory
# Stop with: make stop-rosbag
run-rosbag:
	$(DOCKER_COMPOSE) --profile rosbag up -d ros2-bag-recorder
	@echo "ROS bag recording started. Bags will be saved to ./rosbags/"
	@echo "Run 'make stop-rosbag' to stop recording and save the bag file."

# Stop ROS bag recording gracefully (SIGINT triggers bag file save)
stop-rosbag:
	$(DOCKER_COMPOSE) --profile rosbag down
	@echo "ROS bag recording stopped. Check ./rosbags/ for recorded bag files."

# =============================================================================
# Nav2 Evaluation Targets
# =============================================================================

# Default evaluation parameters
TIMEOUT ?= 169 # based on S_EvalTimeout
NUM_MISSIONS ?= 3

# Run Nav2 evaluation (requires running nav2 instance via make run-nav2)
# Usage: make run-eval-nav2 TIMEOUT=20 NUM_MISSIONS=10
# Output: ./logs/nav2_evaluation_<timestamp>.log
run-eval-nav2:
	@if ! docker ps --format '{{.Names}}' | grep -qx "costnav-ros2-nav2"; then \
		echo "ERROR: 'make run-nav2' is not running."; \
		echo ""; \
		echo "Please start nav2 first in a separate terminal:"; \
		echo "  make run-nav2"; \
		echo ""; \
		echo "Then run this command again:"; \
		echo "  make run-eval-nav2 TIMEOUT=$(TIMEOUT) NUM_MISSIONS=$(NUM_MISSIONS)"; \
		exit 1; \
	fi
	@echo "Starting Nav2 evaluation..."
	@echo "  Timeout per mission: $(TIMEOUT)s"
	@echo "  Number of missions:  $(NUM_MISSIONS)"
	@echo ""
	@bash scripts/eval.sh nav2 $(TIMEOUT) $(NUM_MISSIONS)

# Run Teleop evaluation (requires running teleop instance via make run-teleop)
# Usage: make run-eval-teleop TIMEOUT=20 NUM_MISSIONS=10
# Output: ./logs/teleop_evaluation_<timestamp>.log
run-eval-teleop:
	@if ! docker ps --format '{{.Names}}' | grep -qE "^costnav-ros2-teleop($$|-run-)"; then \
		echo "ERROR: 'make run-teleop' is not running."; \
		echo ""; \
		echo "Please start teleop first in a separate terminal:"; \
		echo "  make run-teleop"; \
		echo ""; \
		echo "Then run this command again:"; \
		echo "  make run-eval-teleop TIMEOUT=$(TIMEOUT) NUM_MISSIONS=$(NUM_MISSIONS)"; \
		exit 1; \
	fi
	@echo "Starting Teleop evaluation..."
	@echo "  Timeout per mission: $(TIMEOUT)s"
	@echo "  Number of missions:  $(NUM_MISSIONS)"
	@echo ""
	@bash scripts/eval.sh teleop $(TIMEOUT) $(NUM_MISSIONS)

# Run ViNT evaluation (requires running vint instance via make run-vint)
# Usage: make run-eval-vint TIMEOUT=20 NUM_MISSIONS=10
# Output: ./logs/vint_evaluation_<timestamp>.log
run-eval-vint:
	@if ! docker ps --format '{{.Names}}' | grep -qx "costnav-ros2-vint"; then \
		echo "ERROR: 'make run-vint' is not running."; \
		echo ""; \
		echo "Please start vint first in a separate terminal:"; \
		echo "  MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint"; \
		echo ""; \
		echo "Then run this command again:"; \
		echo "  make run-eval-vint TIMEOUT=$(TIMEOUT) NUM_MISSIONS=$(NUM_MISSIONS)"; \
		exit 1; \
	fi
	@echo "Starting ViNT evaluation..."
	@echo "  Timeout per mission: $(TIMEOUT)s"
	@echo "  Number of missions:  $(NUM_MISSIONS)"
	@echo ""
	@bash scripts/eval.sh vint $(TIMEOUT) $(NUM_MISSIONS)


# =============================================================================
# Asset Download Targets
# =============================================================================

# Download Omniverse assets to local ./assets/ directory
# Runs inside Isaac Sim Docker container
download-assets-omniverse:
	@echo "Downloading Omniverse assets using Isaac Sim environment..."
	$(DOCKER_COMPOSE) --profile isaac-sim run --rm isaac-sim \
		/isaac-sim/python.sh /workspace/scripts/assets/download_assets_omniverse.py

# Download assets from Hugging Face dataset
# Runs inside dev Docker container with huggingface_hub
download-assets-hf:
	@echo "Downloading assets from Hugging Face..."
	$(DOCKER_COMPOSE) --profile dev run --rm dev \
		bash -c "uv pip install --system --break-system-packages huggingface_hub && python3 /workspace/scripts/assets/download_assets_hf.py"

# Upload assets to Hugging Face dataset
# Runs inside dev Docker container with huggingface_hub
# Requires HF_TOKEN to be set in .env file
upload-assets-hf:
	@echo "Uploading assets to Hugging Face..."
	$(DOCKER_COMPOSE) --profile dev run --rm dev \
		bash -c "uv pip install --system --break-system-packages huggingface_hub && python3 /workspace/scripts/assets/upload_assets_hf.py"

# =============================================================================
# Nucleus Server Targets
# =============================================================================

# Nucleus server configuration
NUCLEUS_STACK_DIR ?= .nucleus-stack
NUCLEUS_STACK_VERSION ?= 2023.2.9
NGC_CLI_VERSION ?= 3.41.4
OMNI_USER ?= omniverse
OMNI_PASS ?= costnav123

# Start local Omniverse Nucleus server in Docker
# Automatically downloads Nucleus stack if not present
# Usage: make start-nucleus
start-nucleus:
	@echo "Starting Omniverse Nucleus server..."
	@if [ ! -d "assets/Users" ]; then \
		echo "ERROR: Assets not found at assets/Users"; \
		echo ""; \
		echo "Please download assets first:"; \
		echo "  make download-assets-hf"; \
		exit 1; \
	fi
	@if [ ! -f "$(NUCLEUS_STACK_DIR)/base_stack/nucleus-stack-no-ssl.yml" ]; then \
		echo "Nucleus stack not found. Downloading via Docker..."; \
		mkdir -p $(NUCLEUS_STACK_DIR); \
		docker run --rm \
			-v $(PWD)/$(NUCLEUS_STACK_DIR):/output \
			-v $(PWD)/.env:/workspace/.env:ro \
			ubuntu:22.04 bash -c '\
				set -e && \
				apt-get update && apt-get install -y wget unzip && \
				cd /tmp && \
				wget -q "https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/$(NGC_CLI_VERSION)/files/ngccli_linux.zip" && \
				unzip -q ngccli_linux.zip && \
				chmod +x ngc-cli/ngc && \
				. /workspace/.env && \
				mkdir -p ~/.ngc && \
				echo -e "[CURRENT]\napikey = $${NGC_PASS}\nformat_type = ascii\norg = nvidia" > ~/.ngc/config && \
				./ngc-cli/ngc registry resource download-version "nvidia/omniverse/nucleus-compose-stack:$(NUCLEUS_STACK_VERSION)" --dest /tmp && \
				tar xzf /tmp/nucleus-compose-stack_v$(NUCLEUS_STACK_VERSION)/*.tar.gz -C /output --strip-components=1 && \
				chown -R $(shell id -u):$(shell id -g) /output && \
				echo "Nucleus stack downloaded successfully"'; \
	fi
	@echo "Configuring Nucleus stack..."
	@cd $(NUCLEUS_STACK_DIR)/base_stack && \
		if [ ! -f nucleus-stack.env.configured ]; then \
			cp nucleus-stack.env nucleus-stack.env.backup 2>/dev/null || true; \
			sed -i 's/^#*ACCEPT_EULA=.*/ACCEPT_EULA=1/' nucleus-stack.env; \
			sed -i 's/^#*SECURITY_REVIEWED=.*/SECURITY_REVIEWED=1/' nucleus-stack.env; \
			sed -i 's/^#*SERVER_IP_OR_HOST=.*/SERVER_IP_OR_HOST=localhost/' nucleus-stack.env; \
			sed -i 's|^#*DATA_ROOT=.*|DATA_ROOT=$(PWD)/$(NUCLEUS_STACK_DIR)/data|' nucleus-stack.env; \
			sed -i 's/^#*MASTER_PASSWORD=.*/MASTER_PASSWORD=$(OMNI_PASS)/' nucleus-stack.env; \
			sed -i 's/^#*SERVICE_PASSWORD=.*/SERVICE_PASSWORD=$(OMNI_PASS)/' nucleus-stack.env; \
			if [ -f generate-sample-insecure-secrets.sh ]; then ./generate-sample-insecure-secrets.sh; fi; \
			touch nucleus-stack.env.configured; \
		fi
	@echo "Logging into NGC registry..."
	@bash -c 'source .env && echo "$${NGC_PASS}" | docker login nvcr.io -u "$${NGC_USER:-\$$oauthtoken}" --password-stdin'
	@echo "Starting Nucleus containers..."
	cd $(NUCLEUS_STACK_DIR)/base_stack && \
		docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml up -d
	@echo ""
	@echo "Uploading assets to Nucleus using Isaac Sim container..."
	@if docker image inspect $(ISAAC_SIM_IMAGE) >/dev/null 2>&1; then \
		docker run --rm \
			--network host \
			--entrypoint /bin/bash \
			-v $(PWD)/assets:/workspace/assets:ro \
			-v $(PWD)/scripts:/workspace/scripts:ro \
			-e "OMNI_USER=$(OMNI_USER)" \
			-e "OMNI_PASS=$(OMNI_PASS)" \
			$(ISAAC_SIM_IMAGE) \
			-c "PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib:\$$PYTHONPATH /isaac-sim/python.sh /workspace/scripts/assets/upload_assets_to_nucleus.py \
				--local-path /workspace/assets \
				--nucleus-url omniverse://localhost \
				--timeout 120"; \
	else \
		echo ""; \
		echo "WARNING: Isaac Sim image not found: $(ISAAC_SIM_IMAGE)"; \
		echo "Run 'make build-isaac-sim' first to enable automatic asset upload."; \
		echo ""; \
		echo "For now, please upload assets manually:"; \
		echo "  1. Open http://localhost:8080"; \
		echo "  2. Login with: $(OMNI_USER) / $(OMNI_PASS)"; \
		echo "  3. Navigate to / and upload files from assets/"; \
	fi
	@echo ""
	@echo "============================================================"
	@echo "Nucleus server is ready!"
	@echo ""
	@echo "Web UI:     http://localhost:8080"
	@echo "Omniverse:  omniverse://localhost"
	@echo ""
	@echo "Default credentials:"
	@echo "  Username: $(OMNI_USER)"
	@echo "  Password: $(OMNI_PASS)"
	@echo ""
	@echo "Main Assets available at:"
	@echo "  omniverse://localhost/Users/worv/costnav/..."
	@echo ""
	@echo "To stop:    make stop-nucleus"
	@echo "============================================================"

# Stop local Omniverse Nucleus server
stop-nucleus:
	@echo "Stopping Omniverse Nucleus server..."
	@if [ -f "$(NUCLEUS_STACK_DIR)/base_stack/nucleus-stack-no-ssl.yml" ]; then \
		cd $(NUCLEUS_STACK_DIR)/base_stack && \
		docker compose --env-file nucleus-stack.env -f nucleus-stack-no-ssl.yml down; \
		echo "Nucleus server stopped."; \
	else \
		echo "Nucleus stack not found at $(NUCLEUS_STACK_DIR)"; \
	fi
