.PHONY: build-isaac-sim build-isaac-lab build-dev build-all build-ros-ws build-ros2 build-vint run-ros2 run-isaac-sim run-nav2 run-teleop run-vint start-mission start-mission-record run-rosbag stop-rosbag run-eval-nav2 run-eval-teleop run-eval-vint

DOCKERFILE ?= Dockerfile
DOCKER_BUILD ?= docker build
DOCKER_COMPOSE ?= docker compose

ISAAC_SIM_VERSION ?= 5.1.0
ISAAC_LAB_VERSION ?= 2.3.0
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
# ROS2 Workspace Build and Runtime Targets
# =============================================================================

# Build the Isaac Sim ROS workspace using build_ros.sh
build-ros-ws:
	@echo "==> Cleaning previous build_ws/$(ROS_DISTRO)..."
	cd third_party/IsaacSim-ros_workspaces && \
		docker run --rm -v $$(pwd)/build_ws:/build_ws ubuntu:22.04 rm -rf /build_ws/$(ROS_DISTRO)
	@echo "==> Building ROS workspace for $(ROS_DISTRO) on Ubuntu $(UBUNTU_VERSION)..."
	cd third_party/IsaacSim-ros_workspaces && ./build_ros.sh -d $(ROS_DISTRO) -v $(UBUNTU_VERSION)
	@echo "==> Build complete!"

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

# Run both Isaac Sim and ROS2 Nav2 navigation together (using combined 'nav2' profile)
# Usage: make run-nav2 NUM_PEOPLE=5 SIM_ROBOT=nova_carter FOOD=1 TUNED=True AMCL=False
run-nav2:
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

ifeq (run-teleop,$(firstword $(MAKECMDGOALS)))
ifneq ($(word 2,$(MAKECMDGOALS)),)
SIM_ROBOT := $(word 2,$(MAKECMDGOALS))
$(eval $(word 2,$(MAKECMDGOALS)):;@:)
endif
SIM_ROBOT := $(subst -,_,$(SIM_ROBOT))
ifeq ($(SIM_ROBOT),segwaye1)
SIM_ROBOT := segway_e1
endif
ifeq ($(SIM_ROBOT),segway)
SIM_ROBOT := segway_e1
endif
endif

# Run Isaac Sim + RViz, then launch teleop node interactively (curses UI visible)
# Usage: make run-teleop NUM_PEOPLE=5 FOOD=1 GOAL_IMAGE=True
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
	@# When teleop exits (Ctrl+C or normal end), bring down the teleop compose profile
	SIM_ROBOT=$(SIM_ROBOT) XBOX_ID=$(XBOX_ID) \
		$(DOCKER_COMPOSE) --profile teleop run --rm ros2-teleop; \
	SIM_ROBOT=$(SIM_ROBOT) $(DOCKER_COMPOSE) --profile teleop down

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
