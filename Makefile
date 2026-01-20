.PHONY: build-isaac-sim build-isaac-lab build-dev build-all build-ros-ws build-ros2 run-ros2 run-isaac-sim run-nav2 run-teleop start-mission start-mission-record run-rosbag stop-rosbag

DOCKERFILE ?= Dockerfile
DOCKER_BUILD ?= docker build
DOCKER_COMPOSE ?= docker compose

ISAAC_SIM_VERSION ?= 5.1.0
ISAAC_LAB_VERSION ?= 2.3.0
COSTNAV_VERSION ?= 0.1.0

# ROS configuration
ROS_DISTRO ?= jazzy
UBUNTU_VERSION ?= 24.04

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
run-isaac-sim:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile isaac-sim down
	$(DOCKER_COMPOSE) --profile isaac-sim up

# Run both Isaac Sim and ROS2 Nav2 navigation together (using combined 'nav2' profile)
# Usage: make run-nav2 NUM_PEOPLE=5
run-nav2:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile nav2 down
	NUM_PEOPLE=$(NUM_PEOPLE) $(DOCKER_COMPOSE) --profile nav2 up

# Trigger mission start (manual)
start-mission:
	@container=""; \
	if docker ps --format '{{.Names}}' | grep -qx "costnav-ros2-nav2"; then \
		container="costnav-ros2-nav2"; \
	elif docker ps --format '{{.Names}}' | grep -qx "costnav-ros2-teleop"; then \
		container="costnav-ros2-teleop"; \
	elif docker ps --format '{{.Names}}' | grep -qx "costnav-ros2"; then \
		container="costnav-ros2"; \
	elif docker ps --format '{{.Names}}' | grep -qx "costnav-isaac-sim"; then \
		container="costnav-isaac-sim"; \
	fi; \
	if [ -z "$$container" ]; then \
		echo "No ROS2 container running (expected costnav-ros2-nav2, costnav-ros2-teleop, costnav-ros2, or costnav-isaac-sim)."; \
		exit 1; \
	fi; \
	echo "Calling /start_mission via $$container"; \
	docker exec "$$container" bash -c " \
		if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi; \
		if [ -f /workspace/build_ws/install/local_setup.sh ]; then source /workspace/build_ws/install/local_setup.sh; fi; \
		if [ -f /isaac-sim/setup_ros_env.sh ]; then source /isaac-sim/setup_ros_env.sh; fi; \
		ros2 service call /start_mission std_srvs/srv/Trigger {}"

# Start ROS bag recording then trigger a mission
start-mission-record:
	$(MAKE) run-rosbag
	$(MAKE) start-mission

# Run both Isaac Sim and ROS2 teleop together (using combined 'teleop' profile)
# Usage: make run-teleop NUM_PEOPLE=5
run-teleop:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile teleop down
	NUM_PEOPLE=$(NUM_PEOPLE) $(DOCKER_COMPOSE) --profile teleop up

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
