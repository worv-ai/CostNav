.PHONY: build-isaac-sim build-isaac-lab build-dev build-all build-ros-ws build-ros2 run-ros2

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
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-sim -t $(ISAAC_SIM_IMAGE) .

build-isaac-lab:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-lab -t $(ISAAC_LAB_IMAGE) .

build-dev:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target dev -t $(COSTNAV_DEV_IMAGE) .

build-all: build-isaac-sim build-isaac-lab build-dev

# =============================================================================
# ROS2 Workspace Build and Runtime Targets
# =============================================================================

# Build the Isaac Sim ROS workspace using build_ros.sh
build-ros-ws:
	cd third_party/IsaacSim-ros_workspaces && ./build_ros.sh -d $(ROS_DISTRO) -v $(UBUNTU_VERSION)

# Build the ROS2 runtime Docker image
build-ros2:
	$(DOCKER_COMPOSE) --profile ros2 build ros2

# Run the ROS2 runtime container
run-ros2:
	xhost +local:docker 2>/dev/null || true
	$(DOCKER_COMPOSE) --profile ros2 up ros2