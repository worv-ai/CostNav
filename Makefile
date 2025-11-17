.PHONY: build-isaac-sim build-isaac-lab build-dev build-all

DOCKERFILE ?= Dockerfile
DOCKER_BUILD ?= docker build

ISAAC_SIM_VERSION ?= 5.1.0
ISAAC_LAB_VERSION ?= 2.3.0
COSTNAV_VERSION ?= 0.1.0

ISAAC_SIM_IMAGE ?= costnav-isaacsim-$(ISAAC_SIM_VERSION):$(COSTNAV_VERSION)
ISAAC_LAB_IMAGE ?= costnav-isaaclab-$(ISAAC_SIM_VERSION)-$(ISAAC_LAB_VERSION):$(COSTNAV_VERSION)
COSTNAV_DEV_IMAGE ?= costnav-dev:$(COSTNAV_VERSION)

build-isaac-sim:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-sim -t $(ISAAC_SIM_IMAGE) .

build-isaac-lab:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target isaac-lab -t $(ISAAC_LAB_IMAGE) .

build-dev:
	$(DOCKER_BUILD) -f $(DOCKERFILE) --target dev -t $(COSTNAV_DEV_IMAGE) .

build-all: build-isaac-sim build-isaac-lab build-dev
