# CostNav Isaac Sim

This directory contains Isaac Sim integration for the CostNav project, including scripts for loading USD environments and Nav2 navigation parameters.

## Current Status

### ✅ Operational
- **Nav2 Integration**: Fully functional with Nova Carter robot
- **Physics Simulation**: Complete physics support
- **Multi-robot Support**: Nova Carter (primary), Franka, UR10

### ⏳ In Progress
- **Parameter Tuning**: Optimizing Nova Carter navigation performance
- **Start/Goal Sampling**: Mission planning system development

### 📋 Future Work
- **COCO Robot Integration**: Adapt Nav2 for COCO delivery robot
- **Cost Model Integration**: Track economic metrics for Nav2 navigation

## Quick Start

### Launch Isaac Sim with Street Sidewalk Environment

```bash
# Simple load (view only)
python launch.py

# With physics simulation
python launch.py --simulate

# With Nova Carter robot for navigation (recommended for Nav2)
python launch.py --simulate --robot carter
```

## Files

### `launch.py`
Main entry point for loading USD environments in Isaac Sim. Supports:
- Simple USD file viewing
- Physics simulation
- Robot integration (Nova Carter, Franka, UR10)
- Headless mode for server deployments

**Documentation**: See [/workspace/docs/nav2/isaac_sim_launch.md](../docs/nav2/isaac_sim_launch.md) for detailed usage and API reference.

### `nav2_params/`
Navigation parameters for Nav2 stack with Nova Carter:
- `carter_navigation_params.yaml` - Nav2 configuration for Nova Carter robot
- `carter_sidewalk.yaml` - Map configuration for sidewalk environment
- `carter_sidewalk.png` - Map image file

**Note**: These configuration files should be mounted to the ROS2 Docker container for Nav2 navigation.

## Usage Examples

### Basic Environment Loading

```bash
# Load default Street_sidewalk.usd
python launch.py --simulate
```

### Load Custom USD File

```bash
# From local path
python launch.py --usd_path /path/to/your/scene.usd --simulate

# From Omniverse server
python launch.py --usd_path omniverse://server/path/scene.usd --simulate
```

### Navigation with Nova Carter Robot and Nav2

**Step 1: Launch Isaac Sim with Nova Carter**
```bash
cd /workspace/costnav_isaacsim
python launch.py --simulate --robot carter
```

**Step 2: In ROS2 Docker Container, Launch Nav2**
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /workspace/build_ws/install/local_setup.sh

# Launch Nav2 with Nova Carter
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.yaml \
    params_file:/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml
```

**Docker Volume Mounting:**

Ensure your `docker-compose.yml` includes:
```yaml
volumes:
  - /workspace/costnav_isaacsim/nav2_params:/workspace/costnav_isaacsim/nav2_params:ro
```

### Headless Mode (for servers)

```bash
python launch.py --simulate --robot carter --headless
```

## Default Environment

The default USD environment is:
```
omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd
```

This represents a street sidewalk scene designed for navigation testing.

## Documentation

For detailed documentation, see:
- **Launch Script**: [/workspace/docs/nav2/isaac_sim_launch.md](../docs/nav2/isaac_sim_launch.md) - Detailed usage guide
- **Nav2 Implementation**: [/workspace/docs/nav2/nav2_implementation_plan.md](../docs/nav2/nav2_implementation_plan.md) - Complete Nav2 integration plan
- **Physics Fixes**: [/workspace/docs/nav2/physics_fix.md](../docs/nav2/physics_fix.md) - Physics properties documentation

## Requirements

- Isaac Sim (properly installed and configured)
- Access to Omniverse server at `10.50.2.21` (for default environment)
- Python 3.x with Isaac Sim Python environment

## Reference

Based on Isaac Sim official documentation:
- [Robot Simulation Snippets](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/robots_simulation.html)
- [Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/python_scripting_concepts.html)

