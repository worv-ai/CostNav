# Isaac Sim Launch Script Documentation

## Overview

The `launch.py` script in `/workspace/costnav_isaacsim/` is the primary entry point for loading and simulating the Street_sidewalk.usd environment in Isaac Sim. This script provides a flexible interface for opening USD files with optional physics simulation and robot integration.

## Current Status

### ✅ Operational
- **Nav2 Integration**: Fully functional with Nova Carter robot
- **Physics Simulation**: Complete physics support
- **Multi-robot Support**: Nova Carter (primary), Franka, UR10

### ⏳ In Progress
- **Parameter Tuning**: Optimizing Nova Carter navigation performance
- **Start/Goal Sampling**: Mission planning system development

## Location

```
/workspace/costnav_isaacsim/launch.py
```

## Features

- **Simple USD Loading**: Load and view USD files without physics simulation
- **Physics Simulation**: Enable full physics simulation with the `--simulate` flag
- **Robot Integration**: Add robots (Nova Carter, Franka, or UR10) to the scene
- **Headless Mode**: Run without GUI for server deployments or automated testing
- **Configurable USD Path**: Load any USD file from local or Omniverse paths

## Usage

### Basic Usage

```bash
# Navigate to the directory
cd /workspace/costnav_isaacsim

# Simple load (view only, no physics)
python launch.py

# With physics simulation
python launch.py --simulate

# With Nova Carter robot (recommended for Nav2)
python launch.py --simulate --robot carter

# With Franka robot
python launch.py --simulate --robot franka

# With UR10 robot
python launch.py --simulate --robot ur10
```

### Advanced Usage

```bash
# Load a different USD file
python launch.py --usd_path /path/to/your/scene.usd --simulate

# Load from Omniverse server
python launch.py --usd_path omniverse://server/path/to/scene.usd --simulate

# Run in headless mode (no GUI)
python launch.py --simulate --headless

# Combine options
python launch.py --simulate --robot carter --headless
```

### Command-Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--usd_path` | string | `omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd` | Path to USD file (local or omniverse://) |
| `--simulate` | flag | False | Enable physics simulation |
| `--robot` | choice | None | Add robot to scene (carter, franka, ur10) |
| `--headless` | flag | False | Run without GUI |

## Implementation Details

### Architecture

The script follows the Isaac Sim Python scripting best practices:

1. **Argument Parsing**: Parse command-line arguments before creating SimulationApp
2. **SimulationApp Creation**: Initialize Isaac Sim application
3. **Conditional Loading**: 
   - **Simple Mode**: Direct USD stage loading via `omni.usd.get_context().open_stage()`
   - **Simulation Mode**: World creation with physics initialization
4. **Scene Setup**: Add ground plane, load environment, optionally add robot
5. **Simulation Loop**: Continuous update/step loop until interrupted

### Code Structure

```python
# 1. Parse arguments
parser = argparse.ArgumentParser(...)
args = parser.parse_args()

# 2. Create SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# 3. Import Isaac Sim modules (after SimulationApp)
from isaacsim.core.api.world import World
...

# 4. Main function with conditional logic
if args.simulate:
    # World-based simulation
    world = World()
    world.initialize_physics()
    # Load USD as reference
    add_reference_to_stage(...)
else:
    # Simple USD loading
    usd_context.open_stage(...)
```

## Integration with Nav2

This script is designed to work with the Nav2 navigation stack. The Nav2 integration with Nova Carter is **fully operational**.

### Quick Start with Nav2

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
    params_file:=/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml
```

### Configuration Files

Nav2 parameters are configured in `/workspace/costnav_isaacsim/nav2_params/`:
- **Map Configuration**: `carter_sidewalk.yaml`
- **Map Image**: `carter_sidewalk.png`
- **Nav2 Parameters**: `carter_navigation_params.yaml`

### Docker Volume Mounting

Ensure these volumes are mounted in your `docker-compose.yml`:
```yaml
volumes:
  - /workspace/costnav_isaacsim/nav2_params:/workspace/costnav_isaacsim/nav2_params:ro
```

See also:
- [Nav2 Implementation Plan](nav2_implementation_plan.md) - Complete implementation details
- [Physics Fix Documentation](physics_fix.md)

## Default Environment

The default USD file is located on the Omniverse server:
```
omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd
```

This environment represents a street sidewalk scene suitable for navigation testing.

## Troubleshooting

### USD File Not Found
```
✗ Not a valid USD file: <path>
```
**Solution**: Verify the USD path is correct and accessible. For Omniverse paths, ensure the server is reachable.

### Failed to Open
```
✗ Failed to open
```
**Solution**: Check that:
- Isaac Sim is properly installed
- The USD file is not corrupted
- You have read permissions for the file

### Robot Not Loading
**Solution**: Ensure the Isaac Sim assets are properly installed and the asset root path is configured correctly.

## Reference

Based on Isaac Sim documentation:
- [Robot Simulation Snippets](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/robots_simulation.html)
- [Python Scripting Concepts](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/python_scripting_concepts.html)

## Related Files

- `/workspace/costnav_isaacsim/nav2_params/` - Nav2 configuration files
- `/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml` - Carter navigation parameters

