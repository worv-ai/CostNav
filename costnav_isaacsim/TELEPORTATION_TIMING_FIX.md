# Teleportation Timing Fix

## Problem

The original implementation had a timing issue when teleporting the robot:

1. **Background Thread Architecture**: `MissionRunner` ran in a separate thread from the main simulation loop
2. **Asynchronous Teleportation**: Teleportation happened in the background thread using `XFormPrim.set_world_pose()`
3. **Missing Physics Steps**: After setting the pose, the physics engine wasn't stepped to process the new state
4. **Race Condition**: The main simulation loop and mission thread were not synchronized

This caused issues where:
- Robot physics state wasn't properly updated after teleportation
- Timing between teleportation and initial pose publication was unreliable
- Navigation could start before physics had settled

## Solution

Following the pattern from `third_party/isaac_sim/src/isaac_sim_launch.py`, we implemented:

### 1. Main Loop Integration (MissionManager)

Instead of running missions in a background thread, we integrated mission management into the main simulation loop:

```python
# In launch.py _step_simulation():
def _step_simulation(self, mission_manager=None, throttle: bool = False):
    self.simulation_context.step(render=True)
    
    # Step mission manager after simulation step
    if mission_manager is not None:
        mission_manager.step()
```

This matches the Isaac Sim pattern:
```python
# From third_party/isaac_sim/src/isaac_sim_launch.py:
def _step_simulation(self, ros_manager=None, throttle: bool = False):
    self.simulation_context.step(render=True)
    if ros_manager is not None:
        ros_manager.step()
```

### 2. Physics Settling After Teleportation

After teleportation, we now step the simulation multiple times to let physics settle:

```python
# In mission_orchestrator.py:
if self._simulation_context is not None:
    for _ in range(self.config.teleport_settle_steps):
        self._simulation_context.step(render=True)
```

This is based on the pattern in `third_party/isaac_sim/src/comm/common.py`:
```python
# BaseTeleportation.step():
for attribute_list, value_list in self.attribute_data:
    for value in value_list:
        for attribute in attribute_list:
            attribute.Set(value)
        self.simulation_context.step()  # Step after each attribute change
```

### 3. State Machine for Mission Execution

`MissionManager` uses a state machine to ensure proper sequencing:

1. `INIT` → Initialize ROS2 and orchestrator
2. `WAITING_FOR_NAV2` → Wait for Nav2 stack
3. `READY` → Sample positions and start mission
4. `TELEPORTING` → Teleport robot
5. `SETTLING` → **Step physics N times to settle** (NEW!)
6. `PUBLISHING_INITIAL_POSE` → Publish initial pose for AMCL
7. `PUBLISHING_GOAL` → Publish goal to Nav2
8. `WAITING_FOR_COMPLETION` → Wait for mission delay
9. Back to `READY` for next mission

## Key Changes

### Files Modified

1. **`costnav_isaacsim/launch.py`**:
   - Modified `_step_simulation()` to accept `mission_manager` parameter
   - Changed from `_start_mission_runner()` to `_setup_mission_manager()`
   - Integrated mission manager into main loop

2. **`costnav_isaacsim/nav2_mission/mission_orchestrator.py`**:
   - Added `simulation_context` parameter to `__init__()`
   - Added `teleport_settle_steps` to `OrchestratorConfig`
   - Modified `run_mission()` to step physics after teleportation

3. **`costnav_isaacsim/nav2_mission/mission_manager.py`** (NEW):
   - State machine-based mission execution
   - Runs in main simulation loop via `step()` calls
   - Ensures proper synchronization with physics

4. **`costnav_isaacsim/nav2_mission/__init__.py`**:
   - Added `MissionManager` to exports
   - Updated documentation

### Configuration

New config parameter in `OrchestratorConfig`:
```python
teleport_settle_steps: int = 5  # Number of simulation steps after teleportation
```

## Benefits

1. **Proper Physics Synchronization**: Teleportation is now synchronized with simulation steps
2. **Deterministic Timing**: No race conditions between threads
3. **Follows Isaac Sim Patterns**: Matches the established architecture in `third_party/isaac_sim/`
4. **Configurable Settling**: Can adjust `teleport_settle_steps` based on robot complexity

## Migration

### Old (Background Thread - Has Timing Issues):
```python
from nav2_mission import MissionRunner

runner = MissionRunner(config)
runner.start()

while simulation_app.is_running():
    simulation_context.step(render=True)

runner.stop()
```

### New (Main Loop Integration - Recommended):
```python
from nav2_mission import MissionManager

manager = MissionManager(config, simulation_context)

while simulation_app.is_running():
    simulation_context.step(render=True)
    manager.step()  # Step after physics
```

## References

- `third_party/isaac_sim/src/isaac_sim_launch.py` - Main loop pattern with `ros_manager.step()`
- `third_party/isaac_sim/src/comm/common.py` - `BaseTeleportation.step()` with `simulation_context.step()`
- `third_party/isaac_sim/src/runtime/manager.py` - `RuntimeManager.step()` integration

