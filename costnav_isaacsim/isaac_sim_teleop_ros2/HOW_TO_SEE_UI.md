# How to See the Terminal UI

The teleop node has a nice curses-based terminal UI that shows real-time velocity, control status, and progress bars. Here's how to see it:

## Method 1: Launch File (Easiest - Opens in New Window)

The launch file is configured to open the teleop node in a separate xterm window:

```bash
# Make sure xterm is installed
sudo apt install xterm

# Launch - teleop UI will open in new window
export SIM_ROBOT=nova_carter
source install/setup.bash
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py
```

The UI will appear in a new xterm window automatically!

## Method 2: Run Directly in Terminal (Full Control)

**Terminal 1** - Joy node:
```bash
source install/setup.bash
ros2 run joy joy_node
```

**Terminal 2** - Teleop node with UI (resize to at least 50x15):
```bash
export SIM_ROBOT=nova_carter
source install/setup.bash
ros2 run isaac_sim_teleop_ros2 isaac_sim_teleop_node
```

The UI will appear in Terminal 2.

## Method 3: Single Terminal (Quick Test)

```bash
export SIM_ROBOT=nova_carter
source install/setup.bash

# Run joy in background
ros2 run joy joy_node &

# Run teleop with UI in foreground
ros2 run isaac_sim_teleop_ros2 isaac_sim_teleop_node
```

## Disable the Separate Terminal Window

If you don't want the separate xterm window, edit `launch/teleop_isaac_sim.launch.py` and change:

```python
prefix='xterm -e',  # Opens in new terminal window for UI
```

to:

```python
prefix=None,  # Run in same terminal (no UI)
```

## Terminal Requirements

- **Minimum size**: 50 characters wide × 15 lines tall
- **Check your size**: `echo "Terminal: $(tput cols)x$(tput lines)"`
- **Resize if needed**: Make your terminal window bigger

## What You'll See

The UI displays:
- Robot information and type
- Angular velocity (rate % and actual rad/s)
- Linear velocity (rate % and actual km/h)
- Max velocity setting
- Model command velocity
- Control status (emergency stop, linear lock, model switch)
- Real-time progress bars for all velocities
- FPS and current time (if clock enabled)

## Troubleshooting

**"Could not initialize terminal UI"**
- Terminal is too small - resize to at least 50x15
- Not running in interactive terminal - use Method 1 or 2

**"xterm: command not found"**
```bash
sudo apt install xterm
```

**Want a different terminal?**
Edit the launch file and change `xterm -e` to:
- `gnome-terminal --` for GNOME Terminal
- `konsole -e` for KDE Konsole
- `xfce4-terminal -e` for XFCE Terminal

