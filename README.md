# agribotvjunk

## GUI standalone mode (no ROS topics required)

The `agribotv3` dashboard can now start in **standalone mode** even when ROS 2 nodes/topics are not available yet.

### What changed
- GUI startup no longer hard-fails if ROS Python packages are missing.
- The GUI can launch and stay responsive while waiting for ROS/Nav2 and topic publishers.
- Navigation commands in standalone mode are accepted by the UI and shown as waiting states instead of crashing.

### Run
```bash
python3 -m agribotv3.gui.app
```

When ROS 2 + publishers become available, the controller thread can consume topic data normally.
