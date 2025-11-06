# SaveMap Action Server - Testing Guide

## Overview

The SaveMap action server allows you to save the current map from RESPLE's ikd-tree to a PCD file with progress feedback and cancellation support.

## Action Interface

**Action Name**: `/save_map`

**Goal**:
- `filename` (string): Path where the map should be saved (e.g., "/tmp/my_map.pcd")

**Result**:
- `success` (bool): True if the map was saved successfully
- `message` (string): Status message or error description
- `points_saved` (uint32): Number of points saved to the file

**Feedback** (sent during execution):
- `progress` (float): Progress percentage (0.0 to 100.0)
- `status` (string): Current operation status

---

## Testing the Action Server

### 1. Launch RESPLE with a Dataset

```bash
# Source your workspace
cd ~/RESPLE
source install/setup.bash

# Launch with any dataset (let it run to build a map)
ros2 launch resple resple_r_campus.launch.py
```

### 2. Check Available Actions

```bash
# List all available actions
ros2 action list

# Should show:
# /save_map

# Get action type info
ros2 action info /save_map
```

### 3. Save the Map

#### Basic Usage:
```bash
# Save map to /tmp/map.pcd
ros2 action send_goal /save_map estimate_msgs/action/SaveMap "{filename: '/tmp/map.pcd'}"
```

#### With Feedback:
```bash
# Save map and show progress feedback
ros2 action send_goal --feedback /save_map estimate_msgs/action/SaveMap "{filename: '/tmp/my_map.pcd'}"
```

**Expected Output**:
```
Waiting for an action server to become available...
Sending goal:
     filename: /tmp/my_map.pcd

Goal accepted with ID: ...

Feedback:
    progress: 10.0
    status: Extracting points from map...

Feedback:
    progress: 50.0
    status: Writing map to file...

Feedback:
    progress: 100.0
    status: Map saved successfully

Result:
    success: True
    message: Map saved successfully to /tmp/my_map.pcd
    points_saved: 123456

Goal finished with status: SUCCEEDED
```

### 4. Test Cancellation

```bash
# In one terminal, start a save operation
ros2 action send_goal --feedback /save_map estimate_msgs/action/SaveMap "{filename: '/tmp/test.pcd'}"

# In another terminal, cancel it immediately
ros2 action send_goal --feedback /save_map estimate_msgs/action/SaveMap "{filename: '/tmp/test2.pcd'}"
# Press Ctrl+C to cancel
```

### 5. Verify the Saved Map

```bash
# Check the file exists
ls -lh /tmp/my_map.pcd

# View map info with PCL tools (if installed)
pcl_viewer /tmp/my_map.pcd

# Or use CloudCompare, MeshLab, etc.
```

---

## Python Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from estimate_msgs.action import SaveMap

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        self._action_client = ActionClient(self, SaveMap, 'save_map')

    def send_goal(self, filename):
        goal_msg = SaveMap.Goal()
        goal_msg.filename = filename

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        self.get_logger().info(f'Points saved: {result.points_saved}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.progress:.1f}% - {feedback.status}')

def main(args=None):
    rclpy.init(args=args)
    client = MapSaverClient()
    client.send_goal('/tmp/my_map.pcd')
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

Save this as `test_save_map.py`, make it executable, and run:
```bash
chmod +x test_save_map.py
./test_save_map.py
```

---

## Troubleshooting

### Action Server Not Available
**Symptom**: `Waiting for an action server to become available...` (times out)

**Solutions**:
1. Check RESPLE is running: `ros2 node list` should show `/RESPLE`
2. Check action is advertised: `ros2 action list`
3. Check RESPLE logs for errors

### Save Fails
**Symptom**: `Result: success: False`

**Common Causes**:
1. **Invalid path**: Ensure the directory exists and is writable
   ```bash
   mkdir -p /tmp/maps
   ros2 action send_goal /save_map estimate_msgs/action/SaveMap "{filename: '/tmp/maps/map.pcd'}"
   ```

2. **No map data**: Wait for RESPLE to build a map before saving

3. **Permission denied**: Use a writable location like `/tmp/` or `~/maps/`

### Empty Map
**Symptom**: `points_saved: 0` or very small number

**Cause**: Map not initialized yet. Let RESPLE run with sensor data for a few seconds to build the map.

---

## Implementation Details

The action server:
1. **Extracts** all points from the ikd-tree (10% progress)
2. **Writes** the point cloud to a PCD file in binary format (50% progress)
3. **Confirms** success and reports the number of points saved (100% progress)

The operation runs in a separate thread to allow cancellation and doesn't block the main RESPLE processing.

---

## Integration with Lifecycle

The action server is created during the `on_configure()` lifecycle transition and is available whenever RESPLE is in the **active** state.

To check lifecycle state:
```bash
ros2 lifecycle get /RESPLE
```

Expected: `active [3]`
