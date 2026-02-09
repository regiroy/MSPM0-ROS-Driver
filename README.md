# Quad-MD ROS2 Workspace

# MSPM0 Motor Driver – Setup & Debug Notes

This README captures the **exact steps, fixes, and lessons learned** while bringing up the `mspm0_motor_driver` ROS 2 node on **ROS 2 Jazzy (Ubuntu 24.04 / Python 3.12)**. Keep this handy to avoid rediscovering the same issues later.

---

## 1. Environment Assumptions

* Ubuntu 24.04
* ROS 2 **Jazzy**
* Python 3.12
* Workspace: `~/ros2_ws`
* Package: `mspm0_motor_driver`
* Serial device: `/dev/ttyUSB0`

Always start with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 2. Common Runtime Errors & Fixes

### ❌ `ModuleNotFoundError: No module named 'diagnostic_updater'`

**Cause**

* `diagnostic_updater` is a ROS package, not a pip module

**Fix (ROS Jazzy):**

```bash
sudo apt update
sudo apt install ros-jazzy-diagnostic-updater
```

**Verify:**

```bash
python3 -c "from diagnostic_updater import Updater; print('OK')"
```

---

### ❌ `ParameterAlreadyDeclaredException: wheel_base`

**Cause**

* Calling `declare_parameter()` more than once
* Usually happens if code is reloaded or node init logic is duplicated

**Rule:**

* Declare each parameter **once**, inside `__init__()` only

---

### ❌ `AttributeError: '_parse_values' not found`

**Cause**

* Function renamed or typo

**Fix**

* Use the actual implemented method name:

```python
_parse_int_values()
```

---

## 3. Command & Topic Sanity Checks

### Send velocity command

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

Expected:

* `$spd:` commands sent
* Encoder counts (`$MAll`) increase
* `$MSPD` shows non-zero values

---

### Verify odometry

```bash
ros2 topic echo /odom
```

Expected:

* `position.x / y` change over time
* `orientation.z / w` update on turns
* `frame_id: odom`, `child_frame_id: base_link`

---

## 4. TF Validation (Step A)

```bash
ros2 run tf2_tools view_frames
```

Expected TF tree:

```text
odom → base_link
```

This confirms odometry + TF broadcasting is correct.

---

## 5. Encoder & Serial Data (Step B)

Normal log patterns:

* `$MAll:x,x,x,x` → encoder totals
* `$MTEP:x,x,x,x` → delta ticks
* `$MSPD:x,x,x,x` → motor speeds

Movement confirmed when:

* Encoder totals increase
* MSPD values are non-zero

---

## 6. cmd_vel Watchdog / Timeout (Step C)

Expected warning when `/cmd_vel` stops:

```text
[WARN] cmd_vel timeout — stopping motors
```

Followed by:

```text
SEND: $spd:0,0,0,0#
```

This is **correct behavior**.

---

## 7. Where to Place Timing Logic (Important)

### ✅ Serial RX timestamp

```python
self.last_serial_rx_time = self.get_clock().now()
```

**Place this:**

* Inside the serial read handler
* Every time valid data is received
* **Not** outside the loop

---

### ✅ Periodic command timer

```python
self.create_timer(5.0, lambda: self.send_cmd('$read_vol#'))
```

**Place this:**

* In `__init__()`
* Outside all loops
* Timers must be created once

---

## 8. Python Imports Used

```python
import rclpy
from rclpy.node import Node
import serial
import time
import math
import tf2_ros

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from diagnostic_updater import Updater, DiagnosticStatusWrapper
```

⚠️ Do **not** put ROS packages in `requirements.txt`.
They are installed via `apt`, not `pip`.

---

## 9. Git Commit Messages Used

### Steps A–C

```
feat: add TF, odom publishing, and cmd_vel safety handling
```

### Step D

```
fix: stabilize serial handling and cmd_vel timeout behavior
```

---

## 10. Mental Checklist for Next Time

* Source ROS + workspace
* Install ROS dependencies via `apt`, not pip
* Verify `/cmd_vel`, `/odom`, and TF early
* Encoder counts increasing = hardware OK
* cmd_vel timeout stopping motors = GOOD

---

## Status

✔ Motors move
✔ Encoders working
✔ Odometry valid
✔ TF tree correct
✔ Safety timeout active

Ready for:
➡ Step F: diagnostics publishing
➡ Step G: robot_state_publisher + URDF
