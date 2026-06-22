# Yahboom Robot Control, Safety, Video, and VIT System

This project contains the main Python scripts used to control the Yahboom robot through MQTT and ROS 2, monitor LiDAR safety, stream live camera video using WebRTC, and publish MobileCLIP/VIT image embeddings over MQTT.

The system is split into four main files:

1. `mqtt_ros_node.py`
   Handles robot movement, camera servo control, LiDAR grid publishing, auto movement, and emergency-stop state.

2. `lidar_safety_node.py`
   Monitors the LiDAR front area and triggers either a hard emergency stop in manual mode or a soft stop in auto mode.

3. `robot_sender.py`
   Runs MobileCLIP-S1 on camera frames and publishes image embeddings to MQTT.

4. `webrtc_server.py`
   Streams live video to a browser using WebRTC and also runs MobileCLIP-S1 embedding publishing in the background.

---

# 1. `mqtt_ros_node.py`

This file is the main bridge between MQTT commands and ROS 2 robot movement.

It subscribes to MQTT command messages and converts them into ROS 2 messages such as `/cmd_vel`, `/servo_s1`, and `/servo_s2`. It also reads LiDAR data from `/scan`, builds a simple occupancy grid, and publishes robot status back to MQTT.

## Main Purpose

`mqtt_ros_node.py` is responsible for:

* Receiving movement commands from MQTT.
* Publishing movement commands to ROS 2 `/cmd_vel`.
* Controlling camera servos through `/servo_s1` and `/servo_s2`.
* Handling manual movement.
* Handling auto movement.
* Handling emergency stop.
* Reading LiDAR data from `/scan`.
* Publishing a local LiDAR grid to MQTT.
* Publishing drive status to MQTT.

---

## Important MQTT Topics

### `yahboom/cmd`

This is the main command topic. The dashboard or client sends commands here.

Example commands:

```text
fwd
bck
left
right
stop
auto_on
auto_off
estop_on
estop_off
cleft
cright
up
down
crst
```

### `yahboom/grid`

The robot publishes the LiDAR occupancy grid here.

This contains information such as:

* Grid data
* Resolution
* Robot position inside the grid
* Front distance
* Left distance
* Right distance
* Auto mode state
* E-stop state
* Timestamp

### `yahboom/drive/status`

The robot publishes movement and safety feedback here.

Example statuses:

```text
moving_forward
turning_left
stopped
auto_forward
auto_turn_left
auto_all_blocked
estop_active
estop_cleared
blocked_by_estop
```

---

## Important ROS 2 Topics

### `/cmd_vel`

Used to control robot movement.

The node publishes `Twist` messages here.

### `/servo_s1`

Controls the left-right camera servo.

### `/servo_s2`

Controls the up-down camera servo.

### `/scan`

The node subscribes to this LiDAR topic to read obstacle distances.

---

## Functions in `mqtt_ros_node.py`

### `__init__(self)`

Sets up the whole ROS 2 node.

It creates:

* ROS publishers for `/cmd_vel`, `/servo_s1`, and `/servo_s2`
* ROS subscriber for `/scan`
* MQTT client
* Timers for movement, servo movement, and grid publishing
* Movement state variables
* Auto mode state variables
* E-stop state variables
* LiDAR distance variables

This function is the starting setup for the MQTT-to-ROS bridge.

---

### `on_connect(self, client, userdata, flags, rc)`

Runs when the MQTT client connects to the broker.

If the connection is successful, it subscribes to the command topic:

```text
yahboom/cmd
```

This allows the robot to receive commands from the dashboard or client.

---

### `on_message(self, client, userdata, msg)`

Runs every time a new MQTT command is received.

This is one of the most important functions in the file.

It checks the command and decides what to do.

For example:

* `auto_on` enables auto mode.
* `auto_off` disables auto mode.
* `estop_on` activates emergency stop.
* `estop_off` clears emergency stop.
* `fwd` moves the robot forward.
* `bck` moves the robot backward.
* `left` turns the robot left.
* `right` turns the robot right.
* `stop` stops the robot.
* `cleft`, `cright`, `up`, and `down` move the camera servo.
* `crst` resets the camera position.

If emergency stop is active, movement commands are blocked.

---

### `scan_callback(self, scan_data)`

Runs whenever new LiDAR data is received from `/scan`.

It reads the LiDAR ranges and separates them into:

* Front distance
* Left distance
* Right distance

These distances are later used for:

* Auto movement
* Obstacle detection
* Grid publishing
* Drive status feedback

---

### `build_grid(self)`

Builds a simple 2D occupancy grid using LiDAR scan data.

The grid uses these values:

```text
0  = free space
1  = occupied space
-1 = unknown space
```

The robot is placed at the centre of the grid.

This function converts LiDAR angle and distance readings into grid cells.

---

### `publish_grid_mqtt(self)`

Publishes the LiDAR occupancy grid to MQTT topic:

```text
yahboom/grid
```

The payload includes:

* The grid
* Grid resolution
* Grid size
* Robot row and column
* Front distance
* Left distance
* Right distance
* Auto mode state
* E-stop state
* Timestamp

This allows the dashboard or client to display the robot’s surrounding map.

---

### `publish_drive_status(self, status)`

Publishes the robot’s current drive status to MQTT topic:

```text
yahboom/drive/status
```

The message includes:

* Current status
* Auto mode state
* E-stop state
* Front distance
* Left distance
* Right distance
* Timestamp

This is useful for debugging and displaying robot feedback on the dashboard.

---

### `enable_estop(self)`

Activates emergency stop.

When this function runs:

* `estop_active` becomes `True`
* Auto mode is disabled
* Servo movement is stopped
* Target movement is set to zero
* Robot is immediately stopped
* Drive status is published as `estop_active`

After this, movement commands are blocked until `estop_off` is received.

---

### `disable_estop(self)`

Clears emergency stop.

When this function runs:

* `estop_active` becomes `False`
* Auto mode remains off
* Robot movement is reset to zero
* Robot is stopped
* Drive status is published as `estop_cleared`

After this, the robot can receive movement commands again.

---

### `force_zero_motion(self)`

Resets all movement values to zero.

It sets:

```python
target_linear_x = 0.0
target_angular_z = 0.0
current_linear_x = 0.0
current_angular_z = 0.0
```

This is used when stopping the robot, activating e-stop, or disabling auto mode.

---

### `ramp_value(self, current, target, step)`

Smoothly changes the current speed toward the target speed.

Instead of jumping instantly from 0 to full speed, the robot slowly increases or decreases speed.

This makes movement smoother and less jerky.

---

### `auto_movement_logic(self)`

Controls the robot’s basic autonomous movement.

It uses LiDAR distances to decide whether the robot should:

* Move forward
* Turn left
* Turn right
* Stop if all directions are blocked

The logic is:

1. If the front is clear, move forward.
2. If the front is blocked and the left is clearer, turn left.
3. If the front and left are blocked but the right is clear, turn right.
4. If all directions are blocked, stop.

---

### `find_best_direction(self)`

Scans through possible directions and tries to find the most open direction for the robot.

It checks whether a direction has enough gap width for the robot to pass.

This function is useful for more advanced autonomous navigation, although the current auto movement mainly uses front, left, and right distances.

---

### `format_distance_angle(self, angle)`

Converts an angle into readable text.

For example:

```text
45.0 left
30.0 right
```

If no angle is available, it returns:

```text
unknown direction
```

---

### `publish_cmd_vel(self)`

Publishes movement commands to ROS 2 `/cmd_vel`.

This function runs repeatedly at the publish rate.

It:

* Stops the robot if e-stop is active.
* Runs auto movement logic if auto mode is active.
* Smoothly ramps current speed toward target speed.
* Publishes a `Twist` message to `/cmd_vel`.

This is the function that actually makes the robot move.

---

### `servo_tick(self)`

Runs repeatedly to move the camera servos.

It checks the active servo command and updates the servo angles.

Example commands:

* `cleft` moves camera left.
* `cright` moves camera right.
* `up` moves camera up.
* `down` moves camera down.
* `upcleft` moves camera up-left.
* `downcright` moves camera down-right.

It also stops moving the servo once the limit is reached.

---

### `publish_servos(self)`

Publishes the current camera servo positions to ROS 2.

It publishes:

* `servo_s1` to `/servo_s1`
* `servo_s2` to `/servo_s2`

This physically moves the camera mount.

---

### `stop_robot_now(self)`

Immediately publishes a zero `Twist` message to `/cmd_vel`.

This forces the robot to stop immediately.

It is used during:

* Emergency stop
* Auto soft stop
* Manual stop
* Node shutdown

---

### `destroy_node(self)`

Runs when the node is shutting down.

It:

* Activates stop behaviour
* Disables auto mode
* Stops servo movement
* Stops the robot
* Disconnects MQTT
* Cleans up the ROS node

This helps prevent the robot from continuing to move after the script exits.

---

### `main(args=None)`

Starts the ROS 2 node.

It:

1. Initializes ROS 2.
2. Creates the `MqttRosNode`.
3. Keeps the node running using `rclpy.spin`.
4. Cleans up when the program exits.

---

# 2. `lidar_safety_node.py`

This file is focused only on safety.

It monitors the front area of the robot using LiDAR and decides whether the robot should stop.

## Main Purpose

`lidar_safety_node.py` is responsible for:

* Reading LiDAR data from `/scan`.
* Checking whether an obstacle is too close in front.
* Publishing safety status to MQTT.
* Triggering hard e-stop in manual mode.
* Triggering soft stop in auto mode.
* Adding a 30-second grace period after `estop_off`.

---

## Important MQTT Topics

### `yahboom/cmd`

The safety node listens to this topic to know whether the robot is in manual mode or auto mode.

It also publishes safety stop commands back to this topic.

Example commands it may publish:

```text
estop_on
stop
auto_soft_stop
```

### `yahboom/safety/status`

The safety node publishes safety feedback here.

Example statuses:

```text
clear
warning
blocked
manual_estop_triggered
auto_blocked
estop_grace
auto_clear
auto_warning
```

---

## Safety Distance Settings

### `WARNING_DISTANCE = 0.60`

If an object is within 0.60 m, the node enters warning state.

### `BLOCK_DISTANCE = 0.35`

If an object is within 0.35 m, the node considers the path blocked.

### `FRONT_ANGLE = 20.0`

Only objects within 20 degrees left or right of the front are checked.

### `CONFIRM_POINTS = 8`

At least 8 LiDAR points must detect the obstacle before the warning or blocked state is confirmed.

This reduces false triggers.

### `ESTOP_REARM_DELAY = 30.0`

After `estop_off`, the LiDAR safety node waits 30 seconds before it can trigger another hard e-stop.

This prevents the robot from immediately stopping again after being resumed.

---

## Functions in `lidar_safety_node.py`

### `__init__(self)`

Sets up the LiDAR safety node.

It creates:

* ROS subscriber for `/scan`
* MQTT client
* MQTT command listener
* Safety status timer
* Variables for auto mode
* Variables for front distance
* Variables for e-stop rearm timing

---

### `on_connect(self, client, userdata, flags, rc)`

Runs when the node connects to the MQTT broker.

If successful, it subscribes to:

```text
yahboom/cmd
```

This allows the safety node to detect mode changes and e-stop commands.

---

### `on_message(self, client, userdata, msg)`

Runs whenever a command is received from MQTT.

It checks commands such as:

* `auto_on`
* `auto_off`
* `estop_on`
* `estop_off`

If `auto_on` is received, the safety node changes to auto mode.

If `auto_off` or `estop_on` is received, it changes back to manual safety behaviour.

If `estop_off` is received, it starts the 30-second grace period before LiDAR can trigger another e-stop.

---

### `scan_callback(self, scan_data)`

Runs whenever new LiDAR scan data is received.

It checks only the front cone of the robot.

It counts:

* How many LiDAR points are within warning distance.
* How many LiDAR points are within block distance.

Then it updates:

```python
front_distance
front_warning
front_blocked
```

If the front is blocked:

* In auto mode, it calls `trigger_auto_soft_stop()`.
* In manual mode, it calls `trigger_manual_estop()`.

---

### `trigger_manual_estop(self)`

Triggers a hard emergency stop when the robot is in manual mode.

It publishes:

```text
estop_on
stop
```

to `yahboom/cmd`.

It also publishes a safety status message to:

```text
yahboom/safety/status
```

This tells the dashboard that a real e-stop has been triggered.

If the 30-second grace period is active, it does not trigger e-stop yet.

---

### `trigger_auto_soft_stop(self)`

Triggers a soft stop when the robot is in auto mode.

It publishes:

```text
auto_soft_stop
```

to `yahboom/cmd`.

This stops the robot but does not latch emergency stop.

This is useful because during auto mode, the robot should stop and allow the client or auto logic to decide the next direction, instead of locking the whole system.

---

### `publish_status(self)`

Publishes the current safety status every 0.5 seconds.

The status depends on:

* Whether auto mode is active
* Whether front area is clear
* Whether warning distance is reached
* Whether block distance is reached
* Whether e-stop grace period is active

Example statuses:

```text
clear
warning
blocked
auto_clear
auto_warning
auto_blocked
estop_grace
estop_grace_warning
estop_grace_clear
```

---

### `publish_mode_status(self)`

Publishes the current safety mode.

Example output:

```text
mode=auto,distance=0.55m,estop=false,auto=true
```

or:

```text
mode=manual,distance=0.55m,estop=false,auto=false
```

---

### `get_distance_text(self)`

Converts the current front distance into readable text.

If distance is known, it returns something like:

```text
0.42m
```

If distance is unknown, it returns:

```text
unknown
```

---

### `destroy_node(self)`

Stops the MQTT loop, disconnects from MQTT, and shuts down the ROS node safely.

---

### `main(args=None)`

Starts the LiDAR safety node.

It:

1. Initializes ROS 2.
2. Creates the `LidarSafetyNode`.
3. Keeps it running using `rclpy.spin`.
4. Cleans up when the program exits.

---

# 3. `robot_sender.py`

This file runs MobileCLIP-S1 on the robot camera feed and publishes image embeddings through MQTT.

## Main Purpose

`robot_sender.py` is responsible for:

* Opening the robot camera.
* Loading MobileCLIP-S1.
* Capturing camera frames.
* Running image embedding inference every few frames.
* Publishing image embeddings to MQTT.
* Publishing VIT status updates to MQTT.

---

## Important MQTT Topics

### `yahboom/vit/embedding`

The image embedding is published here.

The payload includes:

* Raw embedding size
* Embedding dimension
* Data type
* Frame number
* Image file size
* Base64 encoded embedding data

### `yahboom/vit/status`

Status messages are published here.

Example statuses:

```text
vit_encoder_started
running
camera_error
embedding_error
fatal_error
vit_encoder_stopped
```

---

## Important Settings

### `INFERENCE_EVERY_N_FRAMES = 5`

The model does not run on every frame.

It runs once every 5 frames to reduce processing load.

### `EMBEDDING_BYTES = 2048`

This controls the embedding size.

Options:

```text
512 bytes  = 128 dimensions
1024 bytes = 256 dimensions
2048 bytes = 512 dimensions
```

### `SHOW_PREVIEW = False`

This should stay `False` when running over SSH or without a monitor.

---

## Functions in `robot_sender.py`

### `create_mqtt_client()`

Creates the MQTT client.

It supports both newer and older versions of the Paho MQTT library.

---

### `publish_status(client, status, extra=None)`

Publishes status information to:

```text
yahboom/vit/status
```

The message includes:

* Status text
* Timestamp
* Extra information if provided

This is used to let the dashboard know whether the VIT encoder is running, stopped, or facing errors.

---

### `load_model()`

Loads the MobileCLIP-S1 model using `open_clip`.

It chooses the device automatically:

* Uses CUDA if available.
* Uses CPU if CUDA is not available.

It returns:

```python
model
preprocess
device
```

These are needed for image embedding inference.

---

### `get_embedding(frame, model, preprocess, device)`

Converts one camera frame into an image embedding.

Steps:

1. Converts OpenCV BGR image to RGB.
2. Converts the frame into a PIL image.
3. Applies MobileCLIP preprocessing.
4. Sends the image through the model.
5. Normalizes the embedding.
6. Slices the embedding based on the selected byte size.
7. Returns the embedding as a NumPy float32 array.

---

### `open_camera()`

Opens the camera using OpenCV.

It uses:

```python
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
```

If the camera cannot be opened, it returns `None`.

---

### `main()`

Runs the full VIT sender program.

It:

1. Loads the MobileCLIP-S1 model.
2. Connects to MQTT.
3. Publishes `vit_encoder_started`.
4. Opens the camera.
5. Reads frames in a loop.
6. Runs embedding inference every 5 frames.
7. Publishes embeddings to MQTT.
8. Publishes running status every 10 embeddings.
9. Handles errors.
10. Releases the camera and disconnects MQTT when stopped.

---

# 4. `webrtc_server.py`

This file provides live browser video streaming using WebRTC.

It also includes MobileCLIP-S1 embedding publishing, similar to `robot_sender.py`, but combined with the WebRTC video server.

## Main Purpose

`webrtc_server.py` is responsible for:

* Opening the robot camera.
* Streaming live video to a browser.
* Creating WebRTC peer connections.
* Running MobileCLIP-S1 in the background.
* Publishing embeddings to MQTT.
* Allowing embedding size changes through MQTT commands.
* Publishing VIT status updates.

---

## Web Server

The server runs on:

```text
http://<robot-ip>:8080
```

The page shows:

* Live WebRTC video
* Connection status
* Restart video button

---

## Important MQTT Topics

### `yahboom/vit/embedding`

Publishes image embeddings.

### `yahboom/vit/status`

Publishes VIT status updates.

### `yahboom/vit/command`

Receives commands to change embedding size.

Supported commands:

```text
embds1
embds2
embds3
```

These mean:

```text
embds1 = 512 bytes  = 128 dimensions
embds2 = 1024 bytes = 256 dimensions
embds3 = 2048 bytes = 512 dimensions
```

---

## Functions in `webrtc_server.py`

### `get_local_ip()`

Finds the robot’s local IP address.

This is used to print the WebRTC dashboard link, such as:

```text
http://192.168.x.x:8080
```

If the IP cannot be detected, it returns:

```text
127.0.0.1
```

---

### `init_camera()`

Opens the camera using OpenCV.

It sets:

```python
WIDTH = 640
HEIGHT = 480
FPS = 30
```

If the camera cannot be opened, it raises an error.

---

### `load_model()`

Loads the MobileCLIP-S1 model using `open_clip`.

It uses:

* CUDA if available
* CPU if CUDA is not available

The loaded model is used by the VIT worker to create image embeddings.

---

### `publish_status(client, status, extra=None)`

Publishes status information to:

```text
yahboom/vit/status
```

It is used to tell the dashboard when the VIT encoder starts, runs, changes embedding size, or encounters an error.

---

### `create_mqtt_client()`

Creates the MQTT client.

It also defines the MQTT callbacks inside the function.

When connected, it subscribes to:

```text
yahboom/vit/command
```

This allows the system to receive embedding size commands.

---

### `_on_connect(c, _ud, _flags, rc)`

This is the MQTT connection callback inside `create_mqtt_client()`.

When MQTT connects successfully, it:

* Subscribes to `yahboom/vit/command`
* Prints the embedding topic
* Prints the status topic

---

### `_on_message(c, _ud, msg)`

This is the MQTT message callback inside `create_mqtt_client()`.

It handles commands such as:

```text
embds1
embds2
embds3
```

When one of these commands is received, it changes the embedding size at runtime.

---

### `get_target_dims()`

Returns the current embedding dimension.

Possible values:

```text
128
256
512
```

This function uses a lock so that the value is safe to read from multiple threads.

---

### `get_embedding_bytes()`

Returns the current embedding size in bytes.

Possible values:

```text
512
1024
2048
```

---

### `set_embedding_size(new_bytes)`

Changes the embedding size at runtime.

For example:

```text
512 bytes  -> 128 dimensions
1024 bytes -> 256 dimensions
2048 bytes -> 512 dimensions
```

This function is called when an MQTT command such as `embds1`, `embds2`, or `embds3` is received.

---

### `get_embedding(frame)`

Converts one camera frame into a MobileCLIP embedding.

Steps:

1. Converts OpenCV BGR frame to RGB.
2. Converts the frame to PIL image.
3. Applies preprocessing.
4. Runs MobileCLIP image encoder.
5. Normalizes the full embedding.
6. Slices the embedding based on the current target dimension.
7. Re-normalizes the sliced embedding.
8. Returns the embedding as a float32 NumPy array.

---

### `camera_worker()`

Runs in a background thread.

It continuously:

* Reads frames from the camera.
* Stores the latest frame in `latest_frame`.

Both the WebRTC video stream and VIT worker use this same latest frame.

This prevents multiple scripts from trying to open the camera at the same time.

---

### `vit_worker()`

Runs in a background thread.

It continuously:

* Reads the latest camera frame.
* Runs MobileCLIP inference every few frames.
* Converts the embedding into bytes.
* Encodes the bytes using Base64.
* Publishes the embedding to `yahboom/vit/embedding`.
* Publishes status updates to `yahboom/vit/status`.

This is the VIT embedding part of the WebRTC server.

---

### `CameraVideoTrack`

This is a custom WebRTC video track.

It takes the latest camera frame and sends it to the browser through WebRTC.

If no camera frame is available yet, it sends a blank black frame.

---

### `CameraVideoTrack.recv(self)`

This function is called repeatedly by WebRTC when the browser needs the next video frame.

It:

1. Gets the latest camera frame.
2. Resizes it to the selected width and height.
3. Converts it into an `av.VideoFrame`.
4. Sends it to the browser.

---

### `index(request)`

Serves the main browser page.

The page contains:

* A video element
* A status text
* A restart video button
* JavaScript to automatically start WebRTC video

When the user opens the robot’s IP address in a browser, this function sends the HTML page.

---

### `offer(request)`

Handles the WebRTC offer from the browser.

It:

1. Receives the browser’s WebRTC offer.
2. Creates a new peer connection.
3. Adds the camera video track.
4. Creates a WebRTC answer.
5. Sends the answer back to the browser.

This is required for the browser and robot to connect through WebRTC.

---

### `on_shutdown(app)`

Runs when the WebRTC server shuts down.

It:

* Stops background threads.
* Closes WebRTC peer connections.
* Releases the camera.
* Stops and disconnects MQTT.

This prevents the camera or MQTT connection from being left open.

---

### `main()`

Starts the full WebRTC and VIT server.

It:

1. Loads the MobileCLIP-S1 model.
2. Connects to MQTT.
3. Publishes `vit_encoder_started`.
4. Starts the camera worker thread.
5. Starts the VIT worker thread.
6. Starts the web server.
7. Prints the dashboard URL.

---

# How the Whole System Works Together

## Manual Control Flow

1. The dashboard sends a command to:

```text
yahboom/cmd
```

2. `mqtt_ros_node.py` receives the command.

3. It converts the command into a ROS 2 `/cmd_vel` message.

4. The robot moves.

5. `lidar_safety_node.py` keeps checking the front LiDAR area.

6. If the robot is too close to an obstacle, it publishes:

```text
estop_on
stop
```

7. `mqtt_ros_node.py` receives `estop_on` and locks movement.

---

## Auto Mode Flow

1. The dashboard sends:

```text
auto_on
```

2. `mqtt_ros_node.py` enables auto mode.

3. `lidar_safety_node.py` also switches to auto mode.

4. The robot moves forward if the path is clear.

5. If an obstacle is detected, `lidar_safety_node.py` sends:

```text
auto_soft_stop
```

6. `mqtt_ros_node.py` stops the robot but does not lock e-stop.

7. Auto logic or the client can then decide the next movement.

---

## Video and VIT Flow

1. `webrtc_server.py` opens the camera.
2. The browser connects to:

```text
http://<robot-ip>:8080
```

3. WebRTC sends live video to the browser.
4. The VIT worker takes camera frames.
5. MobileCLIP-S1 creates embeddings.
6. Embeddings are published to:

```text
yahboom/vit/embedding
```

7. Status updates are published to:

```text
yahboom/vit/status
```

---

# Command Summary

## Movement Commands

```text
fwd        Move forward
bck        Move backward
left       Turn left
right      Turn right
stop       Stop robot
fwdleft    Move forward-left
fwdright   Move forward-right
bckleft    Move backward-left
bckright   Move backward-right
```

## Camera Servo Commands

```text
cleft       Move camera left
cright      Move camera right
up          Move camera up
down        Move camera down
upcleft     Move camera up-left
upcright    Move camera up-right
downcleft   Move camera down-left
downcright  Move camera down-right
cstop       Stop camera movement
crst        Reset camera position
```

## Safety Commands

```text
estop_on    Activate emergency stop
estop_off   Clear emergency stop
```

## Auto Mode Commands

```text
auto_on     Enable auto mode
auto_off    Disable auto mode
```

## VIT Embedding Size Commands

```text
embds1      512-byte embedding, 128 dimensions
embds2      1024-byte embedding, 256 dimensions
embds3      2048-byte embedding, 512 dimensions
```

---

# Basic Run Order

A typical run order is:

```bash
python3 mqtt_ros_node.py
```

```bash
python3 lidar_safety_node.py
```

```bash
python3 webrtc_server.py
```

If only VIT embedding is needed without WebRTC video:

```bash
python3 robot_sender.py
```

---

# Testing with MQTT

Move forward:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "fwd"
```

Stop:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "stop"
```

Enable auto mode:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "auto_on"
```

Disable auto mode:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "auto_off"
```

Activate e-stop:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "estop_on"
```

Clear e-stop:

```bash
mosquitto_pub -h localhost -t yahboom/cmd -m "estop_off"
```

Change VIT embedding size:

```bash
mosquitto_pub -h localhost -t yahboom/vit/command -m "embds1"
```

---

# Notes

* `mqtt_ros_node.py` is the main robot control node.
* `lidar_safety_node.py` is the safety monitoring node.
* `robot_sender.py` is only for VIT embedding publishing.
* `webrtc_server.py` combines live WebRTC video and VIT embedding publishing.
* In manual mode, LiDAR obstacles trigger a hard emergency stop.
* In auto mode, LiDAR obstacles trigger a soft stop only.
* `estop_off` starts a 30-second grace period before LiDAR can trigger another hard e-stop.
* WebRTC video can be viewed from a browser using the robot’s IP address and port `8080`.
