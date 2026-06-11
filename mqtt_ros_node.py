import rclpy
import json
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

import paho.mqtt.client as mqtt
import numpy as np
import time


# =========================
# MQTT SETTINGS
# =========================
BROKER_IP = "localhost"

# Client sends movement commands here
TOPIC = "yahboom/cmd"

# Client subscribes here to receive the local LiDAR grid/map
TOPIC_GRID = "yahboom/grid"

# Client subscribes here to receive drive state feedback
TOPIC_DRIVE_STATUS = "yahboom/drive/status"


# =========================
# MANUAL MOVEMENT SETTINGS
# =========================
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0

# Publish /cmd_vel at 20Hz
PUBLISH_RATE = 20.0

# Smoothing values
LINEAR_STEP = 0.02
ANGULAR_STEP = 0.05

# =========================
# AUTO MOVEMENT SETTINGS
# =========================
AUTO_LINEAR_SPEED = 0.25
AUTO_TURN_SPEED = 0.65

AUTO_BLOCK_DISTANCE = 0.40   # Must be larger than your lidar_safety_node estop distance
AUTO_SIDE_DISTANCE  = 0.30
AUTO_90_TURN_TIME = 2.4      # Approximate 90-degree turn duration - tune after testing

BEST_DIRECTION_WINDOW = 10.0

ROBOT_WIDTH_M = 0.17
GAP_SAFETY_MARGIN = 1.3
MIN_GAP_WIDTH_M = ROBOT_WIDTH_M * GAP_SAFETY_MARGIN  # ~0.22m effective minimum


# =========================
# LIDAR SETTINGS
# =========================
RAD2DEG = 180.0 / np.pi

# LiDAR sector angles
FRONT_ANGLE = 25.0
SIDE_ANGLE_MIN = 30.0
SIDE_ANGLE_MAX = 110.0

MAX_SCAN_RANGE = 3.0


# =========================
# OCCUPANCY GRID SETTINGS
# =========================
GRID_RESOLUTION = 0.05
GRID_PUBLISH_RATE = 5.0

CELL_FREE = 0
CELL_OCCUPIED = 1
CELL_UNKNOWN = -1


# =========================
# CAMERA SERVO SETTINGS
# =========================
S1_MIN, S1_MAX = -90, 90
S2_MIN, S2_MAX = -90, 20

CAM_STEP = 10
S_CAM_STEP = 5


# =========================
# COMMAND GROUPS
# =========================
MOVEMENT_COMMANDS = {
    "fwd",
    "fwdright",
    "fwdleft",
    "bck",
    "bckright",
    "bckleft",
    "left",
    "right",
    "stop",
}

CAMERA_COMMANDS = {
    "cleft",
    "cright",
    "up",
    "upcleft",
    "upcright",
    "down",
    "downcleft",
    "downcright",
    "cstop",
    "crst",
}

ESTOP_COMMANDS = {
    "estop_on",
    "estop_off",
}


AUTO_COMMANDS = {
    "auto_on",
    "auto_off",
}

class MqttRosNode(Node):
    def __init__(self):
        super().__init__("mqtt_ros_cmd_vel_node")

        # =========================
        # ROS PUBLISHERS
        # =========================
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.servo_s1_pub = self.create_publisher(Int32, "/servo_s1", 10)
        self.servo_s2_pub = self.create_publisher(Int32, "/servo_s2", 10)

        # =========================
        # ROS SUBSCRIBER: LIDAR
        # =========================
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # =========================
        # MOVEMENT STATE
        # =========================
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        self.last_command = "stop"

        # =========================
        # MODE / SAFETY STATE
        # =========================
        self.estop_active = False
        self.auto_mode = False

        # =========================
        # LIDAR STATE
        # =========================
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.scan_angles = np.array([])
        self.scan_ranges = np.array([])

        grid_cells = int((MAX_SCAN_RANGE * 2) / GRID_RESOLUTION)
        self.grid_size = grid_cells
        self.grid_centre = grid_cells // 2

        # =========================
        # SERVO STATE
        # =========================
        self.servo_s1 = 0
        self.servo_s2 = -60
        self.active_servo_cmd = None
        
        # =========================
        # AUTO LIDAR STATE
        # =========================
        self.auto_state = "idle"        # idle | forward | turning_90
        self.turn_direction = "left"    # left or right (fallback)
        self.turn_start_time = 0.0

        # =========================
        # TIMERS
        # =========================
        self.movement_timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_cmd_vel
        )

        self.servo_timer = self.create_timer(
            0.1,
            self.servo_tick
        )

        self.grid_timer = self.create_timer(
            1.0 / GRID_PUBLISH_RATE,
            self.publish_grid_mqtt
        )

        # =========================
        # MQTT SETUP
        # =========================
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.get_logger().info(f"Connecting to MQTT broker at {BROKER_IP}:1883")
        self.mqtt_client.connect(BROKER_IP, 1883, 60)
        self.mqtt_client.loop_start()

        self.get_logger().info("MQTT ROS node started")
        self.get_logger().info(f"Listening to MQTT topic: {TOPIC}")
        self.get_logger().info(f"Publishing grid to MQTT topic: {TOPIC_GRID}")
        self.get_logger().info(f"Publishing drive status to MQTT topic: {TOPIC_DRIVE_STATUS}")
        self.get_logger().info(f"Grid: {self.grid_size}x{self.grid_size} cells @ {GRID_RESOLUTION}m/cell")
        self.get_logger().info("Publishing to ROS topics: /cmd_vel, /servo_s1, /servo_s2")
        self.get_logger().info("Hard e-stop: estop_on / estop_off")
        self.get_logger().info("Auto soft stop: auto_soft_stop")
    
    # =========================
    # MQTT CALLBACKS
    # =========================
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe(TOPIC)
            self.get_logger().info(f"Subscribed to MQTT topic: {TOPIC}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker. Code: {rc}")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().lower()

        if command != self.last_command:
            self.get_logger().info(f"Received MQTT command: {command}")
            self.last_command = command

        # =========================
        # MODE COMMANDS
        # =========================
        if command == "auto_on":
            self.auto_mode = True
            self.auto_state = "forward"    #immediately start moving forward
            self.force_zero_motion()
            self.get_logger().info("AUTO MODE ENABLED - moving forward")
            return

        elif command == "auto_off":
            self.auto_mode = False
            self.auto_state = "idle"
            self.force_zero_motion()
            self.stop_robot_now()
            self.get_logger().info("AUTO MODE DISABLED")
            return

        # =========================
        # HARD E-STOP COMMANDS
        # =========================
        if command == "estop_on":
            self.enable_estop()
            return

        elif command == "estop_off":
            self.disable_estop()
            return

        # =========================
        # AUTO SOFT STOP
        # This stops the robot but does NOT latch e-stop.
        # Used when LiDAR detects obstacle during auto mode.
        # Client can still send left/right/fwd after this.
        # =========================
        if command == "auto_soft_stop":
            self.force_zero_motion()
            self.stop_robot_now()
            self.publish_drive_status("auto_soft_stop")
            self.get_logger().warn("AUTO SOFT STOP - robot halted, e-stop not latched")
            return
        # =========================
        # BLOCK MOVEMENT WHILE HARD E-STOP ACTIVE
        # =========================
        if self.estop_active:
            if command in MOVEMENT_COMMANDS:
                self.force_zero_motion()
                self.stop_robot_now()
                self.publish_drive_status(f"blocked_by_estop:{command}")
                self.get_logger().warn(
                    f"Command blocked because E-stop is active: {command}"
                )
                return

        # =========================
        # MANUAL / CLIENT MOVEMENT COMMANDS
        # These commands work in both manual mode and auto/client-decision mode.
        # The client decides the direction during auto mode.
        # =========================
        if command == "fwd":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = 0.0
            self.publish_drive_status("moving_forward")

        elif command == "fwdright":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = -ANGULAR_SPEED
            self.publish_drive_status("moving_forward_right")

        elif command == "fwdleft":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = ANGULAR_SPEED
            self.publish_drive_status("moving_forward_left")

        elif command == "bck":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = 0.0
            self.publish_drive_status("moving_backward")

        elif command == "bckright":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = ANGULAR_SPEED
            self.publish_drive_status("moving_backward_right")

        elif command == "bckleft":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = -ANGULAR_SPEED
            self.publish_drive_status("moving_backward_left")

        elif command == "left":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = 0.0
            self.target_angular_z = ANGULAR_SPEED
            self.publish_drive_status("turning_left")

        elif command == "right":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = 0.0
            self.target_angular_z = -ANGULAR_SPEED
            self.publish_drive_status("turning_right")

        elif command == "stop":
            self.auto_mode = False
            self.auto_state = "idle"
            self.force_zero_motion()
            self.stop_robot_now()
            self.publish_drive_status("stopped")
            
        # =========================
        # CAMERA MOVEMENT COMMANDS
        # =========================
        elif command == "cleft":
            self.active_servo_cmd = "cleft"

        elif command == "cright":
            self.active_servo_cmd = "cright"

        elif command == "up":
            self.active_servo_cmd = "up"

        elif command == "upcleft":
            self.active_servo_cmd = "upcleft"

        elif command == "upcright":
            self.active_servo_cmd = "upcright"

        elif command == "down":
            self.active_servo_cmd = "down"

        elif command == "downcleft":
            self.active_servo_cmd = "downcleft"

        elif command == "downcright":
            self.active_servo_cmd = "downcright"

        elif command == "cstop":
            self.active_servo_cmd = None

        elif command == "crst":
            self.active_servo_cmd = None
            self.servo_s1 = 0
            self.servo_s2 = -60
            self.publish_servos()

        else:
            self.get_logger().warn(f"Unknown command: {command}")
            self.publish_drive_status(f"unknown_command:{command}")

    # =========================
    # LIDAR CALLBACK
    # =========================
    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        angles = (
            scan_data.angle_min
            + scan_data.angle_increment * np.arange(len(ranges))
        ) * RAD2DEG

        angles = np.where(angles > 180, angles - 360, angles)

        valid_mask = (
            np.isfinite(ranges)
            & (ranges >= scan_data.range_min)
            & (ranges <= min(scan_data.range_max, MAX_SCAN_RANGE))
        )

        self.scan_angles = angles[valid_mask]
        self.scan_ranges = ranges[valid_mask]

        front_vals = self.scan_ranges[
            np.abs(self.scan_angles) <= FRONT_ANGLE
        ]

        left_vals = self.scan_ranges[
            (self.scan_angles >= SIDE_ANGLE_MIN)
            & (self.scan_angles <= SIDE_ANGLE_MAX)
        ]

        right_vals = self.scan_ranges[
            (self.scan_angles >= -SIDE_ANGLE_MAX)
            & (self.scan_angles <= -SIDE_ANGLE_MIN)
        ]

        self.front_distance = float(np.min(front_vals)) if len(front_vals) > 0 else None
        self.left_distance = float(np.min(left_vals)) if len(left_vals) > 0 else None
        self.right_distance = float(np.min(right_vals)) if len(right_vals) > 0 else None
        
    # =========================
    # OCCUPANCY GRID BUILDER
    # =========================
    def build_grid(self):
        size = self.grid_size
        centre = self.grid_centre

        grid = np.full((size, size), CELL_UNKNOWN, dtype=np.int8)

        if len(self.scan_angles) == 0:
            return grid.tolist()

        angles_rad = self.scan_angles / RAD2DEG

        for angle_r, dist in zip(angles_rad, self.scan_ranges):
            # ROS convention:
            # +x = forward
            # +y = left
            x_m = dist * np.cos(angle_r)
            y_m = dist * np.sin(angle_r)

            steps = max(1, int(dist / GRID_RESOLUTION))

            for step in range(steps):
                frac = step / steps
                xi = x_m * frac
                yi = y_m * frac

                row = centre - int(round(xi / GRID_RESOLUTION))
                col = centre + int(round(yi / GRID_RESOLUTION))

                if 0 <= row < size and 0 <= col < size:
                    if grid[row, col] != CELL_OCCUPIED:
                        grid[row, col] = CELL_FREE

            end_row = centre - int(round(x_m / GRID_RESOLUTION))
            end_col = centre + int(round(y_m / GRID_RESOLUTION))

            if 0 <= end_row < size and 0 <= end_col < size:
                grid[end_row, end_col] = CELL_OCCUPIED

        return grid.tolist()

    # =========================
    # PUBLISH GRID OVER MQTT
    # =========================
    def publish_grid_mqtt(self):
        if len(self.scan_ranges) == 0:
            return

        grid = self.build_grid()

        payload = {
            "grid": grid,
            "resolution": GRID_RESOLUTION,
            "size": self.grid_size,
            "robot_row": self.grid_centre,
            "robot_col": self.grid_centre,
            "front": self.front_distance,
            "left": self.left_distance,
            "right": self.right_distance,
            "auto_mode": self.auto_mode,
            "estop_active": self.estop_active,
            "timestamp": time.time(),
        }

        try:
            self.mqtt_client.publish(
                TOPIC_GRID,
                json.dumps(payload),
                qos=0
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to publish grid: {e}")
            
        # =========================
    # DRIVE STATUS PUBLISHER
    # =========================
    def publish_drive_status(self, status):
        payload = {
            "status": status,
            "auto_mode": self.auto_mode,
            "estop_active": self.estop_active,
            "front": self.front_distance,
            "left": self.left_distance,
            "right": self.right_distance,
            "timestamp": time.time(),
        }

        try:
            self.mqtt_client.publish(
                TOPIC_DRIVE_STATUS,
                json.dumps(payload),
                qos=0
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to publish drive status: {e}")

    # =========================
    # E-STOP HELPERS
    # =========================
    def enable_estop(self):
        self.estop_active = True
        self.auto_mode = False         
        self.auto_state = "idle"       
        self.active_servo_cmd = None
        self.force_zero_motion()
        self.stop_robot_now()
        self.publish_drive_status("estop_active")
        self.get_logger().warn("EMERGENCY STOP ACTIVE - robot halted and movement locked")

    def disable_estop(self):
        self.estop_active = False
        self.auto_mode = False 
        self.auto_state = "idle"
        self.force_zero_motion()
        self.stop_robot_now()
        self.publish_drive_status("estop_cleared")
        self.get_logger().info("EMERGENCY STOP CLEARED - robot control enabled")

    def force_zero_motion(self):
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

    # =========================
    # SMOOTHING
    # =========================
    def ramp_value(self, current, target, step):
        if abs(target - current) <= step:
            return target

        return current + step if current < target else current - step
    
    # =========================
    # AUTO MOVEMENT LOGIC
    # =========================
    def auto_movement_logic(self):
        # Guard: wait for first LiDAR scan
        if self.front_distance is None:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            self.get_logger().warn("AUTO MODE: waiting for LiDAR /scan data")
            return

        front_blocked = self.front_distance < AUTO_BLOCK_DISTANCE
        left_blocked  = self.left_distance  < AUTO_SIDE_DISTANCE   if self.left_distance  is not None else False
        right_blocked = self.right_distance < AUTO_SIDE_DISTANCE   if self.right_distance is not None else False

        left_dist  = self.left_distance  if self.left_distance  is not None else 0.0
        right_dist = self.right_distance if self.right_distance is not None else 0.0

        if not front_blocked:
            # Path is clear - drive forward
            self.target_linear_x  =  AUTO_LINEAR_SPEED
            self.target_angular_z =  0.0
            self.publish_drive_status("auto_forward")

        elif not left_blocked and left_dist >= right_dist:
            # Front blocked, left side is more open - turn left
            self.target_linear_x  =  0.0
            self.target_angular_z =  AUTO_TURN_SPEED
            self.publish_drive_status("auto_turn_left")

        elif not right_blocked:
            # Front and left blocked, right is open - turn right
            self.target_linear_x  =  0.0
            self.target_angular_z = -AUTO_TURN_SPEED
            self.publish_drive_status("auto_turn_right")

        else:
            # All directions blocked - stop
            self.force_zero_motion()
            self.stop_robot_now()
            self.publish_drive_status("auto_all_blocked")
            self.get_logger().warn(
                f"AUTO: all blocked"
                f"front={self.front_distance:.2f}m "
                f"left={self.left_distance}m "
                f"right={self.right_distance}m"
            )

    def find_best_direction(self):
        """
        Return angle (degrees) of the most open direction wide enough for the robot.
        Returns None if no passable direction found.
        """
        if len(self.scan_angles) == 0:
            return None

        best_angle = None
        best_score = -1.0
        half_win = BEST_DIRECTION_WINDOW / 2.0

        for candidate in np.arange(-180, 180, 5):
            mask = (
                (self.scan_angles >= candidate - half_win)
                & (self.scan_angles <= candidate + half_win)
            )
            sector_ranges = self.scan_ranges[mask]

            if len(sector_ranges) == 0:
                continue

            nearest = float(np.min(sector_ranges))

            # Estimate gap width at nearest obstacle distance
            half_win_rad = (half_win * 2.0) / 2.0 / RAD2DEG
            estimated_gap_width = 2.0 * nearest * np.tan(half_win_rad)

            if estimated_gap_width < MIN_GAP_WIDTH_M:
                continue

            # Penalise rear directions
            forward_bias = 1.0 if abs(candidate) <= 90.0 else 0.6
            score = nearest * forward_bias

            if score > best_score:
                best_score = score
                best_angle = float(candidate)

        return best_angle

    def format_distance_angle(self, angle):
        if angle is None:
            return "unknown direction"
        side = "left" if angle > 0 else "right"
        return f"{abs(angle):.1f} {side}"

    # =========================
    # PUBLISH MOVEMENT
    # =========================
    def publish_cmd_vel(self):
        if self.estop_active:
            self.force_zero_motion()
            self.stop_robot_now()
            return
        
        if self.auto_mode:              
            self.auto_movement_logic()

        self.current_linear_x = self.ramp_value(
            self.current_linear_x,
            self.target_linear_x,
            LINEAR_STEP
        )

        self.current_angular_z = self.ramp_value(
            self.current_angular_z,
            self.target_angular_z,
            ANGULAR_STEP
        )

        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z

        self.cmd_vel_pub.publish(twist)
    
    # =========================
    # SERVO MOVEMENT
    # =========================
    def servo_tick(self):
        if self.active_servo_cmd is None:
            return

        cmd = self.active_servo_cmd
        changed = True

        if cmd == "cleft":
            self.servo_s1 = max(S1_MIN, self.servo_s1 - CAM_STEP)

        elif cmd == "cright":
            self.servo_s1 = min(S1_MAX, self.servo_s1 + CAM_STEP)

        elif cmd == "up":
            self.servo_s2 = min(S2_MAX, self.servo_s2 + CAM_STEP)

        elif cmd == "down":
            self.servo_s2 = max(S2_MIN, self.servo_s2 - CAM_STEP)

        elif cmd == "upcleft":
            self.servo_s1 = max(S1_MIN, self.servo_s1 - S_CAM_STEP)
            self.servo_s2 = min(S2_MAX, self.servo_s2 + S_CAM_STEP)

        elif cmd == "upcright":
            self.servo_s1 = min(S1_MAX, self.servo_s1 + S_CAM_STEP)
            self.servo_s2 = min(S2_MAX, self.servo_s2 + S_CAM_STEP)

        elif cmd == "downcleft":
            self.servo_s1 = max(S1_MIN, self.servo_s1 - S_CAM_STEP)
            self.servo_s2 = max(S2_MIN, self.servo_s2 - S_CAM_STEP)

        elif cmd == "downcright":
            self.servo_s1 = min(S1_MAX, self.servo_s1 + S_CAM_STEP)
            self.servo_s2 = max(S2_MIN, self.servo_s2 - S_CAM_STEP)

        else:
            changed = False

        s1_at_limit = self.servo_s1 <= S1_MIN or self.servo_s1 >= S1_MAX
        s2_at_limit = self.servo_s2 <= S2_MIN or self.servo_s2 >= S2_MAX

        if cmd in ("cleft", "cright") and s1_at_limit:
            self.active_servo_cmd = None

        elif cmd in ("up", "down") and s2_at_limit:
            self.active_servo_cmd = None

        if changed:
            self.publish_servos()

    def publish_servos(self):
        msg_s1 = Int32()
        msg_s1.data = self.servo_s1
        self.servo_s1_pub.publish(msg_s1)

        msg_s2 = Int32()
        msg_s2.data = self.servo_s2
        self.servo_s2_pub.publish(msg_s2)

        self.get_logger().info(
            f"Servo - s1: {self.servo_s1}d, s2: {self.servo_s2}d"
        )

    # =========================
    # STOP ROBOT NOW
    # =========================
    def stop_robot_now(self):
        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        try:
            self.cmd_vel_pub.publish(twist)
        except Exception:
            pass
        
    # =========================
    # CLEAN EXIT
    # =========================
    def destroy_node(self):
        self.estop_active = True
        self.auto_mode = False 
        self.auto_state = "idle" 
        self.active_servo_cmd = None
        self.force_zero_motion()
        self.stop_robot_now()

        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MqttRosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
