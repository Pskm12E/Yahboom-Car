import rclpy
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
TOPIC = "yahboom/cmd"


# =========================
# MANUAL MOVEMENT SETTINGS
# =========================
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0

PUBLISH_RATE = 20.0

LINEAR_STEP = 0.02
ANGULAR_STEP = 0.05


# =========================
# AUTO MOVEMENT SETTINGS
# =========================
RAD2DEG = 180.0 / np.pi

AUTO_LINEAR_SPEED = 0.25
AUTO_TURN_SPEED = 0.65

# Auto should react before lidar_safety_node reaches estop distance.
# Your lidar_safety_node estop distance is 0.35m, so this is larger.
AUTO_FRONT_CLEAR_DISTANCE = 0.70
AUTO_SIDE_CLEAR_DISTANCE = 0.45

FRONT_ANGLE = 25.0
SIDE_ANGLE_MIN = 30.0
SIDE_ANGLE_MAX = 110.0

# Minimum turning time so the robot does not start/stop turning too quickly.
MIN_TURN_TIME = 1.2
FORWARD_COOLDOWN_TIME = 1.0

# If front becomes clear while turning, it can continue forward.
# This makes it turn until it is not blocked.
TURN_UNTIL_CLEAR = True


# =========================
# CAMERA SERVO SETTINGS
# =========================
S1_MIN, S1_MAX = -90, 90
S2_MIN, S2_MAX = -90, 20
CAM_STEP = 10
S_CAM_STEP = 5


MOVEMENT_COMMANDS = {
    "fwd", "fwdright", "fwdleft",
    "bck", "bckright", "bckleft",
    "left", "right", "stop",
}

AUTO_COMMANDS = {
    "auto_on", "auto_off",
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
        # MODE STATE
        # =========================
        self.auto_mode = False
        self.estop_active = False

        # =========================
        # AUTO LIDAR STATE
        # =========================
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.auto_state = "idle"        # idle, forward, turning
        self.turn_direction = "left"    # left or right
        self.turn_start_time = 0.0
        self.forward_start_time = 0.0 

        # =========================
        # SERVO STATE
        # =========================
        self.servo_s1 = 0
        self.servo_s2 = -60
        self.active_servo_cmd = None

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
        self.get_logger().info("Publishing to ROS topics: /cmd_vel, /servo_s1, /servo_s2")
        self.get_logger().info("Reading LiDAR topic: /scan")
        self.get_logger().info("Supported auto commands: auto_on, auto_off")
        self.get_logger().info("Supported estop commands: estop_on, estop_off")

    # =========================
    # MQTT CALLBACKS
    # =========================
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe(TOPIC)
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker. Code: {rc}")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().lower()

        if command != self.last_command:
            self.get_logger().info(f"Received MQTT command: {command}")
            self.last_command = command

        # =========================
        # E-STOP
        # =========================
        if command == "estop_on":
            self.enable_estop()
            return

        elif command == "estop_off":
            self.disable_estop()
            return

        # Block movement and auto while e-stop is active
        if self.estop_active:
            if command in MOVEMENT_COMMANDS or command in AUTO_COMMANDS:
                self.force_zero_motion()
                self.stop_robot_now()
                self.get_logger().warn(
                    f"Command blocked because E-stop is active: {command}"
                )
                return

        # =========================
        # AUTO MODE COMMANDS
        # =========================
        if command == "auto_on":
            self.auto_mode = True
            self.auto_state = "forward"
            self.force_zero_motion()
            self.get_logger().info("AUTO MODE ENABLED")
            return

        elif command == "auto_off":
            self.auto_mode = False
            self.auto_state = "idle"
            self.force_zero_motion()
            self.stop_robot_now()
            self.get_logger().info("AUTO MODE DISABLED")
            return

        # =========================
        # MANUAL MOVEMENT
        # Manual commands cancel auto mode.
        # =========================
        if command == "fwd":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = 0.0

        elif command == "fwdright":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = -ANGULAR_SPEED

        elif command == "fwdleft":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = LINEAR_SPEED
            self.target_angular_z = ANGULAR_SPEED

        elif command == "bck":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = 0.0

        elif command == "bckright":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = ANGULAR_SPEED

        elif command == "bckleft":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = -LINEAR_SPEED
            self.target_angular_z = -ANGULAR_SPEED

        elif command == "left":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = 0.0
            self.target_angular_z = ANGULAR_SPEED

        elif command == "right":
            self.auto_mode = False
            self.auto_state = "idle"
            self.target_linear_x = 0.0
            self.target_angular_z = -ANGULAR_SPEED

        elif command == "stop":
            self.auto_mode = False
            self.auto_state = "idle"
            self.force_zero_motion()
            self.stop_robot_now()

        # =========================
        # CAMERA MOVEMENT
        # Camera movement does not cancel auto mode.
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

    # =========================
    # LIDAR CALLBACK
    # =========================
    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        front_values = []
        left_values = []
        right_values = []

        for i, r in enumerate(ranges):
            if not np.isfinite(r):
                continue

            if not (scan_data.range_min <= r <= scan_data.range_max):
                continue

            angle = (
                scan_data.angle_min + scan_data.angle_increment * i
            ) * RAD2DEG

            # Normalize angle to [-180, 180]
            if angle > 180:
                angle -= 360

            # Front sector
            if abs(angle) <= FRONT_ANGLE:
                front_values.append(r)

            # Left sector
            elif SIDE_ANGLE_MIN <= angle <= SIDE_ANGLE_MAX:
                left_values.append(r)

            # Right sector
            elif -SIDE_ANGLE_MAX <= angle <= -SIDE_ANGLE_MIN:
                right_values.append(r)

        self.front_distance = float(np.min(front_values)) if front_values else None
        self.left_distance = float(np.min(left_values)) if left_values else None
        self.right_distance = float(np.min(right_values)) if right_values else None

    # =========================
    # AUTO MOVEMENT LOGIC
    # =========================
    def auto_movement_logic(self):
        front = self.front_distance
        left  = self.left_distance
        right = self.right_distance

        if front is None:
            self.target_linear_x  = 0.0
            self.target_angular_z = 0.0
            self.get_logger().warn("AUTO MODE: waiting for LiDAR /scan data")
            return

        front_clear = front > AUTO_FRONT_CLEAR_DISTANCE

        # --------------------------------------------------
        # TURNING STATE
        # Direction is LOCKED when turn starts - not re-evaluated mid-turn.
        # --------------------------------------------------
        if self.auto_state == "turning":
            elapsed = time.time() - self.turn_start_time

            # Apply the locked turn direction
            self.target_linear_x = 0.0
            if self.turn_direction == "left":
                self.target_angular_z = AUTO_TURN_SPEED
            else:
                self.target_angular_z = -AUTO_TURN_SPEED

            # Only exit turn after MIN_TURN_TIME AND front is clear
            if elapsed >= MIN_TURN_TIME and front_clear:
                self.auto_state       = "forward"
                self.forward_start_time = time.time()   # start cooldown
                self.target_linear_x  = AUTO_LINEAR_SPEED
                self.target_angular_z = 0.0
                self.get_logger().info(
                    f"AUTO: front clear, resuming forward. "
                    f"Front={front:.2f}m, turned for {elapsed:.1f}s"
                )
            return

        # --------------------------------------------------
        # FORWARD STATE
        # --------------------------------------------------
        if front_clear:
            self.auto_state       = "forward"
            self.target_linear_x  = AUTO_LINEAR_SPEED
            self.target_angular_z = 0.0
            return

        # Front is blocked - only start a new turn if cooldown has passed
        # This prevents immediately re-triggering a turn after just resuming forward
        cooldown_elapsed = time.time() - getattr(self, "forward_start_time", 0.0)
        if cooldown_elapsed < FORWARD_COOLDOWN_TIME:
            # Still in cooldown - keep moving forward briefly before deciding to turn
            self.target_linear_x  = AUTO_LINEAR_SPEED
            self.target_angular_z = 0.0
            return

        # Pick turn direction based on which side has more space - LOCK IT IN
        left_clearance  = left  if left  is not None else 0.0
        right_clearance = right if right is not None else 0.0

        if left_clearance >= right_clearance:
            self.turn_direction = "left"
        else:
            self.turn_direction = "right"

        self.auto_state      = "turning"
        self.turn_start_time = time.time()
        self.target_linear_x  = 0.0

        if self.turn_direction == "left":
            self.target_angular_z = AUTO_TURN_SPEED
        else:
            self.target_angular_z = -AUTO_TURN_SPEED

        self.get_logger().warn(
            f"AUTO: front blocked at {front:.2f}m, "
            f"turning {self.turn_direction}. "
            f"Left={self.format_distance(left)}, Right={self.format_distance(right)}"
        )

    def format_distance(self, value):
        if value is None:
            return "unknown"
        return f"{value:.2f}m"

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

        self.get_logger().warn("EMERGENCY STOP ACTIVE - robot halted")

    def disable_estop(self):
        self.estop_active = False

        # Keep robot stopped after clearing e-stop.
        # User must press a movement command or auto_on again.
        self.auto_mode = False
        self.auto_state = "idle"

        self.force_zero_motion()
        self.stop_robot_now()

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

        if current < target:
            return current + step

        return current - step

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

