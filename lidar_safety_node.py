import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import paho.mqtt.client as mqtt
import numpy as np
import time


# =========================
# MQTT SETTINGS
# =========================
BROKER_IP = "localhost"

# Same command topic used by mqtt_ros_node.py
CMD_TOPIC = "yahboom/cmd"

# Topic for dashboard/client feedback
SAFETY_STATUS_TOPIC = "yahboom/safety/status"


# =========================
# LIDAR SAFETY SETTINGS
# =========================
RAD2DEG = 180.0 / np.pi

# Front cone: checks -20 degrees to +20 degrees
FRONT_ANGLE = 20.0

# Warning only distance
WARNING_DISTANCE = 0.60

# Emergency stop / soft stop distance
BLOCK_DISTANCE = 0.35

# Number of LiDAR points needed to confirm obstacle
CONFIRM_POINTS = 8

# Publish status every 0.5 seconds
STATUS_PUBLISH_PERIOD = 0.5

# Prevent spamming estop_on / auto_soft_stop too quickly
SAFETY_COMMAND_COOLDOWN = 0.5

# After estop_off, wait 30 seconds before LiDAR can trigger estop_on again
ESTOP_REARM_DELAY = 30.0


# =========================
# COMMAND GROUPS
# =========================
AUTO_ON_COMMANDS = {
    "auto_on",
}

AUTO_OFF_COMMANDS = {
    "auto_off",
}

# Manual movement commands.
# These do NOT automatically turn auto mode off here,
# because the client may use fwd/left/right during auto/client-decision mode.
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


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__("lidar_safety_node")

        # =========================
        # MODE STATE
        # =========================
        self.auto_mode = False

        # =========================
        # E-STOP REARM STATE
        # =========================
        self.estop_rearm_time = 0.0

        # =========================
        # LIDAR STATE
        # =========================
        self.front_distance = None
        self.front_warning = False
        self.front_blocked = False

        self.last_status_text = ""
        self.last_safety_publish_time = 0.0

        # =========================
        # ROS SUBSCRIBER
        # =========================
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
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

        # Status timer
        self.status_timer = self.create_timer(
            STATUS_PUBLISH_PERIOD,
            self.publish_status
        )

        self.get_logger().info("LiDAR safety node started")
        self.get_logger().info("Reading LiDAR topic: /scan")
        self.get_logger().info(f"Listening to MQTT command topic: {CMD_TOPIC}")
        self.get_logger().info(f"Publishing safety status to MQTT topic: {SAFETY_STATUS_TOPIC}")
        self.get_logger().info(f"Warning distance: {WARNING_DISTANCE}m")
        self.get_logger().info(f"Block distance: {BLOCK_DISTANCE}m")
        self.get_logger().info(f"Manual mode: obstacle triggers estop_on + stop")
        self.get_logger().info(f"Auto mode: obstacle triggers auto_soft_stop only")
        self.get_logger().info(f"After estop_off: LiDAR e-stop re-arms after {ESTOP_REARM_DELAY:.0f}s")

    # =========================
    # MQTT CALLBACKS
    # =========================
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe(CMD_TOPIC)
            self.get_logger().info(f"Subscribed to MQTT topic: {CMD_TOPIC}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker. Code: {rc}")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().lower()

        # =========================
        # AUTO MODE ON
        # =========================
        if command in AUTO_ON_COMMANDS:
            self.auto_mode = True
            self.get_logger().info(
                "LiDAR safety: AUTO MODE ON. Obstacles will cause auto_soft_stop, not estop_on."
            )
            self.publish_mode_status()
            return

        # =========================
        # AUTO MODE OFF
        # =========================
        if command in AUTO_OFF_COMMANDS:
            if self.auto_mode:
                self.get_logger().info("LiDAR safety: AUTO MODE OFF. Manual e-stop behaviour enabled.")
            self.auto_mode = False
            self.publish_mode_status()
            return

        # =========================
        # HARD E-STOP ON
        # If someone else sends estop_on, treat system as manual locked mode.
        # =========================
        if command == "estop_on":
            self.auto_mode = False
            self.get_logger().warn("LiDAR safety: estop_on detected. Auto mode disabled.")
            self.publish_mode_status()
            return

        # =========================
        # HARD E-STOP OFF / RESUME
        # After resume, wait 30s before LiDAR can trigger another e-stop.
        # =========================
        if command == "estop_off":
            self.auto_mode = False
            self.estop_rearm_time = time.time() + ESTOP_REARM_DELAY

            self.get_logger().info(
                f"LiDAR safety: estop_off detected. E-stop will re-arm in {ESTOP_REARM_DELAY:.0f}s."
            )

            self.mqtt_client.publish(
                SAFETY_STATUS_TOPIC,
                f"estop_grace_started,remaining={ESTOP_REARM_DELAY:.1f},estop=false,auto=false"
            )
            return

        # Do not disable auto mode when fwd/left/right is received.
        # In your new design, the client uses these commands during auto/client-decision mode.

    # =========================
    # LIDAR CALLBACK
    # =========================
    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        front_valid_distances = []
        warning_count = 0
        blocked_count = 0

        for i, r in enumerate(ranges):
            # Skip invalid values
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

            # Check only front cone
            if abs(angle) <= FRONT_ANGLE:
                front_valid_distances.append(r)

                if r <= WARNING_DISTANCE:
                    warning_count += 1

                if r <= BLOCK_DISTANCE:
                    blocked_count += 1

        # Store closest front distance
        if len(front_valid_distances) > 0:
            self.front_distance = float(np.min(front_valid_distances))
        else:
            self.front_distance = None

        # Confirm warning / blocked state
        self.front_warning = warning_count >= CONFIRM_POINTS
        self.front_blocked = blocked_count >= CONFIRM_POINTS

        # =========================
        # SAFETY DECISION
        # =========================
        if self.front_blocked:
            if self.auto_mode:
                # Auto/client-decision mode:
                # Stop only. Do NOT latch e-stop.
                self.trigger_auto_soft_stop()
            else:
                # Manual mode:
                # Trigger real e-stop, unless still inside the 30s grace period.
                self.trigger_manual_estop()

    # =========================
    # MANUAL MODE: TRIGGER HARD E-STOP
    # =========================
    def trigger_manual_estop(self):
        now = time.time()
        distance_text = self.get_distance_text()

        # 30-second grace period after estop_off
        if now < self.estop_rearm_time:
            remaining = self.estop_rearm_time - now

            self.mqtt_client.publish(
                SAFETY_STATUS_TOPIC,
                f"estop_grace,distance={distance_text},remaining={remaining:.1f},estop=false,auto=false"
            )

            # Avoid log spam
            if now - self.last_safety_publish_time >= SAFETY_COMMAND_COOLDOWN:
                self.last_safety_publish_time = now
                self.get_logger().warn(
                    f"Obstacle detected, but e-stop grace is active. Rearm in {remaining:.1f}s. Distance: {distance_text}"
                )

            return

        # Avoid spamming estop_on too quickly
        if now - self.last_safety_publish_time < SAFETY_COMMAND_COOLDOWN:
            return

        self.last_safety_publish_time = now

        self.get_logger().warn(
            f"MANUAL MODE: obstacle too close. Triggering E-stop. Distance: {distance_text}"
        )

        # Hard e-stop:
        # mqtt_ros_node.py will latch estop_active=True and block movement commands.
        self.mqtt_client.publish(CMD_TOPIC, "estop_on")

        # Backup stop command
        self.mqtt_client.publish(CMD_TOPIC, "stop")

        # Tell dashboard/client this is a real e-stop.
        self.mqtt_client.publish(
            SAFETY_STATUS_TOPIC,
            f"manual_estop_triggered,distance={distance_text},estop=true,auto=false"
        )

    # =========================
    # AUTO MODE: TRIGGER SOFT STOP ONLY
    # =========================
    def trigger_auto_soft_stop(self):
        now = time.time()
        distance_text = self.get_distance_text()

        # Avoid spamming auto_soft_stop too quickly
        if now - self.last_safety_publish_time < SAFETY_COMMAND_COOLDOWN:
            return

        self.last_safety_publish_time = now

        self.get_logger().warn(
            f"AUTO MODE: obstacle too close. Sending auto_soft_stop only. Distance: {distance_text}"
        )

        # Soft stop:
        # mqtt_ros_node.py should stop the robot but NOT latch estop_active.
        self.mqtt_client.publish(CMD_TOPIC, "auto_soft_stop")

        # Tell dashboard/client this is NOT a hard e-stop.
        self.mqtt_client.publish(
            SAFETY_STATUS_TOPIC,
            f"auto_blocked,distance={distance_text},estop=false,auto=true"
        )

    # =========================
    # STATUS PUBLISHER
    # =========================
    def publish_status(self):
        distance_text = self.get_distance_text()
        now = time.time()

        # If e-stop grace is active, include remaining time.
        grace_remaining = max(0.0, self.estop_rearm_time - now)
        grace_active = grace_remaining > 0.0

        if self.auto_mode:
            if self.front_blocked:
                status = f"auto_blocked,distance={distance_text},estop=false,auto=true"
            elif self.front_warning:
                status = f"auto_warning,distance={distance_text},estop=false,auto=true"
            else:
                status = f"auto_clear,distance={distance_text},estop=false,auto=true"

        else:
            if grace_active:
                if self.front_blocked:
                    status = f"estop_grace,distance={distance_text},remaining={grace_remaining:.1f},estop=false,auto=false"
                elif self.front_warning:
                    status = f"estop_grace_warning,distance={distance_text},remaining={grace_remaining:.1f},estop=false,auto=false"
                else:
                    status = f"estop_grace_clear,distance={distance_text},remaining={grace_remaining:.1f},estop=false,auto=false"

            else:
                if self.front_blocked:
                    status = f"blocked,distance={distance_text},estop=true,auto=false"
                elif self.front_warning:
                    status = f"warning,distance={distance_text},estop=false,auto=false"
                else:
                    status = f"clear,distance={distance_text},estop=false,auto=false"

        self.mqtt_client.publish(SAFETY_STATUS_TOPIC, status)

        # Print only when status changes to reduce spam
        if status != self.last_status_text:
            self.last_status_text = status

            if self.front_blocked or self.front_warning:
                self.get_logger().warn(status)
            else:
                self.get_logger().info(status)

    def publish_mode_status(self):
        mode = "auto" if self.auto_mode else "manual"
        distance_text = self.get_distance_text()

        self.mqtt_client.publish(
            SAFETY_STATUS_TOPIC,
            f"mode={mode},distance={distance_text},estop=false,auto={str(self.auto_mode).lower()}"
        )

    # =========================
    # DISTANCE TEXT
    # =========================
    def get_distance_text(self):
        if self.front_distance is None:
            return "unknown"

        return f"{self.front_distance:.2f}m"

    # =========================
    # CLEAN EXIT
    # =========================
    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LidarSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

