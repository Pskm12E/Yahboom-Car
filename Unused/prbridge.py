import atexit
import os
import threading
import time

import paho.mqtt.client as mqtt
import subprocess

BROKER_IP    = "localhost"
TOPIC        = "yahboom/cmd"

# Your Yahboom ROS Docker image name.
# This one does not change, even if the container ID/name changes.
DOCKER_IMAGE = "yahboomtechnology/ros-humble:4.1.2"

ROS_DOMAIN_ID = "20"

# Path to the ROS node script that will be copied into the container once.
_HERE               = os.path.dirname(os.path.abspath(__file__))
NODE_SCRIPT_HOST      = os.path.join(_HERE, "ros_cmd_vel_node.py")
NODE_SCRIPT_CONTAINER = "/tmp/ros_cmd_vel_node.py"

# Long-lived process handle - one per bridge lifetime.
_node_proc: subprocess.Popen | None = None
_node_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Container helpers
# ---------------------------------------------------------------------------

def get_yahboom_container() -> str:
    """
    Finds the running Yahboom ROS container using the Docker image name.
    We do NOT use container name because it changes.
    We do NOT hardcode container ID because it changes.
    """
    result = subprocess.run(
        [
            "docker", "ps",
            "--filter", f"ancestor={DOCKER_IMAGE}",
            "--format", "{{.ID}}",
        ],
        capture_output=True,
        text=True,
    )
    ids = result.stdout.strip().splitlines()
    if not ids:
        raise RuntimeError(
            f"No running container found for image: {DOCKER_IMAGE}. "
            "Please start the Yahboom ROS container first."
        )
    return ids[0]


def _copy_node_script(container_id: str) -> None:
    """Copy ros_cmd_vel_node.py from the host into the container."""
    subprocess.run(
        ["docker", "cp", NODE_SCRIPT_HOST, f"{container_id}:{NODE_SCRIPT_CONTAINER}"],
        check=True,
    )


def _spawn_node(container_id: str) -> subprocess.Popen:
    """
    Launch the long-lived ROS node inside the container.
    docker exec -i keeps stdin connected so we can write commands to it.
    """
    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        f"python3 {NODE_SCRIPT_CONTAINER}"
    )
    return subprocess.Popen(
        ["docker", "exec", "-i", container_id, "bash", "-lc", launch_cmd],
        stdin=subprocess.PIPE,
        text=True,
    )


# ---------------------------------------------------------------------------
# Command dispatch
# ---------------------------------------------------------------------------

def send_command(command: str) -> None:
    """
    Write a command line to the long-lived ROS node's stdin.
    If the node is not running (first call or unexpected exit), start it first.
    Retries once after an automatic restart on pipe errors.
    """
    global _node_proc

    for attempt in range(2):
        with _node_lock:
            if _node_proc is None or _node_proc.poll() is not None:
                container_id = get_yahboom_container()
                _copy_node_script(container_id)
                print(f"[bridge] Starting ROS node in container {container_id}...")
                _node_proc = _spawn_node(container_id)
                # Give rclpy a moment to initialise and register with DDS.
                time.sleep(1.0)
                print("[bridge] ROS node running.")

            try:
                _node_proc.stdin.write(command + "\n")
                _node_proc.stdin.flush()
                return
            except (BrokenPipeError, OSError) as exc:
                print(f"[bridge] Node pipe error ({exc}), restarting...")
                _node_proc = None

    print(f"[bridge] Failed to deliver '{command}' after restart.")

# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------

@atexit.register
def _shutdown() -> None:
    global _node_proc
    with _node_lock:
        if _node_proc and _node_proc.poll() is None:
            print("[bridge] Stopping ROS node...")
            try:
                _node_proc.stdin.write("stop\n")
                _node_proc.stdin.flush()
                _node_proc.stdin.close()
            except Exception:
                pass
            try:
                _node_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                _node_proc.kill()


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker")
        client.subscribe(TOPIC)
        print(f"Listening to MQTT topic: {TOPIC}")
    else:
        print("Failed to connect to MQTT Broker. Return code:", rc)


def on_message(client, userdata, msg):
    command = msg.payload.decode().strip().lower()
    print("Received:", command)

    try:
        match command:
            case "forward" | "backward" | "left" | "right" | "stop":
                send_command(command)
            case _:
                print("Unknown command:", command)
    except Exception as exc:
        print("Error:", exc)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    print("Python to Docker ROS bridge started...")
    print(f"Docker image:   {DOCKER_IMAGE}")
    print(f"ROS_DOMAIN_ID:  {ROS_DOMAIN_ID}")

    while True:
        try:
            container_id = get_yahboom_container()
            print(f"Found running Yahboom ROS container: {container_id}")
            break
        except Exception as e:
            print(e)
            print("Waiting for Docker container...")
            time.sleep(3)

    # Pre-start the node so the first real command has no startup delay.
    send_command("stop")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to MQTT broker at {BROKER_IP}:1883...")
    client.connect(BROKER_IP, 1883, 60)

    client.loop_forever()


if __name__ == "__main__":
    main()
