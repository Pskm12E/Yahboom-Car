# Python_to_ros_bridge [V6_Cached Docker ID]

import paho.mqtt.client as mqtt
import subprocess
import time

BROKER_IP = "localhost"
TOPIC = "yahboom/cmd"

DOCKER_IMAGE = "yahboomtechnology/ros-humble:4.1.2"
ROS_DOMAIN_ID = "20"

LINEAR_SPEED = 2.0
ANGULAR_SPEED = 4.0

YAHBOOM_CONTAINER_ID = None


def get_yahboom_container():
    """
    Finds the running Yahboom ROS container using the Docker image name.
    This should only run once at startup.
    """
    result = subprocess.run(
        [
            "docker", "ps",
            "--filter", f"ancestor={DOCKER_IMAGE}",
            "--format", "{{.ID}}"
        ],
        capture_output=True,
        text=True
    )

    container_ids = result.stdout.strip().splitlines()

    if len(container_ids) == 0:
        raise RuntimeError(
            f"No running container found for image: {DOCKER_IMAGE}. "
            "Please start the Yahboom ROS container first."
        )

    return container_ids[0]


def run_in_docker(command, wait=True):
    """
    Runs a bash command inside the cached Yahboom ROS container.
    """
    global YAHBOOM_CONTAINER_ID

    if YAHBOOM_CONTAINER_ID is None:
        YAHBOOM_CONTAINER_ID = get_yahboom_container()

    full_cmd = [
        "docker", "exec", YAHBOOM_CONTAINER_ID,
        "bash", "-c", command
    ]

    if wait:
        return subprocess.run(full_cmd)
    else:
        return subprocess.Popen(full_cmd)


def stop_existing_ros_publishers():
    """
    Stop old ros2 topic pub commands so multiple publishers do not fight each other.
    """
    try:
        run_in_docker(
            "pkill -f 'ros2 topic pub.*cmd_vel' || true",
            wait=True
        )
    except Exception as e:
        print("Warning: could not stop old ROS publishers:", e)


def start_ros_publisher(linear_x, angular_z):
    """
    Start continuous publishing to /cmd_vel.
    """
    stop_existing_ros_publishers()

    ros_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        "ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "
        f"'{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, "
        f"angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}'"
    )

    run_in_docker(ros_cmd, wait=False)


def send_stop():
    """
    Stop the robot immediately.
    """
    stop_existing_ros_publishers()

    ros_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        "'{linear: {x: 0.0, y: 0.0, z: 0.0}, "
        "angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    )

    run_in_docker(ros_cmd, wait=True)


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
        if command == "forward":
            start_ros_publisher(LINEAR_SPEED, 0.0)

        elif command == "backward":
            start_ros_publisher(-LINEAR_SPEED, 0.0)

        elif command == "left":
            start_ros_publisher(0.0, ANGULAR_SPEED)

        elif command == "right":
            start_ros_publisher(0.0, -ANGULAR_SPEED)

        elif command == "stop":
            send_stop()

        else:
            print("Unknown command:", command)

    except Exception as e:
        print("Error:", e)


def main():
    global YAHBOOM_CONTAINER_ID

    print("Python to Docker ROS bridge started...")
    print(f"Docker image: {DOCKER_IMAGE}")
    print(f"ROS_DOMAIN_ID={ROS_DOMAIN_ID}")

    while True:
        try:
            YAHBOOM_CONTAINER_ID = get_yahboom_container()
            print(f"Cached Yahboom ROS container: {YAHBOOM_CONTAINER_ID}")
            break
        except Exception as e:
            print(e)
            print("Waiting for Docker container...")
            time.sleep(3)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to MQTT broker at {BROKER_IP}:1883...")
    client.connect(BROKER_IP, 1883, 60)

    client.loop_forever()


if __name__ == "__main__":
    main()


