import paho.mqtt.client as mqtt
import subprocess

BROKER_IP = "localhost"
TOPIC = "yahboom/cmd"

DOCKER_IMAGE = "yahboomtechnology/ros-humble:4.1.2"
ROS_DOMAIN_ID = "20"

LINEAR_SPEED = 2.0
ANGULAR_SPEED = 4.0


def get_docker_container():
    result = subprocess.run(
        ["docker", "ps", "-q", "--filter", f"ancestor={DOCKER_IMAGE}"],
        capture_output=True,
        text=True
    )

    container_id = result.stdout.strip().splitlines()

    if not container_id:
        raise RuntimeError(f"No running Docker container found for image: {DOCKER_IMAGE}")

    return container_id[0]


def stop_existing_ros_publishers():
    docker_container = get_docker_container()

    subprocess.run([
        "docker", "exec", docker_container,
        "bash", "-lc",
        "pkill -f 'ros2 topic pub.*cmd_vel' || true"
    ])


def start_ros_publisher(linear_x, angular_z):
    stop_existing_ros_publishers()

    docker_container = get_docker_container()

    ros_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        "ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "
        f"'{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, "
        f"angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}'"
    )

    subprocess.Popen([
        "docker", "exec", docker_container,
        "bash", "-lc", ros_cmd
    ])


def send_stop():
    stop_existing_ros_publishers()

    docker_container = get_docker_container()

    ros_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        "'{linear: {x: 0.0, y: 0.0, z: 0.0}, "
        "angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    )

    subprocess.run([
        "docker", "exec", docker_container,
        "bash", "-lc", ros_cmd
    ])


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker")
    client.subscribe(TOPIC)
    print(f"Listening to MQTT topic: {TOPIC}")


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


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_IP, 1883, 60)

print("MQTT to Docker ROS bridge started...")
print(f"Docker image: {DOCKER_IMAGE}")
print(f"ROS_DOMAIN_ID={ROS_DOMAIN_ID}")

client.loop_forever()
