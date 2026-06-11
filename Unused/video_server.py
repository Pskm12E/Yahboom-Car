# flask_video_mqtt.py
# Flask video feed from MQTT frames
# This script does NOT open the camera.

from flask import Flask, Response
import socket
import time
import threading
import paho.mqtt.client as mqtt


app = Flask(__name__)


# =========================
# MQTT SETTINGS
# =========================
BROKER_IP = "localhost"
BROKER_PORT = 1883

TOPIC_CAMERA_FRAME = "yahboom/camera/frame"


# =========================
# FRAME STORAGE
# =========================
latest_frame = None
frame_lock = threading.Lock()


# =========================
# MQTT CALLBACKS
# =========================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected to broker")
        print(f"[MQTT] Subscribing to: {TOPIC_CAMERA_FRAME}")
        client.subscribe(TOPIC_CAMERA_FRAME)
    else:
        print(f"[MQTT] Connection failed. rc={rc}")


def on_message(client, userdata, msg):
    global latest_frame

    if msg.topic == TOPIC_CAMERA_FRAME:
        with frame_lock:
            latest_frame = msg.payload


# =========================
# FLASK VIDEO STREAM
# =========================
def generate_frames():
    while True:
        with frame_lock:
            frame = latest_frame

        if frame is None:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame +
            b"\r\n"
        )

        time.sleep(0.03)


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/")
def index():
    return """
    <html>
        <head>
            <title>MQTT Video Feed</title>
        </head>
        <body>
            <h2>MQTT Video Feed</h2>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    """


# =========================
# GET LOCAL IP
# =========================
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "127.0.0.1"
    finally:
        s.close()

    return local_ip


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    # Start MQTT client
    try:
        mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        mqtt_client = mqtt.Client()

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    print(f"[MQTT] Connecting to broker: {BROKER_IP}:{BROKER_PORT}")
    mqtt_client.connect(BROKER_IP, BROKER_PORT, 60)
    mqtt_client.loop_start()

    local_ip = get_local_ip()

    print(f"[DASHBOARD LINK]: http://{local_ip}:5001/video_feed")
    print(f"[DASHBOARD PAGE]: http://{local_ip}:5001/")
    print("[INFO] This Flask script does NOT open the camera.")
    print(f"[INFO] Waiting for MQTT frames on: {TOPIC_CAMERA_FRAME}")

    app.run(
        host="0.0.0.0",
        port=5001,
        threaded=True,
        debug=False,
        use_reloader=False
    )
