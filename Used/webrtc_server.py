# webrtc_server.py

import asyncio
import base64
import json
import cv2
import av
import socket
import threading
import time
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

import open_clip
import torch
import numpy as np
from PIL import Image
import paho.mqtt.client as mqtt


# =========================
# SETTINGS
# =========================


CAMERA_INDEX = 0       # Change to 1, 2, 3 if /dev/video0 is not your camera
WIDTH = 640
HEIGHT = 480
FPS = 30
PORT = 8080

BROKER_IP = "localhost"
BROKER_PORT = 1883
# Must match dashboard backend (MQTT_VIT_*_TOPIC in .env).
TOPIC_CLIP = "yahboom/vit/embedding"
TOPIC_STATUS = "yahboom/vit/status"
INFERENCE_EVERY_N_FRAMES = 5
SHOW_PREVIEW = False

# Store active WebRTC connections
pcs = set()


# Shared camera object
camera = None
latest_frame = None
frame_lock = threading.Lock()
stop_event = threading.Event()

model = None
preprocess = None
device = None
mqtt_client = None

# =========================
# CAMERA SETUP
# =========================


def get_local_ip():
    """
    Gets the Raspberry Pi's current local IP address.
    Used only for printing the dashboard link.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except Exception:
        return "127.0.0.1"


def init_camera():
    """
    Opens the camera once.
    This avoids multiple viewers trying to open /dev/video0 again and again.
    """
    global camera

    if camera is not None and camera.isOpened():
        return camera

    print(f"[INFO] Opening camera index {CAMERA_INDEX}...")

    camera = cv2.VideoCapture(CAMERA_INDEX)

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, FPS)

    if not camera.isOpened():
        raise RuntimeError(
            f"Could not open camera index {CAMERA_INDEX}. "
            f"Try changing CAMERA_INDEX to 1, 2, or 3."
        )

    print("[INFO] Camera opened successfully.")
    return camera


def load_model():
    global model, preprocess, device
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, _, preprocess = open_clip.create_model_and_transforms(
        "MobileCLIP-S1",
        pretrained="datacompdr",
        device=device
    )
    model.eval()
    print(f"Loaded MobileCLIP-S1 on {device}")


def create_mqtt_client():
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        client = mqtt.Client()

    def _on_connect(c, _ud, _flags, rc):
        if rc == 0:
            print(f"[MQTT] Connected to {BROKER_IP}:{BROKER_PORT}")
            print(f"[MQTT] Publishing embeddings -> {TOPIC_CLIP}")
            print(f"[MQTT] Publishing status     -> {TOPIC_STATUS}")
        else:
            print(f"[MQTT] Connect failed (rc={rc})")

    client.on_connect = _on_connect
    return client


def publish_status(client, status, extra=None):
    payload = {"status": status, "timestamp": time.time()}
    if extra:
        payload.update(extra)
    try:
        client.publish(TOPIC_STATUS, json.dumps(payload), qos=0)
    except Exception as e:
        print("WARNING: Failed to publish status:", e)


def get_embedding(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(rgb)
    img_tensor = preprocess(pil_img).unsqueeze(0).to(device)
    with torch.no_grad():
        emb = model.encode_image(img_tensor.float())
    emb = emb / emb.norm(dim=-1, keepdim=True)
    return emb.cpu().numpy().astype(np.float32)


def camera_worker():
    global latest_frame
    cap = init_camera()
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        with frame_lock:
            latest_frame = frame.copy()


def vit_worker():
    global mqtt_client
    frame_count = 0
    embedding_count = 0
    while not stop_event.is_set():
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            time.sleep(0.01)
            continue

        frame_count += 1
        if frame_count % INFERENCE_EVERY_N_FRAMES == 0:
            try:
                embedding = get_embedding(frame)
                raw_bytes = embedding.tobytes()
                image_file_size = int(frame.nbytes)
                payload = json.dumps({
                    "raw_bytes": len(raw_bytes),
                    "embedding_dim": int(embedding.shape[-1]),
                    "dtype": "float32",
                    "frame": frame_count,
                    "image_file_size": image_file_size,
                    "data": base64.b64encode(raw_bytes).decode("utf-8"),
                })
                mqtt_client.publish(TOPIC_CLIP, payload, qos=0)
                embedding_count += 1
                if embedding_count % 10 == 0:
                    print(
                        f"Published {embedding_count} embeddings "
                        f"(image {image_file_size} B, embedding {len(raw_bytes)} B)"
                    )
                    publish_status(
                        mqtt_client,
                        "running",
                        {
                            "frames_seen": frame_count,
                            "embeddings_sent": embedding_count,
                            "embedding_shape": list(embedding.shape),
                            "embedding_size_bytes": len(raw_bytes),
                            "dtype": str(embedding.dtype),
                            "topic": TOPIC_CLIP,
                            "image_file_size": image_file_size,
                            "image_payload_size_bytes": image_file_size,
                        },
                    )
            except Exception as e:
                print("ERROR during embedding publish:", e)
                publish_status(
                    mqtt_client,
                    "embedding_error",
                    {"error": str(e), "frame_count": frame_count},
                )

        time.sleep(0.001)


class CameraVideoTrack(VideoStreamTrack):
    """
    Sends camera frames to the browser using WebRTC.
    """

    def __init__(self):
        super().__init__()

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame


# =========================
# WEB PAGE
# =========================


async def index(request):
    html = """
    <!DOCTYPE html>
    <html>
    <head>
      <title>Yahboom WebRTC Video</title>
      <style>
        body { font-family: Arial, sans-serif; background: #111; color: white; text-align: center; margin: 0; padding: 20px; }
        video { width: 90%; max-width: 900px; background: black; border: 3px solid white; border-radius: 12px; }
        button { margin-top: 18px; padding: 12px 24px; font-size: 18px; border-radius: 8px; border: none; cursor: pointer; }
        #status { margin-bottom: 20px; font-size: 18px; color: #ccc; }
      </style>
    </head>
    <body>
      <h1>Yahboom WebRTC Live Video</h1>
      <div id="status">Starting video...</div>
      <video id="video" autoplay playsinline muted></video><br>
      <button onclick="restartVideo()">Restart Video</button>
      <script>
        let pc = null;
        async function startVideo() {
          try {
            document.getElementById("status").innerText = "Creating WebRTC connection...";
            pc = new RTCPeerConnection();
            pc.ontrack = function(event) {
              const video = document.getElementById("video");
              video.srcObject = event.streams[0];
              document.getElementById("status").innerText = "Video connected";
            };
            pc.onconnectionstatechange = function() {
              document.getElementById("status").innerText = "Connection state: " + pc.connectionState;
            };
            pc.addTransceiver("video", { direction: "recvonly" });
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            const response = await fetch("/offer", {
              method: "POST",
              body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
              headers: { "Content-Type": "application/json" }
            });
            if (!response.ok) throw new Error(await response.text());
            const answer = await response.json();
            await pc.setRemoteDescription(answer);
          } catch (error) {
            document.getElementById("status").innerText = "Failed to start video: " + error.message;
          }
        }
        async function restartVideo() {
          if (pc) pc.close();
          pc = null;
          document.getElementById("video").srcObject = null;
          document.getElementById("status").innerText = "Restarting video...";
          await startVideo();
        }
        window.onload = startVideo;
      </script>
    </body>
    </html>
    """
    return web.Response(content_type="text/html", text=html)


# =========================
# WEBRTC OFFER HANDLER
# =========================


async def offer(request):
    try:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                pcs.discard(pc)

        pc.addTrack(CameraVideoTrack())
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp,
                            "type": pc.localDescription.type}),
        )
    except Exception as e:
        return web.Response(status=500, text=str(e))

# =========================
# SHUTDOWN
# =========================


async def on_shutdown(app):
    stop_event.set()
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)
    pcs.clear()
    global camera, mqtt_client
    if camera is not None:
        camera.release()
        camera = None
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


# =========================
# MAIN APP
# =========================


def main():
    global mqtt_client
    load_model()
    mqtt_client = create_mqtt_client()
    try:
        mqtt_client.connect(BROKER_IP, BROKER_PORT, 60)
        mqtt_client.loop_start()
    except Exception as exc:
        print(f"[MQTT] ERROR: could not reach broker at {BROKER_IP}:{BROKER_PORT}: {exc}")
        raise

    publish_status(
        mqtt_client,
        "vit_encoder_started",
        {
            "model": "MobileCLIP-S1",
            "device": str(device),
            "embedding_topic": TOPIC_CLIP,
            "embedding_shape": [1, 512],
            "dtype": "float32",
            "inference_every_n_frames": INFERENCE_EVERY_N_FRAMES,
        },
    )

    threading.Thread(target=camera_worker, daemon=True).start()
    threading.Thread(target=vit_worker, daemon=True).start()

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)

    ip_address = get_local_ip()
    dashboard_url = f"http://{ip_address}:{PORT}"
    print(f"[DASHBOARD LINK] {dashboard_url}")
    web.run_app(app, host="0.0.0.0", port=PORT)


if __name__ == "__main__":
    main()
