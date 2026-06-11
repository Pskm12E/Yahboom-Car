# webrtc_server.py

import asyncio
import json
import cv2
import av
import socket

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack


# =========================
# SETTINGS
# =========================

CAMERA_INDEX = 0       # Change to 1, 2, 3 if /dev/video0 is not your camera
WIDTH = 640
HEIGHT = 480
FPS = 30
PORT = 8080


# Store active WebRTC connections
pcs = set()

# Shared camera object
camera = None


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


class CameraVideoTrack(VideoStreamTrack):
    """
    Sends camera frames to the browser using WebRTC.
    """

    def __init__(self):
        super().__init__()
        init_camera()

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        global camera

        success, frame = camera.read()

        if not success:
            raise RuntimeError("Failed to read frame from camera")

        # Optional resize to keep stream stable
        frame = cv2.resize(frame, (WIDTH, HEIGHT))

        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame


# =========================
# WEB PAGE
# =========================

async def index(request):
    """
    Web dashboard page.
    Open this from laptop:
    http://<pi-ip>:8080
    """

    html = """
<!DOCTYPE html>
<html>
<head>
    <title>Yahboom WebRTC Video</title>

    <style>
        body {
            font-family: Arial, sans-serif;
            background: #111;
            color: white;
            text-align: center;
            margin: 0;
            padding: 20px;
        }

        h1 {
            margin-bottom: 10px;
        }

        #status {
            margin-bottom: 20px;
            font-size: 18px;
            color: #cccccc;
        }

        video {
            width: 90%;
            max-width: 900px;
            background: black;
            border: 3px solid white;
            border-radius: 12px;
        }

        button {
            margin-top: 18px;
            padding: 12px 24px;
            font-size: 18px;
            border-radius: 8px;
            border: none;
            cursor: pointer;
        }
    </style>
</head>

<body>
    <h1>Yahboom WebRTC Live Video</h1>

    <div id="status">Starting video...</div>

    <video id="video" autoplay playsinline muted></video>

    <br>

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
                    document.getElementById("status").innerText =
                        "Connection state: " + pc.connectionState;
                };

                pc.addTransceiver("video", {
                    direction: "recvonly"
                });

                const offer = await pc.createOffer();
                await pc.setLocalDescription(offer);

                const response = await fetch("/offer", {
                    method: "POST",
                    body: JSON.stringify({
                        sdp: pc.localDescription.sdp,
                        type: pc.localDescription.type
                    }),
                    headers: {
                        "Content-Type": "application/json"
                    }
                });

                if (!response.ok) {
                    const text = await response.text();
                    throw new Error(text);
                }

                const answer = await response.json();

                await pc.setRemoteDescription(answer);

            } catch (error) {
                console.error(error);
                document.getElementById("status").innerText =
                    "Failed to start video: " + error.message;
            }
        }

        async function restartVideo() {
            if (pc) {
                pc.close();
                pc = null;
            }

            const video = document.getElementById("video");
            video.srcObject = null;

            document.getElementById("status").innerText = "Restarting video...";

            await startVideo();
        }

        // Auto-start video when page loads
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
    """
    Browser sends WebRTC offer.
    Pi replies with WebRTC answer.
    """

    try:
        params = await request.json()

        offer = RTCSessionDescription(
            sdp=params["sdp"],
            type=params["type"]
        )

        pc = RTCPeerConnection()
        pcs.add(pc)

        print("[INFO] New WebRTC connection created.")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("[INFO] Connection state:", pc.connectionState)

            if pc.connectionState in ["failed", "closed", "disconnected"]:
                await pc.close()
                pcs.discard(pc)
                print("[INFO] WebRTC connection closed.")

        video_track = CameraVideoTrack()
        pc.addTrack(video_track)

        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })
        )

    except Exception as e:
        print("[ERROR]", str(e))
        return web.Response(
            status=500,
            text=str(e)
        )


# =========================
# SHUTDOWN
# =========================

async def on_shutdown(app):
    """
    Close all WebRTC connections and release camera when server stops.
    """

    print("[INFO] Shutting down WebRTC server...")

    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

    global camera

    if camera is not None:
        camera.release()
        camera = None
        print("[INFO] Camera released.")


# =========================
# MAIN APP
# =========================

app = web.Application()
app.router.add_get("/", index)
app.router.add_post("/offer", offer)
app.on_shutdown.append(on_shutdown)


if __name__ == "__main__":
    ip_address = get_local_ip()

    print("==========================================")
    print(" Yahboom WebRTC Video Server")
    print("==========================================")
    print(f"[DASHBOARD LINK] http://{ip_address}:{PORT}")
    print(f"[CAMERA INDEX] {CAMERA_INDEX}")
    print("==========================================")

    web.run_app(app, host="0.0.0.0", port=PORT)

