# robot_sender_mqtt.py - open_clip MobileCLIP-S1 + MQTT


import cv2
import open_clip
import torch
import numpy as np
from PIL import Image
import paho.mqtt.client as mqtt
import time
import json
import sys



# =========================
# MQTT SETTINGS
# =========================
BROKER_IP = "localhost"
BROKER_PORT = 1883


TOPIC_CLIP = "yahboom/vit/embedding"
TOPIC_STATUS = "yahboom/vit/status"


# Run MobileCLIP every N frames
INFERENCE_EVERY_N_FRAMES = 5


# Keep False if using SSH or no monitor
SHOW_PREVIEW = False




# =========================
# CAMERA SETTINGS
# =========================
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480




# =========================
# MQTT HELPER
# =========================
def create_mqtt_client():
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        client = mqtt.Client()


    return client




def publish_status(client, status, extra=None):
    payload = {
        "status": status,
        "timestamp": time.time()
    }


    if extra:
        payload.update(extra)


    try:
        client.publish(TOPIC_STATUS, json.dumps(payload), qos=0)
    except Exception as e:
        print(f"WARNING: Failed to publish status: {e}")




# =========================
# MODEL SETUP
# =========================
def load_model():
    device = "cuda" if torch.cuda.is_available() else "cpu"


    print("Loading MobileCLIP-S1 model...")
    print(f"Using device: {device}")


    model, _, preprocess = open_clip.create_model_and_transforms(
        "MobileCLIP-S1",
        pretrained="datacompdr",
        device=device
    )


    model.eval()


    print("MobileCLIP-S1 loaded successfully.")


    return model, preprocess, device




# =========================
# ENCODER FUNCTION
# =========================
def get_embedding(frame, model, preprocess, device):
    # Convert OpenCV BGR image to RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


    # Convert to PIL image
    pil_img = Image.fromarray(rgb)


    # Preprocess image
    img_tensor = preprocess(pil_img).unsqueeze(0).to(device)


    # Encode image
    with torch.no_grad():
        emb = model.encode_image(img_tensor).float()


        # Normalize embedding
        emb = emb / emb.norm(dim=-1, keepdim=True)


    # Return float32 numpy array with shape [1, 512]
    return emb.cpu().numpy().astype(np.float32)




# =========================
# CAMERA SETUP
# =========================
def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)


    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


    if not cap.isOpened():
        return None


    return cap




# =========================
# MAIN PROGRAM
# =========================
def main():
    mqtt_client = None
    cap = None


    frame_count = 0
    embedding_count = 0


    try:
        # Load model
        model, preprocess, device = load_model()


        # Connect MQTT
        mqtt_client = create_mqtt_client()


        print(f"Connecting to MQTT broker: {BROKER_IP}:{BROKER_PORT}")
        mqtt_client.connect(BROKER_IP, BROKER_PORT, 60)
        mqtt_client.loop_start()


        print("Connected to MQTT broker.")
        print(f"Publishing embeddings to topic: {TOPIC_CLIP}")
        print(f"Publishing status to topic: {TOPIC_STATUS}")


        publish_status(
            mqtt_client,
            "vit_encoder_started",
            {
                "model": "MobileCLIP-S1",
                "device": device,
                "embedding_topic": TOPIC_CLIP,
                "embedding_shape": [1, 512],
                "dtype": "float32",
                "inference_every_n_frames": INFERENCE_EVERY_N_FRAMES
            }
        )


        # Open camera
        print("Opening camera...")
        cap = open_camera()


        if cap is None:
            print("ERROR: Camera could not be opened.")


            publish_status(
                mqtt_client,
                "camera_error",
                {
                    "camera_index": CAMERA_INDEX
                }
            )


            return 1


        print("Camera opened successfully.")
        print("VIT encoder is running. Press CTRL + C to stop.")


        # Main loop
        while True:
            ret, frame = cap.read()


            if not ret:
                print("WARNING: Failed to read camera frame.")
                time.sleep(0.1)
                continue


            frame_count += 1


            if frame_count % INFERENCE_EVERY_N_FRAMES == 0:
                try:
                    embedding = get_embedding(frame, model, preprocess, device)


                    mqtt_client.publish(
                        TOPIC_CLIP,
                        embedding.tobytes(),
                        qos=0
                    )


                    embedding_count += 1


                    if embedding_count % 10 == 0:
                        print(
                            f"Published {embedding_count} embeddings "
                            f"at frame {frame_count}"
                        )


                        publish_status(
                            mqtt_client,
                            "running",
                            {
                                "frames_seen": frame_count,
                                "embeddings_sent": embedding_count,
                                "embedding_shape": list(embedding.shape),
                                "dtype": str(embedding.dtype),
                                "topic": TOPIC_CLIP
                            }
                        )


                except Exception as e:
                    print(f"ERROR during embedding publish: {e}")


                    publish_status(
                        mqtt_client,
                        "embedding_error",
                        {
                            "error": str(e),
                            "frame_count": frame_count
                        }
                    )


            if SHOW_PREVIEW:
                cv2.imshow("Robot Camera Feed", frame)


                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Preview closed by user.")
                    break


    except KeyboardInterrupt:
        print("\nShutting down...")


    except Exception as e:
        print(f"FATAL ERROR: {e}")


        if mqtt_client is not None:
            publish_status(
                mqtt_client,
                "fatal_error",
                {
                    "error": str(e)
                }
            )


        return 1


    finally:
        if cap is not None:
            cap.release()


        if SHOW_PREVIEW:
            cv2.destroyAllWindows()


        if mqtt_client is not None:
            publish_status(
                mqtt_client,
                "vit_encoder_stopped",
                {
                    "frames_seen": frame_count,
                    "embeddings_sent": embedding_count
                }
            )


            mqtt_client.loop_stop()
            mqtt_client.disconnect()


        print("VIT encoder stopped.")


    return 0




if __name__ == "__main__":
    sys.exit(main())





