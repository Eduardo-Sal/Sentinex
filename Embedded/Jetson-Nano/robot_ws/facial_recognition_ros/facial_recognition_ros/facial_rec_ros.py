import os
import cv2
import numpy as np
import tensorflow as tf
import time
import shutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
from pathlib import Path
import requests
import random  # Added for randomized image filenames

# ========== PATH SETUP ==========
SCRIPT_DIR = Path(__file__).resolve().parent

MODEL_DIR = SCRIPT_DIR / "models"
KNOWN_DIR = SCRIPT_DIR / "images/face_database"
DETECTED_DIR = SCRIPT_DIR / "images/detected_faces"
DETECTED_KNOWN_DIR = DETECTED_DIR / "known_faces"
DETECTED_UNKNOWN_DIR = DETECTED_DIR / "unknown_faces"
EMB_KNOWN_DIR = SCRIPT_DIR / "embeddings/known"
EMB_UNKNOWN_PATH = SCRIPT_DIR / "embeddings/unknown/unknown.npy"
FACENET_MODEL = MODEL_DIR / "facenet.tflite"
DNN_PROTO = MODEL_DIR / "dnn/deploy.prototxt"
DNN_MODEL = MODEL_DIR / "dnn/res10_300x300_ssd_iter_140000.caffemodel"

# ========== CONFIGURATION ==========
CONFIDENCE_THRESHOLD = 0.5
RECOGNITION_THRESHOLD = 0.6
UNKNOWN_SIMILARITY_THRESHOLD = 0.7
UNKNOWN_COOLDOWN = 30.0
NOTIFICATION_COOLDOWN = 30.0

# ========== PREPARE DIRECTORIES ==========
for path in [EMB_KNOWN_DIR, EMB_UNKNOWN_PATH.parent, DETECTED_KNOWN_DIR, DETECTED_UNKNOWN_DIR]:
    os.makedirs(path, exist_ok=True)

for f in EMB_KNOWN_DIR.glob("*.npy"):
    f.unlink()

if EMB_UNKNOWN_PATH.exists():
    EMB_UNKNOWN_PATH.unlink()

# ========== MODEL LOAD ==========
net = cv2.dnn.readNetFromCaffe(str(DNN_PROTO), str(DNN_MODEL))
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

interpreter = tf.lite.Interpreter(model_path=str(FACENET_MODEL), num_threads=4)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# ========== GLOBAL REGISTRATION STATE ==========
register_name = None
should_register_face = False
register_lock = threading.Lock()

# ========== ROS NODE ==========
class FaceRegisterNode(Node):
    def __init__(self):
        super().__init__('face_register_node')
        self.subscription = self.create_subscription(
            String,
            '/Camera',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global register_name, should_register_face
        try:
            data = json.loads(msg.data)
            name = data.get("name", "").strip()
            if name and name.lower() != "unknown":
                with register_lock:
                    register_name = name
                    should_register_face = True
        except json.JSONDecodeError:
            self.get_logger().warn("Received malformed JSON")

# ========== FACE RECOGNITION FUNCTIONS ==========
def preprocess_face(img):
    face = cv2.resize(img, (160, 160))
    face = face.astype("float32") / 255.0
    return np.expand_dims(face, axis=0)

def get_embedding(face_img):
    tensor = preprocess_face(face_img)
    interpreter.set_tensor(input_details[0]['index'], tensor)
    interpreter.invoke()
    embedding = interpreter.get_tensor(output_details[0]['index'])[0]
    return embedding / np.linalg.norm(embedding)

def cosine_similarity(a, b):
    return np.dot(a, b)

# ========== LOAD KNOWN EMBEDDINGS ==========
known_embeddings = []
known_names = []

for file in KNOWN_DIR.glob("*.jpg"):
    img = cv2.imread(str(file))
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    emb = get_embedding(rgb)
    known_embeddings.append(emb)
    name = file.stem
    known_names.append(name)
    np.save(str(EMB_KNOWN_DIR / f"{name}.npy"), emb)

# ========== MAIN THREAD ==========
def start_video_loop():
    global should_register_face, register_name

    cap = cv2.VideoCapture(0)
    last_save_times = {}
    last_embeddings = {}
    notification_cooldowns = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104, 177, 123))
        net.setInput(blob)
        detections = net.forward()

        recognized_identities = []

        for i in range(detections.shape[2]):
            conf = detections[0, 0, i, 2]
            if conf < CONFIDENCE_THRESHOLD:
                continue

            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box.astype("int")
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            face = frame[y1:y2, x1:x2]

            if face.size == 0:
                continue

            rgb_face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            emb = get_embedding(rgb_face)

            with register_lock:
                if should_register_face and register_name:
                    cv2.imwrite(str(KNOWN_DIR / f"{register_name}.jpg"), face)
                    np.save(str(EMB_KNOWN_DIR / f"{register_name}.npy"), emb)
                    known_names.append(register_name)
                    known_embeddings.append(emb)
                    if EMB_UNKNOWN_PATH.exists():
                        EMB_UNKNOWN_PATH.unlink()
                    for f in DETECTED_UNKNOWN_DIR.glob("*.jpg"):
                        f.unlink()
                    should_register_face = False
                    register_name = None
                    continue

            best_score = -1
            best_match = "Unknown"
            for name, known_emb in zip(known_names, known_embeddings):
                score = cosine_similarity(emb, known_emb)
                if score > best_score:
                    best_score = score
                    best_match = name

            if best_score >= RECOGNITION_THRESHOLD:
                identity = best_match
                is_known = True
                recognized_identities.append(identity)
            else:
                identity = "Unknown"
                is_known = False

            now = time.time()
            last_time = last_save_times.get(identity, 0)
            time_ok = (now - last_time) > UNKNOWN_COOLDOWN

            new_face_ok = True
            if identity in last_embeddings:
                sim_to_last = cosine_similarity(emb, last_embeddings[identity])
                new_face_ok = sim_to_last < UNKNOWN_SIMILARITY_THRESHOLD

            if time_ok or new_face_ok:
                last_embeddings[identity] = emb
                last_save_times[identity] = now

                # ========== NOTIFICATION COOLDOWN ==========
                last_notify_time = notification_cooldowns.get(identity, 0)
                if (now - last_notify_time) > NOTIFICATION_COOLDOWN:
                    notification_cooldowns[identity] = now

                    try:
                        _, buffer = cv2.imencode('.jpg', frame)
                        image_bytes = buffer.tobytes()

                        random_id = random.randint(1, 1_000_000)
                        filename = f"image{random_id}.jpg"

                        files = {
                            'image': (filename, image_bytes, 'image/jpeg')
                        }

                        data = {
                            'name': identity,
                            'robot_id': '4'
                        }

                        url = "https://api.sentinex.app/api/images/notifications/face"
                        response = requests.post(url, data=data, files=files)

                        if response.status_code == 200:
                            print(f"[DEBUG] Sent notification for {identity} as {filename}", flush=True)
                        else:
                            print(f"[ERROR] Notification failed ({response.status_code}): {response.text}", flush=True)

                    except Exception as e:
                        print(f"[ERROR] HTTP notification failed: {e}", flush=True)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{identity} ({best_score:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

# ========== MAIN ==========
def main():
    rclpy.init()
    node = FaceRegisterNode()
    thread = threading.Thread(target=start_video_loop, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
