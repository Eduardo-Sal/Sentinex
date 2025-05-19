import eventlet
eventlet.monkey_patch()

import os
import cv2
import base64
import numpy as np
import tensorflow as tf
from datetime import datetime
from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS
from pathlib import Path
import time
import requests
import random

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet", logger=True, engineio_logger=True)

SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_DIR = SCRIPT_DIR / "models"
FACENET_MODEL = MODEL_DIR / "facenet.tflite"
DNN_PROTO = MODEL_DIR / "dnn/deploy.prototxt"
DNN_MODEL = MODEL_DIR / "dnn/res10_300x300_ssd_iter_140000.caffemodel"
YOLO_CFG = MODEL_DIR / "yolov4-tiny.cfg"
YOLO_WEIGHTS = MODEL_DIR / "yolov4-tiny.weights"
KNOWN_DIR = SCRIPT_DIR / "images/face_database"
EMB_KNOWN_DIR = SCRIPT_DIR / "embeddings/known"

CONFIDENCE_THRESHOLD = 0.5
RECOGNITION_THRESHOLD = 0.6
NOTIFICATION_COOLDOWN = 15.0  # seconds

yolo_net = cv2.dnn.readNetFromDarknet(str(YOLO_CFG), str(YOLO_WEIGHTS))
yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

face_net = cv2.dnn.readNetFromCaffe(str(DNN_PROTO), str(DNN_MODEL))
face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

interpreter = tf.lite.Interpreter(model_path=str(FACENET_MODEL), num_threads=4)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

known_embeddings = []
known_names = []
last_notification_times = {}  # identity -> timestamp

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

def load_known_faces():
    known_embeddings.clear()
    known_names.clear()
    for file in KNOWN_DIR.glob("*.jpg"):
        img = cv2.imread(str(file))
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        emb = get_embedding(rgb)
        known_embeddings.append(emb)
        known_names.append(file.stem)

load_known_faces()

def video_stream():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("? Could not open camera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        yolo_net.setInput(blob)
        ln = yolo_net.getLayerNames()
        output_layers = [ln[i - 1] for i in yolo_net.getUnconnectedOutLayers().flatten()]
        detections = yolo_net.forward(output_layers)

        best_box = None
        best_conf = 0
        for output in detections:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if class_id == 0 and confidence > 0.5:
                    cx = int(detection[0] * frame.shape[1])
                    cy = int(detection[1] * frame.shape[0])
                    w = int(detection[2] * frame.shape[1])
                    h = int(detection[3] * frame.shape[0])
                    x = max(0, int(cx - w / 2))
                    y = max(0, int(cy - h / 2))
                    if confidence > best_conf:
                        best_conf = confidence
                        best_box = (x, y, min(x + w, frame.shape[1]), min(y + h, frame.shape[0]))

        if best_box:
            x1, y1, x2, y2 = best_box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                blob = cv2.dnn.blobFromImage(roi, 1.0, (300, 300), (104, 177, 123))
                face_net.setInput(blob)
                faces = face_net.forward()

                h, w = roi.shape[:2]
                best_face = None
                best_face_conf = 0.0

                for i in range(faces.shape[2]):
                    conf = faces[0, 0, i, 2]
                    if conf > CONFIDENCE_THRESHOLD and conf > best_face_conf:
                        fx1, fy1, fx2, fy2 = (faces[0, 0, i, 3:7] * np.array([w, h, w, h])).astype(int)
                        fx1, fy1 = max(0, fx1), max(0, fy1)
                        fx2, fy2 = min(w, fx2), min(h, fy2)
                        if (fx2 - fx1) > 0 and (fy2 - fy1) > 0:
                            best_face = (fx1, fy1, fx2, fy2)
                            best_face_conf = conf

                if best_face:
                    fx1, fy1, fx2, fy2 = best_face
                    face = roi[fy1:fy2, fx1:fx2]
                    if face.size != 0:
                        rgb_face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
                        emb = get_embedding(rgb_face)

                        best_score = -1
                        best_match = "Unknown"
                        for name, known_emb in zip(known_names, known_embeddings):
                            score = cosine_similarity(emb, known_emb)
                            if score > best_score:
                                best_score = score
                                best_match = name

                        identity = best_match if best_score >= RECOGNITION_THRESHOLD else "Unknown"
                        abs_x1 = x1 + fx1
                        abs_y1 = y1 + fy1
                        abs_x2 = x1 + fx2
                        abs_y2 = y1 + fy2
                        cv2.rectangle(frame, (abs_x1, abs_y1), (abs_x2, abs_y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"{identity} ({best_score:.2f})", (abs_x1, abs_y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                        # ========== NOTIFICATION COOLDOWN CHECK ==========
                        now = time.time()
                        last_time = last_notification_times.get(identity, 0)
                        if now - last_time > NOTIFICATION_COOLDOWN:
                            last_notification_times[identity] = now
                            try:
                                _, buffer = cv2.imencode('.jpg', frame)
                                image_bytes = buffer.tobytes()
                                filename = f"image{random.randint(1, 1_000_000)}.jpg"

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

        ts = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, ts, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            jpg_b64 = base64.b64encode(buf).decode("utf-8")
            socketio.emit("frame", jpg_b64)

        socketio.sleep(0.03)

    cap.release()

def main():
    socketio.start_background_task(video_stream)
    socketio.run(app, host="0.0.0.0", port=5001)

if __name__ == "__main__":
    main()
