import os
import cv2
import numpy as np
import tensorflow as tf
import time
import shutil

# ========== CONFIGURATION ==========
CONFIDENCE_THRESHOLD = 0.5       # Face detection confidence
RECOGNITION_THRESHOLD = 0.6      # Cosine similarity to match known faces
UNKNOWN_SIMILARITY_THRESHOLD = 0.7  # How different a new unknown must be
UNKNOWN_COOLDOWN = 30.0          # Seconds before allowing repeat screenshot

# ========== PATH SETUP ==========
MODEL_DIR = "models"
KNOWN_DIR = "images/known_faces"
UNKNOWN_DIR = "images/unknown_faces"
EMB_KNOWN_DIR = "embeddings/known"
EMB_UNKNOWN_PATH = "embeddings/unknown/unknown.npy"
FACENET_MODEL = os.path.join(MODEL_DIR, "facenet.tflite")
DNN_PROTO = os.path.join(MODEL_DIR, "dnn/deploy.prototxt")
DNN_MODEL = os.path.join(MODEL_DIR, "dnn/res10_300x300_ssd_iter_140000.caffemodel")

# ========== PREPARE DIRECTORIES ==========
os.makedirs(EMB_KNOWN_DIR, exist_ok=True)
os.makedirs(os.path.dirname(EMB_UNKNOWN_PATH), exist_ok=True)
os.makedirs(UNKNOWN_DIR, exist_ok=True)

# Clear known embeddings
for f in os.listdir(EMB_KNOWN_DIR):
    if f.endswith(".npy"):
        os.remove(os.path.join(EMB_KNOWN_DIR, f))

# Clear unknown embedding
if os.path.exists(EMB_UNKNOWN_PATH):
    os.remove(EMB_UNKNOWN_PATH)

# ========== MODEL LOAD ==========
net = cv2.dnn.readNetFromCaffe(DNN_PROTO, DNN_MODEL)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

interpreter = tf.lite.Interpreter(model_path=FACENET_MODEL, num_threads=4)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# ========== FUNCTIONS ==========
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

def save_unknown(face_img, frame):
    # Save full frame without overlays
    cv2.imwrite(os.path.join(UNKNOWN_DIR, "unknown.jpg"), frame)

    # Save new embedding
    np.save(EMB_UNKNOWN_PATH, face_img)

# ========== LOAD KNOWN ==========
known_embeddings = []
known_names = []

for file in os.listdir(KNOWN_DIR):
    path = os.path.join(KNOWN_DIR, file)
    img = cv2.imread(path)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    emb = get_embedding(rgb)
    known_embeddings.append(emb)
    name = os.path.splitext(file)[0]
    known_names.append(name)
    np.save(os.path.join(EMB_KNOWN_DIR, f"{name}.npy"), emb)

# ========== INIT VIDEO ==========
cap = cv2.VideoCapture(0)
last_unknown_time = 0
last_unknown_emb = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104, 177, 123))
    net.setInput(blob)
    detections = net.forward()

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

        best_score, identity = -1, "Unknown"
        for name, known_emb in zip(known_names, known_embeddings):
            score = cosine_similarity(emb, known_emb)
            if score > best_score:
                best_score = score
                identity = name

        if best_score >= RECOGNITION_THRESHOLD:
            label = f"{identity} ({round(best_score, 2)})"
        else:
            label = f"Unknown"

            now = time.time()
            time_ok = (now - last_unknown_time) > UNKNOWN_COOLDOWN
            new_face_ok = True

            if last_unknown_emb is not None:
                sim_to_last = cosine_similarity(emb, last_unknown_emb)
                new_face_ok = sim_to_last < UNKNOWN_SIMILARITY_THRESHOLD

            if time_ok or new_face_ok:
                # Clone frame without overlays
                raw_frame = frame.copy()
                save_unknown(emb, raw_frame)
                last_unknown_emb = emb
                last_unknown_time = now

        # Draw bounding box and label
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Face Recognition", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
