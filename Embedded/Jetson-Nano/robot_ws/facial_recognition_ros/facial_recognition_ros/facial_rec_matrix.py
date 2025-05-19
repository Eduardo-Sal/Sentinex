import os
import cv2
import numpy as np
import tensorflow as tf
import warnings
import readline
warnings.filterwarnings("ignore")

import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)

# ========== CONFIGURATION ==========
CONFIDENCE_THRESHOLD = 0.5
RECOGNITION_THRESHOLD = 0.6

# ========== PATHS ==========
MODEL_DIR = "models"
DNN_PROTO = os.path.join(MODEL_DIR, "dnn", "deploy.prototxt")
DNN_MODEL = os.path.join(MODEL_DIR, "dnn", "res10_300x300_ssd_iter_140000.caffemodel")
FACENET_MODEL = os.path.join(MODEL_DIR, "facenet.tflite")
KNOWN_DIR = "images/known_faces"
EMBEDDINGS_DIR = "embeddings"

# ========== LOAD MODELS ==========
net = cv2.dnn.readNetFromCaffe(DNN_PROTO, DNN_MODEL)
interpreter = tf.lite.Interpreter(model_path=FACENET_MODEL, num_threads=4)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

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

# ========== REBUILD EMBEDDINGS ==========
if not os.path.exists(EMBEDDINGS_DIR):
    os.makedirs(EMBEDDINGS_DIR)

for file in os.listdir(EMBEDDINGS_DIR):
    if file.endswith(".npy"):
        os.remove(os.path.join(EMBEDDINGS_DIR, file))

known_embeddings, known_names = [], []
for file in os.listdir(KNOWN_DIR):
    if file.lower().endswith((".jpg", ".jpeg", ".png")):
        path = os.path.join(KNOWN_DIR, file)
        img = cv2.imread(path)
        if img is None:
            continue
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        emb = get_embedding(rgb)

        name = os.path.splitext(file)[0]
        np.save(os.path.join(EMBEDDINGS_DIR, f"{name}.npy"), emb)
        known_embeddings.append(emb)
        known_names.append(name)

# ========== USER TERMINAL WALKTHROUGH ==========
print("\n========= FACE VERIFICATION WALKTHROUGH =========")
print("This test checks whether the system correctly identifies ONE specific person")
print("and correctly rejects anyone else.\n")
print("Step 1: Choose the person to test (the 'target')")
print(f"Known people in system: {', '.join(known_names)}")
target_name = input("Enter the target identity (case-sensitive): ").strip()
while target_name not in known_names:
    target_name = input(f"'{target_name}' not found. Try again: ").strip()

target_index = known_names.index(target_name)
target_embedding = known_embeddings[target_index]

print(f"\nTarget set to: {target_name}")
print("\nStep 2: During testing, use the following keys to log ground truth:")
print("""
    [1] Ground Truth = TARGET     (e.g., person in frame IS the target)
    [2] Ground Truth = IMPOSTER   (person in frame is NOT the target)
    [u] Ground Truth = UNKNOWN    (person not in known_faces directory)
    [s] STOP TEST AND PRINT RESULTS
""")

input("Press ENTER to begin the test.\n")

# ========== METRICS ==========
TP = FP = FN = TN = 0
actual_person = "Unknown"
testing_active = True

# ========== VIDEO LOOP ==========
cap = cv2.VideoCapture(0)

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

        embedding = get_embedding(face)
        score = cosine_similarity(embedding, target_embedding)
        predicted = target_name if score >= RECOGNITION_THRESHOLD else "Unknown"

        if testing_active:
            if predicted == target_name and actual_person == target_name:
                TP += 1
            elif predicted == target_name and actual_person != target_name:
                FP += 1
            elif predicted == "Unknown" and actual_person == target_name:
                FN += 1
            elif predicted == "Unknown" and actual_person != target_name:
                TN += 1

        label = f"{predicted} ({round(score, 2)}) | GT: {actual_person}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('1'):
        actual_person = target_name
    elif key == ord('2'):
        actual_person = "Imposter"
    elif key == ord('u'):
        actual_person = "Unknown"
    elif key == ord('s'):
        testing_active = False
        print("\nStopping test... generating report.\n")
        break

    cv2.putText(frame, f"GT: {actual_person}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.imshow("Face Verification", frame)

cap.release()
cv2.destroyAllWindows()

# ========== REPORT ==========
print("========= VERIFICATION REPORT =========")
print(f"Target identity: {target_name}")
print(f"True Positives:  {TP}")
print(f"False Positives: {FP}")
print(f"False Negatives: {FN}")
print(f"True Negatives:  {TN}")

precision = TP / (TP + FP) if (TP + FP) > 0 else 0
recall = TP / (TP + FN) if (TP + FN) > 0 else 0
f1_score = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

print(f"Precision: {precision:.2f}")
print(f"Recall:    {recall:.2f}")
print(f"F1 Score:  {f1_score:.2f}")
