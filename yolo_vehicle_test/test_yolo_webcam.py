import cv2
import time
import os
from ultralytics import YOLO


# ============================================================
# YOLO26 CPU WEBCAM TEST
# Vehicle PC - No GPU
# ============================================================

MODEL_PATH = "yolo26s-seg.pt"
CAMERA = "/dev/video0"

CONF_THRESHOLD = 0.25

print("=" * 70)
print("       YOLO26 WEBCAM TEST - VEHICLE PC")
print("=" * 70)

print()
print("Model       :", MODEL_PATH)
print("Camera      :", CAMERA)
print("Device      : CPU")
print("Confidence  :", CONF_THRESHOLD)
print()

# ------------------------------------------------------------
# Load model
# ------------------------------------------------------------

if not os.path.exists(MODEL_PATH):
    print("ERROR: Model file not found:")
    print(MODEL_PATH)
    exit()

print("Loading model...")

model = YOLO(MODEL_PATH)

print("Model loaded successfully.")
print()

# ------------------------------------------------------------
# Open webcam
# ------------------------------------------------------------

cap = cv2.VideoCapture(CAMERA)

if not cap.isOpened():
    print("ERROR: Could not open webcam.")
    exit()

# Use webcam resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Camera opened successfully.")
print()
print("Controls:")
print("  Q = quit")
print()
print("=" * 70)


# ------------------------------------------------------------
# Benchmark variables
# ------------------------------------------------------------

frame_count = 0
total_inference_time = 0.0
total_objects = 0

start_time = time.time()


# ------------------------------------------------------------
# Main loop
# ------------------------------------------------------------

while True:

    ret, frame = cap.read()

    if not ret:
        print("ERROR: Could not read frame.")
        break

    frame_count += 1

    # --------------------------------------------------------
    # YOLO inference
    # --------------------------------------------------------

    inference_start = time.time()

    results = model.predict(
        source=frame,
        conf=CONF_THRESHOLD,
        device="cpu",
        verbose=False
    )

    inference_time = time.time() - inference_start

    total_inference_time += inference_time

    # --------------------------------------------------------
    # Process detections
    # --------------------------------------------------------

    annotated_frame = frame.copy()

    object_count = 0

    for result in results:

        if result.boxes is None:
            continue

        for i in range(len(result.boxes)):

            object_count += 1

            class_id = int(result.boxes.cls[i])
            confidence = float(result.boxes.conf[i])

            class_name = model.names[class_id]

            # Bounding box
            x1, y1, x2, y2 = map(
                int,
                result.boxes.xyxy[i].tolist()
            )

            # Draw bounding box
            cv2.rectangle(
                annotated_frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            # Label
            label = f"{class_name} {confidence:.2f}"

            cv2.putText(
                annotated_frame,
                label,
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            print(
                f"Frame {frame_count:04d} | "
                f"{class_name:<15} | "
                f"confidence = {confidence:.3f}"
            )

    total_objects += object_count

    # --------------------------------------------------------
    # FPS
    # --------------------------------------------------------

    fps = 1.0 / inference_time if inference_time > 0 else 0

    cv2.putText(
        annotated_frame,
        f"Model: YOLO26s-seg",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    cv2.putText(
        annotated_frame,
        f"CPU Inference: {inference_time * 1000:.1f} ms",
        (10, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )

    cv2.putText(
        annotated_frame,
        f"FPS: {fps:.2f}",
        (10, 78),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )

    cv2.putText(
        annotated_frame,
        f"Objects: {object_count}",
        (10, 104),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )

    # --------------------------------------------------------
    # Display
    # --------------------------------------------------------

    cv2.imshow(
        "YOLO26n-seg - Vehicle PC CPU Test",
        annotated_frame
    )

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

cap.release()
cv2.destroyAllWindows()


# ------------------------------------------------------------
# Final statistics
# ------------------------------------------------------------

total_time = time.time() - start_time

average_inference_time = (
    total_inference_time / frame_count
    if frame_count > 0
    else 0
)

average_fps = (
    1.0 / average_inference_time
    if average_inference_time > 0
    else 0
)

average_objects = (
    total_objects / frame_count
    if frame_count > 0
    else 0
)

print()
print("=" * 70)
print("              FINAL CPU TEST RESULTS")
print("=" * 70)

print(f"Model                  : {MODEL_PATH}")
print(f"Frames processed       : {frame_count}")
print(f"Total objects detected : {total_objects}")
print(f"Average objects/frame  : {average_objects:.2f}")
print(
    f"Average inference time : "
    f"{average_inference_time * 1000:.2f} ms"
)
print(f"Average inference FPS  : {average_fps:.2f}")
print(f"Total test time        : {total_time:.2f} s")

print("=" * 70)
