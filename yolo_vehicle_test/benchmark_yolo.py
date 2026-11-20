import cv2
import time
import csv
import os
import sys
import statistics
from collections import Counter
from ultralytics import YOLO

MODEL_PATH = sys.argv[1] if len(sys.argv) > 1 else "yolo26n-seg.pt"

VIDEO_PATH = "benchmark_input.mp4"
CONF_THRESHOLD = 0.25
DEVICE = "cpu"
WARMUP_FRAMES = 10

if not os.path.exists(MODEL_PATH):
    print(f"ERROR: Model not found: {MODEL_PATH}")
    sys.exit(1)

if not os.path.exists(VIDEO_PATH):
    print(f"ERROR: Video not found: {VIDEO_PATH}")
    sys.exit(1)

print("=" * 70)
print("              YOLO CPU HEADLESS BENCHMARK")
print("=" * 70)
print(f"Model       : {MODEL_PATH}")
print(f"Video       : {VIDEO_PATH}")
print(f"Device      : {DEVICE}")
print(f"Confidence  : {CONF_THRESHOLD}")
print("=" * 70)

print("\nLoading model...")
model = YOLO(MODEL_PATH)
print("Model loaded successfully.")

cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print("ERROR: Could not open benchmark video.")
    sys.exit(1)

print(f"\nWarming up for {WARMUP_FRAMES} frames...")

for i in range(WARMUP_FRAMES):
    ret, frame = cap.read()

    if not ret:
        print("ERROR: Could not read warm-up frame.")
        cap.release()
        sys.exit(1)

    model.predict(
        source=frame,
        conf=CONF_THRESHOLD,
        device=DEVICE,
        verbose=False
    )

print("Warm-up complete.")

cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

latencies = []
detection_counts = []
confidence_values = []
class_counts = Counter()
frame_results = []

frame_number = 0
total_detections = 0

benchmark_start = time.perf_counter()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    frame_number += 1

    t0 = time.perf_counter()

    results = model.predict(
        source=frame,
        conf=CONF_THRESHOLD,
        device=DEVICE,
        verbose=False
    )

    t1 = time.perf_counter()

    latency_ms = (t1 - t0) * 1000.0
    latencies.append(latency_ms)

    result = results[0]

    num_objects = 0
    frame_confidences = []

    if result.boxes is not None and len(result.boxes) > 0:
        num_objects = len(result.boxes)

        for cls_id, conf in zip(
            result.boxes.cls.cpu().tolist(),
            result.boxes.conf.cpu().tolist()
        ):
            class_name = model.names[int(cls_id)]

            class_counts[class_name] += 1
            frame_confidences.append(float(conf))
            confidence_values.append(float(conf))

    total_detections += num_objects
    detection_counts.append(num_objects)

    avg_frame_conf = (
        sum(frame_confidences) / len(frame_confidences)
        if frame_confidences
        else 0.0
    )

    frame_results.append([
        frame_number,
        latency_ms,
        num_objects,
        avg_frame_conf
    ])

    if frame_number % 25 == 0:
        print(
            f"Processed {frame_number:03d} frames | "
            f"Latency = {latency_ms:.2f} ms | "
            f"Objects = {num_objects}"
        )

benchmark_end = time.perf_counter()
cap.release()

total_time = benchmark_end - benchmark_start

mean_latency = statistics.mean(latencies)
median_latency = statistics.median(latencies)

sorted_latencies = sorted(latencies)
p95_index = int(0.95 * len(sorted_latencies)) - 1
p95_index = max(0, min(p95_index, len(sorted_latencies) - 1))
p95_latency = sorted_latencies[p95_index]

fps = 1000.0 / mean_latency
avg_objects = statistics.mean(detection_counts)

avg_conf = (
    statistics.mean(confidence_values)
    if confidence_values
    else 0.0
)

print("\n" + "=" * 70)
print("                    FINAL BENCHMARK RESULTS")
print("=" * 70)

print(f"Model                    : {MODEL_PATH}")
print(f"Frames processed         : {frame_number}")
print(f"Total objects detected  : {total_detections}")
print(f"Average objects/frame   : {avg_objects:.2f}")
print(f"Mean latency             : {mean_latency:.2f} ms")
print(f"Median latency           : {median_latency:.2f} ms")
print(f"P95 latency              : {p95_latency:.2f} ms")
print(f"Average FPS              : {fps:.2f}")
print(f"Average confidence       : {avg_conf:.3f}")
print(f"Total benchmark time     : {total_time:.2f} s")

print("\n" + "-" * 70)
print("CLASS DETECTION COUNTS")
print("-" * 70)

for class_name, count in sorted(
    class_counts.items(),
    key=lambda x: x[1],
    reverse=True
):
    print(f"{class_name:20s} : {count}")

safe_model_name = os.path.splitext(
    os.path.basename(MODEL_PATH)
)[0]

csv_path = f"{safe_model_name}_benchmark.csv"

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "frame",
        "latency_ms",
        "detections",
        "average_confidence"
    ])

    writer.writerows(frame_results)

print("\nPer-frame results saved to:")
print(csv_path)

print("=" * 70)
