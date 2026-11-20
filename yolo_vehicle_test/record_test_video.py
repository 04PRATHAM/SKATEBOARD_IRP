import cv2
import time

CAMERA = "/dev/video0"
OUTPUT = "benchmark_input.mp4"

WIDTH = 640
HEIGHT = 480
FPS = 30
NUM_FRAMES = 300

cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    exit(1)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(
    OUTPUT,
    fourcc,
    FPS,
    (WIDTH, HEIGHT)
)

if not writer.isOpened():
    print("ERROR: Could not create video file.")
    cap.release()
    exit(1)

print("=" * 60)
print("       RECORDING FIXED YOLO BENCHMARK VIDEO")
print("=" * 60)
print(f"Camera : {CAMERA}")
print(f"Output : {OUTPUT}")
print(f"Frames : {NUM_FRAMES}")
print(f"Size   : {WIDTH}x{HEIGHT}")
print("=" * 60)

count = 0
start = time.perf_counter()

while count < NUM_FRAMES:
    ret, frame = cap.read()

    if not ret:
        print("WARNING: Could not read frame.")
        continue

    writer.write(frame)
    count += 1

    if count % 25 == 0:
        print(f"Recorded {count}/{NUM_FRAMES} frames")

writer.release()
cap.release()

elapsed = time.perf_counter() - start

print("=" * 60)
print("RECORDING COMPLETE")
print("=" * 60)
print(f"Frames recorded : {count}")
print(f"Time            : {elapsed:.2f} s")
print(f"Saved as        : {OUTPUT}")
print("=" * 60)
