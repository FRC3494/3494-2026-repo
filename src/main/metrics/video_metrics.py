import cv2
import numpy as np

# -------- SETTINGS --------
video_path = "../Downloads/drive-download-20260322T003009Z-3-001/"

# Pixel to monitor (x, y)
px = 1044
py = 67

# Yellow color range in HSV
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([35, 255, 255])

# Slow-motion factor:
# 1.0 = normal speed, 5.0 = video is 5x slower than real life
slowmo_factor = 1
# --------------------------

if slowmo_factor <= 0:
    raise ValueError("slowmo_factor must be > 0")

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise RuntimeError(f"Could not open video: {video_path}")

fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = None  # fallback to timestamp-based second index

counter = 0
was_yellow = False
frame_index = 0

# Per-real-second transition counts
counts_per_second = []
current_second = 0
current_second_count = 0
had_frames = False

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    had_frames = True

    # Video timestamp in seconds
    if fps is not None:
        video_seconds = frame_index / fps
    else:
        video_seconds = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

    # Convert to real-world time (undo slow motion)
    real_seconds = video_seconds / slowmo_factor
    second_index = int(real_seconds)

    # Close out completed second(s)
    if second_index > current_second:
        counts_per_second.append(current_second_count)
        for _ in range(current_second + 1, second_index):
            counts_per_second.append(0)
        current_second = second_index
        current_second_count = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Bounds check
    h, w = hsv.shape[:2]
    if not (0 <= px < w and 0 <= py < h):
        cap.release()
        raise IndexError(f"Pixel ({px},{py}) is out of frame bounds ({w}x{h})")

    pixel = hsv[py, px]
    is_yellow = np.all(pixel >= lower_yellow) and np.all(pixel <= upper_yellow)

    # Count transition into yellow
    if is_yellow and not was_yellow:
        counter += 1
        current_second_count += 1
        print(f"Yellow detected! Count = {counter}")

    was_yellow = is_yellow
    frame_index += 1

cap.release()

# Add final second count
if had_frames:
    counts_per_second.append(current_second_count)

print(f"\nFinal count: {counter}")
print(f"Slow-mo factor applied: {slowmo_factor}x")

# Comma-separated output for Google Sheets / Excel
csv_counts = ",".join(str(x) for x in counts_per_second)
print(f"Per-second yellow transitions (real-time adjusted): [{csv_counts}]")
print(csv_counts)

if counts_per_second:
    avg_per_second = sum(counts_per_second) / len(counts_per_second)
    print(f"Average yellow transitions per real second: {avg_per_second:.4f}")