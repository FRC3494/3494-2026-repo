import cv2

video_path = "../Downloads/drive-download-20260322T003009Z-3-001/20260321_195312.mp4"

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise RuntimeError(f"Could not open video: {video_path}")

win = "Pixel Picker (space=pause/play, q=quit)"
cv2.namedWindow(win)

state = {"frame": None, "paused": False}

def on_mouse(event, x, y, flags, param):
    frame = state["frame"]
    if frame is None:
        return
    if 0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]:
        b, g, r = frame[y, x]
        if event == cv2.EVENT_MOUSEMOVE:
            print(f"\r(x={x}, y={y})  BGR=({b},{g},{r})", end="")
        elif event == cv2.EVENT_LBUTTONDOWN:
            print(f"\nClicked: x={x}, y={y}, BGR=({b},{g},{r})")

cv2.setMouseCallback(win, on_mouse)

while True:
    if not state["paused"]:
        ret, frame = cap.read()
        if not ret:
            break
        state["frame"] = frame

    if state["frame"] is not None:
        cv2.imshow(win, state["frame"])

    key = cv2.waitKey(20) & 0xFF
    if key == ord("q"):
        break
    elif key == ord(" "):
        state["paused"] = not state["paused"]

cap.release()
cv2.destroyAllWindows()
print()