#!/usr/bin/env python3
import os
import re
import sys
import cv2

SAVE_DIR = "/home/orangepi/ros_ws/src/yolov11n_rknn/image"
TARGET_W = 640
TARGET_H = 480
MAX_RETRIES = 5


def get_next_index():
    os.makedirs(SAVE_DIR, exist_ok=True)
    max_idx = 0
    pattern = re.compile(r"^(\d+)\.(jpg|jpeg|png)$", re.IGNORECASE)
    for name in os.listdir(SAVE_DIR):
        m = pattern.match(name)
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    return max_idx + 1


def open_camera():
    for dev_idx in range(4):
        print(f"Trying camera index {dev_idx} ...")
        cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)

            for _ in range(MAX_RETRIES):
                ret, frame = cap.read()
                if ret and frame is not None:
                    h, w = frame.shape[:2]
                    print(f"Success: /dev/video{dev_idx} ({w}x{h})")
                    return cap
            cap.release()
    return None


def main():
    idx = get_next_index()
    print(f"Starting index: {idx:04d}")

    cap = open_camera()
    if cap is None:
        print("Error: Could not open any camera.")
        sys.exit(1)

    print("Controls:")
    print("  [Space] or [Enter] = Capture photo")
    print("  [ESC] or [Q]       = Quit")

    fail_count = 0
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            fail_count += 1
            if fail_count > 30:
                print("Error: Too many frame failures, exiting.")
                break
            continue
        fail_count = 0

        preview = frame.copy()
        cv2.putText(
            preview,
            f"Next: {idx:04d}.jpg  |  Space=Capture  |  ESC=Quit",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Dataset Capture", preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            break
        if key in (32, 10, 13):
            fname = os.path.join(SAVE_DIR, f"{idx:04d}.jpg")
            cv2.imwrite(fname, frame)
            print(f"Saved: {fname}")
            idx += 1

    cap.release()
    cv2.destroyAllWindows()
    print("Capture finished.")


if __name__ == "__main__":
    main()
