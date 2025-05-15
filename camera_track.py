import cv2
import numpy as np
import serial
import time

def detect_ball(frame):
    """Detect an orange ball; return position or None if not detected"""
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Orange threshold values that work depending on lighting
    # lower_orange = np.array([5, 130, 100])
    # upper_orange = np.array([15, 255, 255])

    lower_orange = np.array([0, 92, 179])
    upper_orange = np.array([20, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # clean up noise with morph filters
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    # find the largest blob and fit a circle
    c = max(cnts, key=cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c)

    if radius > 10:
        return (int(x), int(y), int(radius))
    return None

def send_message(uart, msg):
    uart.write(f"{msg}\n".encode('utf-8'))

def main():
    # UART setup
    uart = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)
    time.sleep(2)

    # camera setup
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    cv2.namedWindow('Ball Tracker', cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: No frame")
            break

        detection = detect_ball(frame)
        if detection:
            x, y, r = detection
            send_message(uart, f"({x},{y})")
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y),   2, (0,   0, 255), -1)

        cv2.imshow('Ball Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
