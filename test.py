import numpy as np
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()

while True:
    ret, img = cap.read()
    if not ret:
        print("Error: Failed to capture image")
        break

    # Optionally convert to grayscale
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imshow('test', img)

    # Wait for 1 millisecond
    if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
        break

cv2.destroyAllWindows()
cap.release()