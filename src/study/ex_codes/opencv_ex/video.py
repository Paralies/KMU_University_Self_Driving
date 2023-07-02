import cv2

# capture = cv2.VideoCapture(0)
capture = cv2.VideoCapture('small.avi')

while True:
    ret, frame = capture.read()
    if not ret:
        break

    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('video', frame)
    cv2.waitKey(10)

capture.release()
cv2.destroyAllWindows()