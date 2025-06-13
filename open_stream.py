#Aleena - Program 1
import cv2
from imutils.video import VideoStream
import imutils
import time

# Replace with your IP camera stream URL
ip_camera_url = "src=rtsp://192.168.144.25:8554/main.264"

print("[INFO] starting video stream...")
vs = VideoStream(src=ip_camera_url).start()
time.sleep(2.0)

while True:
    frame = vs.read()
    if frame is None:
        print("[WARNING] Empty frame received. Check the stream URL.")
        break

    frame = imutils.resize(frame, width=400)
    # Your object detection model inference goes here

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop()
