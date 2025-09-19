import cv2
import numpy as np


video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

while True:
  ret,frame = video.read()
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  lower_blue = np.array([90, 50, 50], dtype="uint8")
  upper_blue = np.array([130, 255, 255], dtype="uint8")
  mask = cv2.inRange(hsv, lower_blue, upper_blue)

  edges = cv2.Canny(mask, 50, 100)



  #frame = cv2.flip(frame,-1) # used to flip the image vertically
  cv2.imshow('original',edges)
#  cv2.imwrite('original.jpg',frame)

  key = cv2.waitKey(1)
  if key == 27:
     break

video.release()
cv2.destroyAllWindows()