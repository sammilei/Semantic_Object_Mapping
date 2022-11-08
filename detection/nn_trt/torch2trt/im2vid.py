import cv2
import numpy as np
import os

# choose codec according to format needed
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video=cv2.VideoWriter('video.avi', fourcc, 1,(1280,720))

for j in os.listdir("build/images/"):
   if j.endswith('jpg'):
      img = cv2.imread("build/images/" + j)
      video.write(img)

cv2.destroyAllWindows()
video.release()