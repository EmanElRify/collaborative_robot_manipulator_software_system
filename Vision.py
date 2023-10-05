import numpy as np
import cv2
from PIL import ImageTk, Image
import threading

# Capture video from the camera
cap = cv2.VideoCapture(1)
#global last_prediction
#last_prediction = ''
# cm to pixel ratio
cm_to_pixels = 35.0 / 640.0

# Rotation around y-axis by 180 degrees:
Rad_y = (180.0 / 180.0) * np.pi
RY = [[np.cos(Rad_y), 0, np.sin(Rad_y)],
      [0, 1, 0],
      [-np.sin(Rad_y), 0, np.cos(Rad_y)]]
#translation x_base by 13cm & in y_base by 20cm:
dO_C = [[13], [20], [0]]
H0_C = np.concatenate((RY, dO_C), 1)
H0_C = np.concatenate((H0_C, [[0, 0, 0, 1]]), 0)
ik_theta = 0.0

background_set = False

