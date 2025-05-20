import time
import serial
import cv2
from vision_pkg import detectAruco, detectBottle, detectMallet, to_obj
from nav_pkg import forward, reverse, stop, left_turn, right_turn, return_to_origin, turn_90
from config.config import Config


conf = Config()
t = 2

serial = serial.Serial(port=conf.SERIAL_PORT,baudrate=conf.SERIAL_BAUD)

def spiral(target_gps):
    cap = cv2.VideoCapture(conf.CAM_PORT)
    for i in range(20):
        forward()
        # for i in range(t):
        #     ret, frame = cap.read()
        #     if detectAruco(frame) is not None:
        #         stop()
        #         to_obj()
        #         #store aruco
        #         return_to_origin(target_gps)
        #         break
        #     if detectBottle() is not None:
        #         stop()
        #         to_obj()
        #         # Log GPS of Bottle
        #         return_to_origin(target_gps)
        #     if detectMallet() is not None:
        #         stop()
        #         to_obj()
        #         # Log GPS of Mallet
        #         return_to_origin(target_gps)
        #     time.sleep(1)
        if i%2:
            t+=1
        turn_90()
