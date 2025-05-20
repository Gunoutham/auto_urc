import time
import serial
from data_pkg import PixHawk
from nav import WaypointNavigator
from config.config import Config


nav = WaypointNavigator()
hawk = PixHawk()
conf = Config()


serial = serial.Serial(port=conf.SERIAL_PORT,baudrate=conf.SERIAL_BAUD)


def forward():
    # serial.write([conf.LEFT,conf.RIGHT])
    print("FORWARD!")


def left_turn():
    # serial.write([-1*conf.TURN_LEFT,conf.TURN_RIGHT])
    print("LEFT!")


def right_turn():
    # serial.write([conf.TURN_LEFT,-1*conf.TURN_RIGHT])
    print("RIGHT!")


def reverse():
    # serial.write([-1*conf.LEFT,-1*conf.RIGHT])
    print("BACK!")


def stop():
    # for i in range(conf.LEFT,5,-3):
    #     serial.write([i,i])
    #     time.sleep(0.1)
    # serial.write([0,0])
    print("STOP!")



def return_to_origin(target_gps):
    current_gps = hawk.read_gps()
    current_compass = hawk.read_compass()

    nav.target = target_gps

    nav.get_navigation_guidance()  # P2P navigation


def turn_90():
    current_heading = hawk.read_compass()

    target_heading = (int(current_heading)+90)%360

    while abs(current_heading-target_heading)<=1 or abs(current_heading-target_heading)>=359:
        
        break

    left_turn()