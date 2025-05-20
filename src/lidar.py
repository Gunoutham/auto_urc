from config.config import Config
from nav_pkg import forward, reverse, stop, left_turn, right_turn, return_to_origin, turn_90
import time


conf = Config()

lidar = conf.LIDAR


def is_obstacle_in_front(scan_data, threshold=conf.OBSTACLE_DIST):
    for angle in range(conf.ARC[0], conf.ARC[1] + 1):
        distance = scan_data.get(angle)
        if distance and distance < threshold:
            return True
    return False


def lidar_data():
    try:
        lidar.start_motor()
        for scan in lidar.iter_scans():
            yield scan

    except Exception as e:
        print(e)

    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()



def obstacle():
    for scan in lidar_data():
        if is_obstacle_in_front(scan):
            print("🚨 Obstacle detected ahead! Taking evasive action...")
            return True
            
        else:
            # print("✅ Path is clear. Moving forward...")
            return False