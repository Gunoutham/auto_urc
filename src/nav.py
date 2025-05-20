import math
import time
from pymavlink import mavutil
from nav_pkg import forward, left_turn, right_turn, reverse, stop
from data_pkg import PixHawk
from config.config import Config
from lidar import obstacle

class WaypointNavigator(PixHawk):
    def __init__(self):
        self.conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        self.target = (12.9618107, 80.0577110)
        self.STOP_RADIUS = 2.0
        self.ALIGN_TOLERANCE = 5
        self.current_pos = (0, 0)
        self.distance = 0
        self.heading = 0

    def haversine(self, pos1, pos2):
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    


    def calculate_bearing(self, pos1, pos2):
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        return (math.degrees(math.atan2(x, y)) + 360) % 360
    


    def get_navigation_guidance(self):
        print("\n=== Navigation Guidance ===")
        print(f"Target: {self.target[0]:.6f}, {self.target[1]:.6f}")
        print("Press CTRL+C to stop\n")
        
        try:
            while True:
                obs = False
                self.current_pos = self.read_gps()
                self.distance = self.haversine(self.current_pos, self.target)
                
                if self.distance <= self.STOP_RADIUS:
                    print("\n\n✅ Reached target! (Within 2m radius)")
                    return

                self.heading = self.read_compass()
                bearing = self.calculate_bearing(self.current_pos, self.target)
                angle_diff = (bearing - self.heading + 180) % 360 - 180
                
                while obstacle():
                    obs = True
                    left_turn()
                    time.sleep(0.5)
                
                if obs:
                    forward()
                    time.sleep(1)

                turn_msg = "On track ➡️"
                forward()
                if angle_diff < -self.ALIGN_TOLERANCE:
                    turn_msg = "Turn LEFT ⬅️"
                    # left_turn()

                    while(angle_diff< -self.ALIGN_TOLERANCE):
                        left_turn()
                        heading = self.read_compass()
                        angle_diff = (bearing - heading + 180) % 360 - 180

                elif angle_diff > self.ALIGN_TOLERANCE:
                    turn_msg = "Turn RIGHT ➡️"
                    # right_turn()

                    while(angle_diff > self.ALIGN_TOLERANCE):
                        right_turn()
                        heading = self.read_compass()
                        angle_diff = (bearing - heading + 180) % 360 - 180

                print(f"\rDistance: {self.distance:5.1f}m | {turn_msg} | Target: {bearing:03.0f}° | Current: {self.heading:03.0f}°", end='', flush=True)

        except KeyboardInterrupt:
            print("\n\n🚫 Navigation stopped")
