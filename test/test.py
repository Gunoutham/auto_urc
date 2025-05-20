#!/usr/bin/env python3
import math
import time
from typing import Tuple
from pymavlink import mavutil

class StableNavigator:
    def __init__(self, port='/dev/ttyACM0', baud=57600):
        # MAVLink Connection
        self.conn = mavutil.mavlink_connection(port, baud=baud)
        print(f"Connected to Pixhawk on {port}")
        
        # Navigation Parameters
        self.waypoints = [
            (12.9618107, 80.0577110),  # Your target
            # Add more waypoints
        ]
        self.current_wp_index = 0
        self.STOP_RADIUS = 2.0  # meters
        self.STATIONARY_SPEED = 0.3  # m/s
        self.STATIONARY_TIME = 3.0  # seconds
        self.ALIGNMENT_TOLERANCE = 10.0  # degrees
        
        # State Tracking
        self.current_pos = (0.0, 0.0)
        self.current_heading = 0.0
        self.ground_speed = 0.0
        self.last_movement_time = time.time()
        
        self._setup_mavlink_streams()

    def _setup_mavlink_streams(self):
        """Configure MAVLink message rates"""
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10Hz
            1
        )
        time.sleep(1)

    def _update_telemetry(self):
        """Get fresh position, heading, and speed"""
        while True:
            msg = self.conn.recv_match(type=['GLOBAL_POSITION_INT','VFR_HUD'], blocking=True)
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                self.current_pos = (msg.lat/1e7, msg.lon/1e7)
            elif msg.get_type() == 'VFR_HUD':
                self.current_heading = msg.heading
                self.ground_speed = msg.groundspeed
                break

    def _is_stationary(self):
        """Check if actually stopped using speed + position"""
        if self.ground_speed > self.STATIONARY_SPEED:
            return False
            
        # Verify with position history
        pos1 = self.current_pos
        time.sleep(0.5)
        self._update_telemetry()
        pos2 = self.current_pos
        distance = self._haversine(pos1, pos2)
        return distance < 0.5  # Less than 0.5m drift

    def _execute_movement_cycle(self, target):
        """Movement with velocity-based verification"""
        while True:
            self._update_telemetry()
            
            # Check distance using Haversine
            distance = self._haversine(self.current_pos, target)
            
            # True stop condition
            if distance <= self.STOP_RADIUS and self._is_stationary():
                print(f"\nStable arrival at waypoint {self.current_wp_index+1}")
                return True
                
            # Only report movement when actually moving
            if self.ground_speed > self.STATIONARY_SPEED:
                print(f"Moving: {distance:.2f}m | Speed: {self.ground_speed:.1f}m/s | Heading: {self.current_heading:.1f}°", end='\r')
            else:
                print(f"Stationary | GPS Drift: {distance:.2f}m | Waiting for movement...", end='\r')
            
            # Navigation logic here (turns/forward movement)
            # ...

    def _haversine(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate distance in meters"""
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def navigate(self):
        """Main navigation loop"""
        try:
            while self.current_wp_index < len(self.waypoints):
                target = self.waypoints[self.current_wp_index]
                print(f"\n=== Waypoint {self.current_wp_index+1} ===")
                print(f"Target: {target}")
                
                if self._execute_movement_cycle(target):
                    self.current_wp_index += 1
            
            print("\nMission complete!")
            
        except KeyboardInterrupt:
            print("\nEmergency stop!")
        finally:
            print("System shutdown")

# if __name__ == "__main__":
#     nav = StableNavigator(port='/dev/ttyACM0')
#     nav.navigate()
