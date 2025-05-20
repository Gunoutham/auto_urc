from pymavlink import mavutil
import json
from nav import WaypointNavigator
from config.config import Config
import math



class PixHawk():
    
    def __init__(self):
        self.device = '/dev/ttyACM0'

        self.conn = mavutil.mavlink_connection(self.device, baud=57600)
        self.cur_pos = (0,0)
        self.cur_heading = 0
        
        self.conn.wait_heartbeat()

        self.conn.mav.request_data_stream_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    5,  # 5Hz
                    1
                )

    def haversine(self, pos1, pos2):
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def read_gps(self):
        try:
            while True:
                raw_data = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
                if not raw_data:
                    print("No data received, check connection")
                    continue

                if raw_data.get_type() == 'GLOBAL_POSITION_INT':
                    self.cur_pos = (raw_data.lat/1e7, raw_data.lon/1e7)
                    break
        except Exception as e:
            print("Error" + e)
        return self.cur_pos

    def read_compass(self):
        try:
            while True:
                raw_data = self.conn.recv_match(type='VFR_HUD', blocking= True)
                if not raw_data:
                    print("No data received, check connection")
                    continue
                
                if raw_data.get_type() == 'VFR_HUD':
                    self.cur_heading = raw_data.heading
                    break
        except Exception as e:
            print("Error:" + e)
        return self.cur_heading
        

    def greedy_path(self,start_gps):
        json_path = ""

        with open(json_path,"r") as f:
            gps_list = json.load(f)

        gps_list = [(i["latitude"],i["longitude"]) for i in gps_list["gnssPoints"]]


        queue = []

        min = ()
        min_d = float("inf")
        for i in gps_list:
            if self.haversine(start_gps,i)<min_d:
                min = i
                min_d = self.haversine(start_gps,i)

        gps_list.remove(min)
        queue.append(min)

        
        for i in range(6):
            min = ()
            min_d = float("inf")
            for i in gps_list:
                if self.haversine(queue[-1],i)<min_d:
                    min = i
                    min_d = self.haversine(queue[-1],i)
            gps_list.remove(min)

        return queue