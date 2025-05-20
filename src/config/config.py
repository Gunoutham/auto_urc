from rplidar import RPLidar


class Config:
    def __init__(self):
        self.PIX_PORT = ""

        self.SERIAL_PORT = ""
        self.SERIAL_BAUD = 9600

        self.LIDAR_PORT = "/dev/ttyUSB0"
        self.LIDAR_BAUD = 115200

        self.CAM_PORT = ""

        self.LEFT = 30
        self.RIGHT = 30

        self.TURN_LEFT = 10
        self.TURN_RIGHT = 10

        try:
            self.LIDAR = RPLidar(self.LIDAR_PORT,self.LIDAR_BAUD)
        except Exception as e:
            print("Error:",e)
            self.LIDAR = None
            
        self.OBSTACLE_DIST = 1500
        self.ARC = (-30,30)



