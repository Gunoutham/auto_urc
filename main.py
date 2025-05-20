from src.nav import WaypointNavigator
from src.spiral import spiral
from src.nav_pkg import forward, return_to_origin, reverse, left_turn, right_turn, stop
from src.data_pkg import PixHawk


nav = WaypointNavigator()
pix = PixHawk()

# Enter the start gps values
lat = float(input())
lon = float(input())

start_gps = (lat,lon)

queue = pix.greedy_path(start_gps)


for i in queue:
    nav.target = i
    nav.cur_pos = pix.read_gps()

    nav.get_navigation_guidance()

    spiral(i)

nav.target = start_gps
nav.cur_pos = pix.read_gps()

nav.get_navigation_guidance()










