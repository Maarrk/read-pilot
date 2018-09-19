"""
Globals for all drone scripts
"""

# Config
args = None

# UAV params
speed = 5
alt = 5
wpt_radius = 2

# Camera related globals
camera_images_path = "camera_images/"
camera_calibration_path = "ue4_preview800x600.yaml"

# Threads
pilot = None
camera = None

# Logs driectory
logs_dir = "logs/"

# Flight stage events
takeoff_event = None

# pixhawk settings
pix_connection_string = "/dev/ttyS0"
pix_connection_baud = 115200

# sitl settings
sitl_connection_string = "udp:localhost:10000"
sitl_connection_baud = 115200

# UTM zone
zone_no = 34
zone_lt = 'U'
