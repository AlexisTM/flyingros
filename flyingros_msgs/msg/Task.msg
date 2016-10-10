# This message helps to send one mission or cumulatives
# DATA field
# TYPE_TARGET:
#    0 - precisionXY (m)
#    1 - precisionZ (m)
#    2 - precisionYAW (rad)
# TYPE_LOITER:
#    0 - sleep time (s)
# TYPE_TAKEOFF
#    0 - precisionZ (m)
# TYPE_LAND:
#    0 - precisionZ (m)
# TYPE_GRAB:
#    0 - state (0 is closed, 1 is open)
# TYPE_INIT_UAV:
#    0 - sleep (s)
# TYPE_ARM:
#    0 - timeout (s) # retry after timeout
# TYPE_DISARM:
#    0 - timeout (s) # retry after timeout

uint8 TYPE_NULL = 0         # 0b00000000
uint8 TYPE_INIT_UAV = 9     # 0b00001001
uint8 TYPE_ARM = 13         # 0b00001101
uint8 TYPE_DISARM = 11      # 0b00001011
uint8 TYPE_LOITER = 121     # 0b10000001
uint8 TYPE_TAKEOFF = 122    # 0b10000010
uint8 TYPE_LAND = 123       # 0b10000011
uint8 TYPE_TARGET = 124     # 0b10000100
uint8 TYPE_GRAB = 193       # 0b11000001
uint8 TYPE_TEST = 254       # 0b11111110

std_msgs/String name        # Can be unique, you can use it as you please
uint8 mission_type          # Type of the mission
geometry_msgs/Point position
float64 yaw                 # in degrees
float32[] data              # Can be multiple data, like time to sleep, time to loiter, etc.
int64 ID
