# This message helps to send one mission or cumulatives

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
float64 yaw
int32 data                  # Can be multiple data, like time to sleep, time to loiter, etc.
