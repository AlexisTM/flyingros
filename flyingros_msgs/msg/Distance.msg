# Deprecated, kept for compatibility

uint8 SHUTING_DOWN = 240       # Shutdown the laser to reset it
uint8 NEED_RESET = 48          # Too much outliers, need to reset
uint8 RESET_PENDING = 80       # Wait 15ms after you reset the Lidar, we are waiting in this state
uint8 ACQUISITION_IN_PROGRESS = 64 # The acquisition in on progress

int16[] measures # Data
uint8[] status # Laser status
