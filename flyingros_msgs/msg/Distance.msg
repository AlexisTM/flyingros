uint8 NEED_RESET = 48
uint8 RESET_PENDING = 80
uint8 NEED_CONFIGURE = 144
uint8 ACQUISITION_READY = 32
uint8 ACQUISITION_PENDING = 64
uint8 ACQUISITION_DONE = 128
uint8 SHUTING_DOWN = 240

int16[] lasers # data dddd dddd
uint8[] status # outlier ssss nnnn => s = status and n = nack status
# ssss : 
# =(4) under 4 bits
# =(8) under 8 bits : ( nack OR status )
# 0011 xxxx = NEED_RESET =(4) 3 =(8) 48
# 0101 xxxx = RESET_PENDING =(4) 5 =(8) 80
# 1001 xxxx = NEED_CONFIGURE =(4) 9 =(8) 144
# 0010 xxxx = ACQUISITION_READY =(4) 2 =(8) 32
# 0100 xxxx = ACQUISITION_PENDING =(4) 4 =(8) 64
# 1000 xxxx = ACQUISITION_DONE =(4) 8 =(8) 128
