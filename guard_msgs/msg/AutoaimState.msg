#This message contains auto aiming state

std_msgs/Header header

uint8 autoaim_state
#auto aiming priority target
uint8 target_id

uint8 TARGET_ID0_NONE              = 0
uint8 TARGET_ID1_HERO              = 1
uint8 TARGET_ID2_ENGINEEER         = 2
uint8 TARGET_ID3_INFANTRY          = 3
uint8 TARGET_ID4_INFANTRY          = 4
uint8 TARGET_ID5_INFANTRY          = 5
uint8 TARGET_ID6_OUTPOST           = 6
uint8 TARGET_ID7_SENTRY            = 7