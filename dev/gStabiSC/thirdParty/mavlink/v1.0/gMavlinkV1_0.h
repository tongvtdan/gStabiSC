#include "mavlink_types.h"

 void param2Config(uint16_t index);
void getParamsDefault(void);
void getParamsFromFlashAfterPowerOn(void) ;
void mavlinkSend(void);
void handle_mavlink_message(mavlink_channel_t chan, mavlink_message_t* msg);
int  mavlinkReceive(void);

