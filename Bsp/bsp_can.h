#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include	"STC32G_CAN.h"




extern void CAN_config(void);
extern void can1_send_data(u32 ID,u16 ecd,int16 speed_rpm,int16 current,u16 int_s);
#endif
