#ifndef WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_CAN_DRIVER_H_
#define WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_CAN_DRIVER_H_

#include <stdbool.h>

extern bool CAN_init(const char* can_device);
extern void CAN_deinit();


#endif /* WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_CAN_DRIVER_H_ */
