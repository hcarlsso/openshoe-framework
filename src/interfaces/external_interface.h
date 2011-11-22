
#ifndef EXTERNAL_INTERFACE_H_
#define EXTERNAL_INTERFACE_H_

#include "conf_usb.h"
#include "udd.h"
#include "udc.h"
#include "udi_cdc.h"
#include "usbc_device.h"

#include "nav_types.h"


#define PROCESSING_ARRAY_SIZE 10
typedef void (*processing_function_p)(void);


void com_interface_init(void);

void transmit_data(void);
void receive_command(void);

// USB vbus callback function
void vbus_event_callback(bool b_high);

#endif /* EXTERNAL_INTERFACE_H_ */