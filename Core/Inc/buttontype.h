/*
 * buttontype.h
 *
 *  Created on: Jun 15, 2023
 *      Author: michael_uman
 */

#ifndef INC_BUTTONTYPE_H_
#define INC_BUTTONTYPE_H_

#include "stm32f4xx_hal.h"
#include "basictypes.h"

typedef struct {
	GPIO_TypeDef * button_port;
	uint16_t button_pin;
	bool button_pressed;
	uint32_t button_press_started;
	uint32_t button_press_ended;
	uint16_t button_timeout;
} sButtonType;


typedef enum {
	BUTTON_RELEASED,
	BUTTON_PRESSED,
	BUTTON_LONGPRESSED,
} eButtonMsg;

/**
 * Button message structure.
 */
typedef struct {
	eButtonMsg button_event_msg;
	uint16_t button_pin;
	uint16_t button_event_ts;
} sButtonEvent;

#endif /* INC_BUTTONTYPE_H_ */
