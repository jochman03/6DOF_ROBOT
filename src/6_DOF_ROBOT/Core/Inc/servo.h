/*
 * servo.h
 *
 *  Created on: Nov 8, 2025
 *      Author: jochman03
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

#define SERVO_POS_MIN   	0
#define SERVO_POS_MAX   	1024
#define SERVO_POS_MID   	(SERVO_POS_MAX - SERVO_POS_MIN) / 2

#define SERVO_UPDATE_DELAY	20

typedef struct {
	TIM_HandleTypeDef* htim;
	uint32_t channel;
	uint16_t P_MIN;
	uint16_t P_MAX;
	uint16_t current_pos;
	uint16_t target_pos;
	uint32_t last_ms;
	uint8_t reversed;
} servo_t;

void servoInit(servo_t* servo, TIM_HandleTypeDef* htim, uint32_t channel,
		uint16_t pMin, uint16_t pMax, uint16_t startPos, uint8_t reversed);

uint16_t pos_to_pulse(const servo_t* servo, uint16_t pos);

void deg_to_pos(int16_t* phi, size_t size);

void pos_to_deg(int16_t* phi, size_t size);

void servoSetTarget(servo_t* servo, uint16_t pos);

void servoUpdate(servo_t* servo, uint32_t now_ms);

void servoMove(servo_t* servo, uint16_t pos);

void servoMoveRaw(servo_t* servo, uint16_t pulse);

#endif /* INC_SERVO_H_ */
