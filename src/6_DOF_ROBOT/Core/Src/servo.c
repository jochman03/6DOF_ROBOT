#include "servo.h"

static inline uint16_t clampPos(uint16_t pos) {
	if (pos > SERVO_POS_MAX) {
		return SERVO_POS_MAX;
	}
	return pos;
}

uint16_t pos_to_pulse(const servo_t* servo, uint16_t pos) {
	if (servo->reversed) {
		pos = SERVO_POS_MAX - pos;
	}

	return servo->P_MIN
			+ ((uint32_t) pos * (servo->P_MAX - servo->P_MIN)) / SERVO_POS_MAX;
}

void deg_to_pos(int16_t* phi, size_t size) {
	for (uint8_t i = 0; i < size; ++i) {
		phi[i] = (int16_t) (((int32_t) (phi[i] + 90) * 1024) / 180);
	}
}

void pos_to_deg(int16_t* phi, size_t size) {
	for (uint8_t i = 0; i < size; ++i) {
		phi[i] = ((int32_t) phi[i] * 180) / 1024 - 90;
	}
}

void servoInit(servo_t* servo, TIM_HandleTypeDef* htim, uint32_t channel,
		uint16_t pMin, uint16_t pMax, uint16_t startPos, uint8_t reversed) {
	servo->htim = htim;
	servo->channel = channel;
	servo->P_MIN = pMin;
	servo->P_MAX = pMax;
	servo->reversed = reversed;

	startPos = clampPos(startPos);
	servo->current_pos = startPos;
	servo->target_pos = startPos;
	servo->last_ms = 0;

	HAL_TIM_PWM_Start(servo->htim, servo->channel);
	servoMove(servo, startPos);
}

void servoSetTarget(servo_t* servo, uint16_t pos) {
	servo->target_pos = clampPos(pos);
}

void servoMove(servo_t* servo, uint16_t pos) {
	pos = clampPos(pos);
	servo->current_pos = pos;
	servo->target_pos = pos;

	uint16_t pulse = pos_to_pulse(servo, pos);
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

void servoUpdate(servo_t* servo, uint32_t now_ms) {

	if ((now_ms - servo->last_ms) < SERVO_UPDATE_DELAY) {
		return;
	}

	servo->last_ms = now_ms;

	if (servo->current_pos == servo->target_pos) {
		return;
	}

	uint16_t delta =
			(servo->current_pos > servo->target_pos) ?
					(servo->current_pos - servo->target_pos) :
					(servo->target_pos - servo->current_pos);

	uint16_t step = 1 + delta / 10;
	if (step > 5) {
		step = 5;
	}

	if (delta <= step) {
		servo->current_pos = servo->target_pos;
		uint16_t pulse = pos_to_pulse(servo, servo->current_pos);
		__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);

		return;
	} else {
		if (servo->current_pos < servo->target_pos) {
			servo->current_pos += step;
		} else {
			servo->current_pos -= step;
		}
	}
	uint16_t pulse = pos_to_pulse(servo, servo->current_pos);
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

void servoMoveRaw(servo_t* servo, uint16_t pulse) {
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}
