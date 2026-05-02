/*
 * utils.h
 *
 *  Created on: 9 Apr 2026
 *      Author: jochman03
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

static inline int16_t abs_int16(int16_t value) {
	return (value > 0 ? value : -value);
}

static inline int8_t abs_int8(int8_t value) {
	return (value > 0 ? value : -value);
}

static inline int32_t abs_int32(int32_t value) {
	return (value > 0 ? value : -value);
}

static inline int16_t clamp_int16(int16_t val, int16_t min, int16_t max) {
	int temp = val;
	if (temp < min) {
		temp = min;
	} else if (temp > max) {
		temp = max;
	}
	return temp;
}

static inline int32_t clamp_int32(int32_t val, int32_t min, int32_t max) {
	int temp = val;
	if (temp < min) {
		temp = min;
	} else if (temp > max) {
		temp = max;
	}
	return temp;
}

static inline int8_t clamp_int8(int8_t val, int8_t min, int8_t max) {
	int temp = val;
	if (temp < min) {
		temp = min;
	} else if (temp > max) {
		temp = max;
	}
	return temp;
}

#endif /* INC_UTILS_H_ */
