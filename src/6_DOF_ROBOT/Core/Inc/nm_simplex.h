/*
 * nm_simplex.h
 *
 *  Created on: Nov 8, 2025
 *      Author: jochman03
 */

#ifndef NM_SIMPLEX_H
#define NM_SIMPLEX_H

#include <stdint.h>

#define N 4

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define SWAP(a, b) do { \
    int32_t tmp = (a);  \
    (a) = (b);          \
    (b) = tmp;          \
} while (0)

void calculate_position_from_servo(int16_t* phi, int16_t* pos);

int32_t calculate_function_cost(int16_t* phi, int16_t* pos, int16_t* phi_min,
		int16_t* phi_max, int16_t* previous_phi);
int32_t calculate_boundaries_penalty(int16_t* phi, int16_t* phi_min,
		int16_t* phi_max);
int32_t calculate_position_penalty(int16_t* phi, int16_t* target_pos);

void nm_simplex(
		int32_t (*fun)(int16_t*, int16_t*, int16_t*, int16_t*, int16_t*),
		int16_t* x0, uint16_t maxIter, int32_t* f_min, int16_t* x_min,
		int16_t* pos, int16_t* phi_min, int16_t* phi_max);

#endif
