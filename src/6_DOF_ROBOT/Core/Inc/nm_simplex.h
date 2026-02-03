/*
 * nm_simplex.h
 *
 * Nelder–Mead simplex optimization implementation
 *
 *  Created on: Dec 11, 2025
 *      Author: Jakub Ochman
 */

#ifndef INC_NM_SIMPLEX_H_
#define INC_NM_SIMPLEX_H_


/**
 * Converts angles from degrees to radians.
 *
 * @param phi_deg Pointer to array of angles in degrees.
 *                Converted in-place to radians.
 * @param size    Number of elements in the array.
 */
void deg_2_rad(float* phi_deg, int size);

/**
 * Objective function for position optimization.
 *
 * @param phi   Pointer to array of joint angles (radians).
 * @param pos   Pointer to target position vector.
 * @param phi0  Pointer to reference / initial angle vector.
 *
 * @return Cost function value.
 */
float pos_f(float* phi, float* pos , float* phi0);

/**
 * Nelder–Mead simplex optimization algorithm.
 *
 * @param fun      Pointer to objective function.
 * @param phi0     Initial position.
 * @param maxIter  Maximum number of iterations.
 * @param f_min    Pointer to minimum function value found.
 * @param x_min    Pointer to optimal solution vector.
 * @param pos      Pointer to target position.
 * @param phi_min  Pointer to lower bounds of angles.
 * @param phi_max  Pointer to upper bounds of angles.
 */
void simplex(float (*fun)(float*, float*, float*),
             float* phi0,
             int maxIter,
             float* f_min,
             float* x_min,
             float* pos,
             float* phi_min,
             float* phi_max);

/**
 * Computes Cartesian position from joint angles.
 *
 * @param phi Pointer to array of joint angles (radians).
 * @param v   Pointer to output position vector.
 */
void pos_c(float* phi, float* v);

/**
 * Converts angles from radians to degrees.
 *
 * @param phi_rad Pointer to array of angles in radians.
 *                Converted in-place to degrees.
 * @param size    Number of elements in the array.
 */
void rad_2_deg(float* phi_rad, int size);

/**
 * Converts angles from radians to servo pulse values.
 *
 * @param phi_rad   Pointer to array of angles in radians.
 * @param phi_pulse Pointer to output array of pulse values.
 * @param size      Number of elements in the array.
 */
void rad_2_pulse(float* phi_rad, uint16_t* phi_pulse, int size);

/**
 * Converts servo pulse values to radians.
 *
 * @param phi_rad   Pointer to output array of angles in radians.
 * @param phi_pulse Pointer to input array of pulse values.
 * @param size      Number of elements in the array.
 */
void pulse_2_rad(float* phi_rad, uint16_t* phi_pulse, int size);

/**
 * Converts angles from degrees to servo pulse values.
 *
 * @param phi_deg   Pointer to array of angles in degrees.
 * @param phi_pulse Pointer to output array of pulse values.
 * @param size      Number of elements in the array.
 */
void deg_2_pulse(float* phi_deg, uint16_t* phi_pulse, int size);

#endif /* INC_NM_SIMPLEX_H_ */
