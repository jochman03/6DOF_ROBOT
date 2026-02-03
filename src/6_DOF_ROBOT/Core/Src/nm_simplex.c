/*
 * nm_simplex.c
 *
 *  Created on: 11 gru 2025
 *      Author: jakub
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "nm_simplex.h"

#define SWAP(a, b) {swap=(a);(a)=(b);(b)=swap;}
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define N 4

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

static inline float clampf(float x, float lo, float hi){
    if (x < lo) {
    	return lo;
    }
    if (x > hi) {
    	return hi;
    }
    return x;
}

static inline void clamp_vec(float* x, float* lo, float* hi){
    for (int i = 0; i < N; i++) {
        x[i] = clampf(x[i], lo[i], hi[i]);
    }
}

void simplex(float (*fun)(float*, float*, float*), float* phi0, int maxIter, float* f_min, float* x_min, float* pos, float* phi_min, float* phi_max){
    float simplex[N+1][N];
    float step = 0.05f;
    float fvals[N+1];

    float alpha = 1.0f;
    float gamma = 2.0f;
    float rho   = 0.5f;
    float sigma = 0.5f;


    for(int i = 0; i < N+1; i++){
        for(int j = 0; j < N; j++){
            simplex[i][j] = phi0[j];
        }
        if(i > 0){
            simplex[i][i-1] += step;
        }
        clamp_vec(simplex[i], phi_min, phi_max);
    }

    for(int iter = 0; iter < maxIter; iter++){

        for (int k = 0; k < N+1; k++){
            fvals[k] = fun(simplex[k], pos, phi0);
        }

        float swap;
        for(int j = 0; j < N+1; j++){
            for(int k = j+1; k < N+1; k++){
                if(fvals[j] > fvals[k]){
                    SWAP(fvals[j], fvals[k]);
                    for(int l = 0; l < N; l++){
                        SWAP(simplex[j][l], simplex[k][l]);
                    }
                }
            }
        }


        float centroid[N] = {0};
        for(int k = 0; k < N; k++){
            centroid[k] = 0.0;
            for(int l = 0; l < N; l++){
                centroid[k] += simplex[l][k];
            }
            centroid[k] /= N;
        }

        float x_ref[N];
        for(int k = 0; k < N; k++){
            x_ref[k] = centroid[k] + alpha * (centroid[k] - simplex[N][k]);
        }
        clamp_vec(x_ref, phi_min, phi_max);
        float f_ref = fun(x_ref, pos, phi0);

        if(f_ref < fvals[0]) {

            float x_exp[N];
            for(int k = 0; k < N; k++){
                x_exp[k] = centroid[k] + gamma * (x_ref[k] - centroid[k]);
            }
            clamp_vec(x_exp, phi_min, phi_max);
            float f_exp = fun(x_exp, pos, phi0);

            if(f_exp < f_ref){
                for(int k = 0; k < N; k++){
                    simplex[N][k] = x_exp[k];
                }
                fvals[N] = f_exp;
            }
            else{
                for(int k = 0; k < N; k++){
                    simplex[N][k] = x_ref[k];
                }
                fvals[N] = f_ref;
            }
            continue;
        }

        if(f_ref < fvals[N-1]) {
            for(int k = 0; k < N; k++){
                simplex[N][k] = x_ref[k];
            }
            fvals[N] = f_ref;
            continue;
        }

        float x_con[N];
        float f_con;

        if(f_ref < fvals[N]) {
            for(int k = 0; k < N; k++){
                x_con[k] = centroid[k] + rho * (x_ref[k] - centroid[k]);
            }
            clamp_vec(x_con, phi_min, phi_max);
            f_con = fun(x_con, pos, phi0);
        }
        else {
            for(int k = 0; k < N; k++){
                x_con[k] = centroid[k] - rho * (centroid[k] - simplex[N][k]);
            }
            clamp_vec(x_con, phi_min, phi_max);
            f_con = fun(x_con, pos, phi0);
        }

        if(f_con < fvals[N]) {
            for(int k = 0; k < N; k++){
                simplex[N][k] = x_con[k];
            }
            fvals[N] = f_con;
            continue;
        }

        for(int i = 1; i < N+1; i++){
            for(int k = 0; k < N; k++){
                simplex[i][k] = simplex[0][k] + sigma * (simplex[i][k] - simplex[0][k]);
            }
            clamp_vec(simplex[i], phi_min, phi_max);
        }

    }


    for (int k = 0; k < N+1; k++){
        fvals[k] = fun(simplex[k], pos, phi0);
    }

    float swap;
    for(int j = 0; j < N+1; j++){
        for(int k = j+1; k < N+1; k++){
            if(fvals[j] > fvals[k]){
                SWAP(fvals[j], fvals[k]);
                for(int l = 0; l < N; l++){
                    SWAP(simplex[j][l], simplex[k][l]);
                }
            }
        }
    }

    *f_min = fvals[0];
    for(int i = 0; i < N; i++){
        x_min[i] = simplex[0][i];
    }
}

float pos_f(float* phi, float* pos , float* phi0){
    float x_pos =
        -70.0f * sinf(phi[0] + phi[1] + phi[2])
        -25.0f * sinf(-phi[0] + phi[1] + phi[2] + phi[3])
        +55.0f * sinf(phi[0] - phi[1])
        -25.0f * sinf(phi[0] + phi[1] + phi[2] + phi[3])
        -55.0f * sinf(phi[0] + phi[1])
        -70.0f * sinf(-phi[0] + phi[1] + phi[2]);

    float y_pos =
        +70.0f * cosf(phi[0] + phi[1] + phi[2])
        -25.0f * cosf(-phi[0] + phi[1] + phi[2] + phi[3])
        -55.0f * cosf(phi[0] - phi[1])
        +25.0f * cosf(phi[0] + phi[1] + phi[2] + phi[3])
        +55.0f * cosf(phi[0] + phi[1])
        -70.0f * cosf(-phi[0] + phi[1] + phi[2]);

    float z_pos =
        +50.0f * cosf(phi[1] + phi[2] + phi[3])
        +140.0f * cosf(phi[1] + phi[2])
        +110.0f * cosf(phi[1])
        +120.0f;

    float dx = pos[0] - x_pos;
    float dy = pos[1] - y_pos;
    float dz = pos[2] - z_pos;

    float ang_pun = 0;
    float ang_cost = 100;
    for(uint8_t i = 0; i < N; i++){
    	float t = phi[i] - phi0[i];
    	ang_pun += ang_cost * t * t;
    }


    return dx*dx + dy*dy + dz*dz + ang_pun;
}

void pos_c(float* phi, float* v){
    v[0] =
        -70.0f * sinf(phi[0] + phi[1] + phi[2])
        -25.0f * sinf(-phi[0] + phi[1] + phi[2] + phi[3])
        +55.0f * sinf(phi[0] - phi[1])
        -25.0f * sinf(phi[0] + phi[1] + phi[2] + phi[3])
        -55.0f * sinf(phi[0] + phi[1])
        -70.0f * sinf(-phi[0] + phi[1] + phi[2]);

    v[1] =
        +70.0f * cosf(phi[0] + phi[1] + phi[2])
        -25.0f * cosf(-phi[0] + phi[1] + phi[2] + phi[3])
        -55.0f * cosf(phi[0] - phi[1])
        +25.0f * cosf(phi[0] + phi[1] + phi[2] + phi[3])
        +55.0f * cosf(phi[0] + phi[1])
        -70.0f * cosf(-phi[0] + phi[1] + phi[2]);

    v[2] =
        +50.0f * cosf(phi[1] + phi[2] + phi[3])
        +140.0f * cosf(phi[1] + phi[2])
        +110.0f * cosf(phi[1])
        +120.0f;
}

void deg_2_rad(float* phi_deg, int size){
    for(uint8_t i = 0; i < size; i++){
        phi_deg[i] = phi_deg[i] * DEG2RAD;
    }
}
void rad_2_deg(float* phi_rad, int size){
    for(uint8_t i = 0; i < size; i++){
        phi_rad[i] = phi_rad[i] * RAD2DEG;
    }
}
void rad_2_pulse(float* phi_rad, uint16_t* phi_pulse, int size){
    const float scale = 1000.0f / M_PI;

    for(uint8_t i = 0; i < size; i++){
        float phi = phi_rad[i];

        if(phi < -M_PI_2){
            phi = -M_PI_2;
        }
        if(phi >  M_PI_2){
            phi =  M_PI_2;
        }

        float pulse_f = (phi + M_PI_2) * scale;

        if(pulse_f < 0.0f){
            pulse_f = 0.0f;
        }
        if(pulse_f > 1000.0f){
            pulse_f = 1000.0f;
        }

        phi_pulse[i] = (uint16_t)(pulse_f + 0.5f);
    }
}
void pulse_2_rad(float* phi_rad, uint16_t* phi_pulse, int size){
    const float scale = M_PI / 1000.0f;

    for(uint8_t i = 0; i < size; i++){
        uint16_t p = phi_pulse[i];

        if(p > 1000){
            p = 1000;
        }

        phi_rad[i] = p * scale - M_PI_2;
    }
}

void deg_2_pulse(float* phi_deg, uint16_t* phi_pulse, int size){
    const float scale = 1000.0f / 180.0f;

    for(uint8_t i = 0; i < size; i++){
        float phi = phi_deg[i];

        if(phi < -90.0f){
            phi = -90.0f;
        }
        if(phi >  90.0f){
            phi =  90.0f;
        }

        float p = (phi + 90.0f) * scale;

        if(p < 0.0f){
            p = 0.0f;
        }
        if(p > 1000.0f){
            p = 1000.0f;
        }

        phi_pulse[i] = (uint16_t)(p + 0.5f);
    }
}

