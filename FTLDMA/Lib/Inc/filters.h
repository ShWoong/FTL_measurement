
#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

#define FILTER_ORDER 2
#define SECTIONS 2
//#define FILTER_TAP_NUM 100
#define SAMPLE_SIZE 100
#define FILTER_TAP_NUM 100
#define HPFILTER_TAP_NUM 101

float BWHPF(float input, int8_t ch);
float BWLPF(float input, int8_t ch);
float NEURAL_ACTIVATION(float emg);
float MUSCLE_ACTIVATION(float neural_activation);
float FORCE_GENERATION(float muscle_activation, float muscle_fiber_length, float muscle_contraction_velocity);
float STRETCH_SENSOR(void);
float EMG_SENSOR(void);
void FIRF_Init(void);
float FIRF_Process(float input);
uint16_t MAF(uint16_t new_sample);
/*float EWMAF(float new_measurement, float prev_ewma, float alpha);

void HighPassFilter_Init(void);
float HighPassFilter_Process(float input);
float applyLowPassFilter(float input);

typedef struct {
    float estimate;
    float error_estimate;
    float error_measure;
    float kalman_gain;
} KMF;

void KMF_Init(KMF *kf, float init_estimate, float init_error_estimate, float error_measure);
void KMF_Update(KMF *kf, float measurement);*/

#endif /* FILTERS_H */
