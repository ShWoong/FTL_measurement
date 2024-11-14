
#include <filters.h>
#include <main.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

/************************************************BUTTERWORTH HIGH PASS FILTER*************************************************/

/*INITIALIZING*/
/*ADC1(EMG)*/
float hpf_x_buffer10[SECTIONS][2] = {0};
float hpf_x_buffer11[SECTIONS][2] = {0};
float hpf_x_buffer12[SECTIONS][2] = {0};
float hpf_x_buffer13[SECTIONS][2] = {0};
float hpf_y_buffer10[SECTIONS][2] = {0};
float hpf_y_buffer11[SECTIONS][2] = {0};
float hpf_y_buffer12[SECTIONS][2] = {0};
float hpf_y_buffer13[SECTIONS][2] = {0};

/*ADC2(STRETCH)*/
float hpf_x_buffer20[SECTIONS][2] = {0};
float hpf_x_buffer21[SECTIONS][2] = {0};
float hpf_x_buffer22[SECTIONS][2] = {0};
float hpf_x_buffer23[SECTIONS][2] = {0};
float hpf_y_buffer20[SECTIONS][2] = {0};
float hpf_y_buffer21[SECTIONS][2] = {0};
float hpf_y_buffer22[SECTIONS][2] = {0};
float hpf_y_buffer23[SECTIONS][2] = {0};

/*FILTER COEFFICIENTS*/
float hpf_sos_20[SECTIONS][6] = {
	{0.8484753, -1.69695059, 0.8484753, 1.0, -1.77831349, 0.79244747},
	{1.0, -2.0, 1.0, 1.0, -1.8934156, 0.90846441} //20Hz
};
float hpf_sos_30[SECTIONS][6] = {
	{0.78136727, -1.56273453,  0.78136727, 1.0, -1.67466095, 0.70485868},
	{1.0, -2.0,  1.0, 1.0, -1.83312526, 0.86618045}
};
float hpf_sos_50[SECTIONS][6] = {
	{0.8484753, -1.69695059,  0.8686753, 1.0, -1.77831349, 0.79244747},
	{1.0, -2.0,  1.0, 1.0, -1.8934156, 0.90846441}
};

float BWHPF(float input, int8_t ch) {
	float output = 0;
	float xn = 0;

	if (ch == 10){

		for (int i = 0; i < SECTIONS; i++) {
			xn = (i == 0) ? input : output;

			output = hpf_sos_20[i][0] * xn + hpf_sos_20[i][1] * hpf_x_buffer10[i][0] + hpf_sos_20[i][2] * hpf_x_buffer10[i][1]
		    - hpf_sos_20[i][4] * hpf_y_buffer10[i][0] - hpf_sos_20[i][5] * hpf_y_buffer10[i][1];

			hpf_x_buffer10[i][1] = hpf_x_buffer10[i][0];
			hpf_x_buffer10[i][0] = xn;
			hpf_y_buffer10[i][1] = hpf_y_buffer10[i][0];
			hpf_y_buffer10[i][0] = output;
		}
	}
	else if (ch == 20){

		for (int i = 0; i < SECTIONS; i++) {
			xn = (i == 0) ? input : output;

			output = hpf_sos_50[i][0] * xn + hpf_sos_50[i][1] * hpf_x_buffer20[i][0] + hpf_sos_50[i][2] * hpf_x_buffer20[i][1]
		    - hpf_sos_50[i][4] * hpf_y_buffer20[i][0] - hpf_sos_50[i][5] * hpf_y_buffer20[i][1];

			hpf_x_buffer20[i][1] = hpf_x_buffer20[i][0];
			hpf_x_buffer20[i][0] = xn;
			hpf_y_buffer20[i][1] = hpf_y_buffer20[i][0];
			hpf_y_buffer20[i][0] = output;
		}
	}
	return output;
}

/*************************************************BUTTERWORTH LOW PASS FILTER*************************************************/
/*INITILAIZING*/
/*ADC1(EMG)*/
float lpf_x_buffer10[SECTIONS][2] = {0};
float lpf_x_buffer11[SECTIONS][2] = {0};
float lpf_x_buffer12[SECTIONS][2] = {0};
float lpf_x_buffer13[SECTIONS][2] = {0};
float lpf_y_buffer10[SECTIONS][2] = {0};
float lpf_y_buffer11[SECTIONS][2] = {0};
float lpf_y_buffer12[SECTIONS][2] = {0};
float lpf_y_buffer13[SECTIONS][2] = {0};
/*ADC2(STRETCH)*/
float lpf_x_buffer20[SECTIONS][2] = {0};
float lpf_x_buffer21[SECTIONS][2] = {0};
float lpf_x_buffer22[SECTIONS][2] = {0};
float lpf_x_buffer23[SECTIONS][2] = {0};
float lpf_x_buffer30[SECTIONS][2] = {0};
float lpf_y_buffer20[SECTIONS][2] = {0};
float lpf_y_buffer21[SECTIONS][2] = {0};
float lpf_y_buffer22[SECTIONS][2] = {0};
float lpf_y_buffer23[SECTIONS][2] = {0};
float lpf_y_buffer30[SECTIONS][2] = {0};

float lpf_sos_1[SECTIONS][6] = {
    {4.97314931e-10, 9.94629862e-10, 4.97314931e-10, 1.0, -1.98983613, 0.98987572},
    {1.0, 2.0, 1.0, 1.0, -1.99490314, 0.994906996} // 1Hz
};
float lpf_sos_2[SECTIONS][6] = {
    {3.91767593e-09, 7.83535185e-09, 3.91767593e-09, 1.0, -1.97958423, 0.979611834},
    {1.0, 2.0, 1.0, 1.0, -1.98983613, 0.989875721} // 2Hz
};
float lpf_sos_3[SECTIONS][6] = {
    {1.08163976e-08, 2.16327951e-08, 1.08163976e-08, 1.0, -1.95903990, 0.95953147},
    {1.0, 2.0, 1.0, 1.0, -1.98513903, 0.98565738} // 3Hz
};
float lpf_sos_4[SECTIONS][6] = {
    {2.41362231e-08, 4.82724463e-08, 2.41362231e-08, 1.0, -1.95400196, 0.954619251},
	{1.0, 2.0, 1.0, 1.0, -1.98032386, 0.980949464} //4Hz
};
float lpf_sos_5[SECTIONS][6] = {
	{5.84514243e-08, 1.16902849e-07, 5.84514243e-08, 1.0, -1.94263823, 0.943597278},
	{1.0, 2.0, 1.0, 1.0, -1.97526963, 0.976244792} //5Hz
};
float lpf_sos_6[SECTIONS][6] = {
    {1.20231162e-07, 2.40462324e-07, 1.20231162e-07, 1.0, -1.93132782, 0.932701053},
    {1.0, 2.0, 1.0, 1.0, -1.97016249, 0.971563337} //6Hz
};
float lpf_sos_10[SECTIONS][6] = {
    {8.98486146e-07, 1.79697229e-06, 8.98486146e-07, 1.0, -1.88660958, 0.890339736},
	{1.0, 2.0, 1.0, 1.0, -1.94921596, 0.953069895} //10Hz
};
float lpf_sos_20[SECTIONS][6] = {
    {1.32937289e-05, 2.65874578e-05, 1.32937289e-05, 1.0, -1.77831349, 0.79244747},
	{1.00000000, 2.00000000, 1.00000000, 1.00000000, -1.89341560, 0.90846441} //20Hz
};

float BWLPF(float input, int8_t ch) {
	float output = 0;
	float xn = 0;

	if (ch == 10){

		for (int i = 0; i < SECTIONS; i++) {
			xn = (i == 0) ? input : output;

			output = lpf_sos_10[i][0] * xn + lpf_sos_10[i][1] * lpf_x_buffer10[i][0] + lpf_sos_10[i][2] * lpf_x_buffer10[i][1]
			- lpf_sos_10[i][4] * lpf_y_buffer10[i][0] - lpf_sos_10[i][5] * lpf_y_buffer10[i][1];

			lpf_x_buffer10[i][1] = lpf_x_buffer10[i][0];
			lpf_x_buffer10[i][0] = xn;
			lpf_y_buffer10[i][1] = lpf_y_buffer10[i][0];
			lpf_y_buffer10[i][0] = output;
		    }
	}
	else if (ch == 20){

		for (int i = 0; i < SECTIONS; i++) {
			xn = (i == 0) ? input : output;

			output = lpf_sos_20[i][0] * xn + lpf_sos_20[i][1] * lpf_x_buffer20[i][0] + lpf_sos_20[i][2] * lpf_x_buffer20[i][1]
			- lpf_sos_20[i][4] * lpf_y_buffer20[i][0] - lpf_sos_20[i][5] * lpf_y_buffer20[i][1];

			lpf_x_buffer20[i][1] = lpf_x_buffer20[i][0];
			lpf_x_buffer20[i][0] = xn;
			lpf_y_buffer20[i][1] = lpf_y_buffer20[i][0];
			lpf_y_buffer20[i][0] = output;
		    }
	}
	else if (ch == 2){

			for (int i = 0; i < SECTIONS; i++) {
				xn = (i == 0) ? input : output;

				output = lpf_sos_2[i][0] * xn + lpf_sos_2[i][1] * lpf_x_buffer20[i][0] + lpf_sos_2[i][2] * lpf_x_buffer20[i][1]
				- lpf_sos_2[i][4] * lpf_y_buffer20[i][0] - lpf_sos_2[i][5] * lpf_y_buffer20[i][1];

				lpf_x_buffer20[i][1] = lpf_x_buffer20[i][0];
				lpf_x_buffer20[i][0] = xn;
				lpf_y_buffer20[i][1] = lpf_y_buffer20[i][0];
				lpf_y_buffer20[i][0] = output;
			    }
		}
	else if (ch == 4){

				for (int i = 0; i < SECTIONS; i++) {
					xn = (i == 0) ? input : output;

					output = lpf_sos_4[i][0] * xn + lpf_sos_4[i][1] * lpf_x_buffer20[i][0] + lpf_sos_4[i][2] * lpf_x_buffer20[i][1]
					- lpf_sos_4[i][4] * lpf_y_buffer20[i][0] - lpf_sos_4[i][5] * lpf_y_buffer20[i][1];

					lpf_x_buffer20[i][1] = lpf_x_buffer20[i][0];
					lpf_x_buffer20[i][0] = xn;
					lpf_y_buffer20[i][1] = lpf_y_buffer20[i][0];
					lpf_y_buffer20[i][0] = output;
				    }
			}
	else if (ch == 3){

	    for (int i = 0; i < SECTIONS; i++) {
	        xn = (i == 0) ? input : output;

	        output = lpf_sos_3[i][0] * xn + lpf_sos_3[i][1] * lpf_x_buffer30[i][0] + lpf_sos_3[i][2] * lpf_x_buffer30[i][1]
	        - lpf_sos_3[i][4] * lpf_y_buffer30[i][0] - lpf_sos_3[i][5] * lpf_y_buffer30[i][1];

	        lpf_x_buffer30[i][1] = lpf_x_buffer30[i][0];
	        lpf_x_buffer30[i][0] = xn;
	        lpf_y_buffer30[i][1] = lpf_y_buffer30[i][0];
	        lpf_y_buffer30[i][0] = output;
	    }
	}
    return output;
}

/*******************************************FINITE IMPULSE RESPONSE LOW PASS FILTER*******************************************/
/*static float firFilterCoeffs[FILTER_TAP_NUM] = {
    // 여기에 계산된 계수 넣기
    0.00149401, 0.00151131, 0.00156314, 0.00164928, 0.00176939,
    0.00192298, 0.00210944, 0.00232802, 0.00257783, 0.00285788,
    0.00316703, 0.00350404, 0.00386755, 0.0042561 , 0.00466813,
    0.00510197, 0.00555589, 0.00602804, 0.00651654, 0.00701941,
    0.00753463, 0.00806013, 0.00859378, 0.00913344, 0.00967695,
    0.0102221 , 0.0107667 , 0.01130856, 0.01184551, 0.01237536,
    0.012896  , 0.01340533, 0.01390129, 0.01438189, 0.01484519,
    0.01528933, 0.01571251, 0.01611304, 0.0164893 , 0.01683977,
    0.01716305, 0.01745783, 0.01772293, 0.01795727, 0.01815992,
    0.01833005, 0.01846699, 0.01857018, 0.0186392 , 0.01867379,
    0.01867379, 0.0186392 , 0.01857018, 0.01846699, 0.01833005,
    0.01815992, 0.01795727, 0.01772293, 0.01745783, 0.01716305,
    0.01683977, 0.0164893 , 0.01611304, 0.01571251, 0.01528933,
    0.01484519, 0.01438189, 0.01390129, 0.01340533, 0.012896  ,
    0.01237536, 0.01184551, 0.01130856, 0.0107667 , 0.0102221 ,
    0.00967695, 0.00913344, 0.00859378, 0.00806013, 0.00753463,
    0.00701941, 0.00651654, 0.00602804, 0.00555589, 0.00510197,
    0.00466813, 0.0042561 , 0.00386755, 0.00350404, 0.00316703,
    0.00285788, 0.00257783, 0.00232802, 0.00210944, 0.00192298,
    0.00176939, 0.00164928, 0.00156314, 0.00151131, 0.00149401
};

float firFilterState[FILTER_TAP_NUM] = {0};

// FIR 필터 초기화 함수
void FIRF_Init(void) {
    // 필터 상태 초기화
    for (int i = 0; i < FILTER_TAP_NUM; ++i) {
        firFilterState[i] = 0;
    }
}

// FIR 필터 처리 함수
float FIRF_Process(float input) {

    // 상태 배열 업데이트 (가장 오래된 값을 버리고, 새로운 입력을 배열의 처음에 추가)
    for (int i = FILTER_TAP_NUM - 1; i > 0; --i) {
        firFilterState[i] = firFilterState[i - 1];
    }
    firFilterState[0] = input;

    // FIR 필터 적용 (계수와 상태 배열의 곱의 합 계산)
    float output = 0;
    for (int i = 0; i < FILTER_TAP_NUM; ++i) {
        output += firFilterState[i] * firFilterCoeffs[i];
    }

    return output;
}

/*******************************************FINITE IMPULSE RESPONSE HIGH PASS FILTER******************************************/
/*static float highPassFilterCoeffs[FILTER_TAP_NUM] = {
    -3.19998918e-06, -3.23629810e-06, -3.34507992e-06, -3.52590537e-06, -3.77806088e-06,
    -4.10055137e-06, -4.49210415e-06, -4.95117398e-06, -5.47594919e-06, -6.06435875e-06,
    -6.71408052e-06, -7.42255037e-06, -8.18697234e-06, -9.00432962e-06, -9.87139651e-06,
    -1.07847511e-05, -1.17407888e-05, -1.27357366e-05, -1.37656680e-05, -1.48265182e-05,
    -1.59141005e-05, -1.70241229e-05, -1.81522044e-05, -1.92938932e-05, -2.04446835e-05,
    -2.16000336e-05, -2.27553839e-05, -2.39061747e-05, -2.50478644e-05, -2.61759473e-05,
    -2.72859713e-05, -2.83735556e-05, -2.94344081e-05, -3.04643419e-05, -3.14592925e-05,
    -3.24153332e-05, -3.33286909e-05, -3.41957610e-05, -3.50131215e-05, -3.57775468e-05,
    -3.64860198e-05, -3.71357447e-05, -3.77241573e-05, -3.82489352e-05, -3.87080075e-05,
    -3.90995624e-05, -3.94220547e-05, -3.96742114e-05, -3.98550376e-05, -3.99638196e-05,
     9.99963200e-01, -3.99638196e-05, -3.98550376e-05, -3.96742114e-05, -3.94220547e-05,
    -3.90995624e-05, -3.87080075e-05, -3.82489352e-05, -3.77241573e-05, -3.71357447e-05,
    -3.64860198e-05, -3.57775468e-05, -3.50131215e-05, -3.41957610e-05, -3.33286909e-05,
    -3.24153332e-05, -3.14592925e-05, -3.04643419e-05, -2.94344081e-05, -2.83735556e-05,
    -2.72859713e-05, -2.61759473e-05, -2.50478644e-05, -2.39061747e-05, -2.27553839e-05,
    -2.16000336e-05, -2.04446835e-05, -1.92938932e-05, -1.81522044e-05, -1.70241229e-05,
    -1.59141005e-05, -1.48265182e-05, -1.37656680e-05, -1.27357366e-05, -1.17407888e-05,
    -1.07847511e-05, -9.87139651e-06, -9.00432962e-06, -8.18697234e-06, -7.42255037e-06,
    -6.71408052e-06, -6.06435875e-06, -5.47594919e-06, -4.95117398e-06, -4.49210415e-06,
    -4.10055137e-06, -3.77806088e-06, -3.52590537e-06, -3.34507992e-06, -3.23629810e-06,
    -3.19998918e-06
};

// 필터 상태
static float highPassFilterState[HPFILTER_TAP_NUM] = {0};

// FIR 고역 통과 필터 초기화
void HighPassFilter_Init(void) {
    for (int i = 0; i < HPFILTER_TAP_NUM; ++i) {
        highPassFilterState[i] = 0;
    }
}

// FIR 고역 통과 필터 처리 함수
float HighPassFilter_Process(float input) {
    for (int i = HPFILTER_TAP_NUM - 1; i > 0; --i) {
        highPassFilterState[i] = highPassFilterState[i - 1];
    }
    highPassFilterState[0] = input;

    float output = 0;
    for (int i = 0; i < HPFILTER_TAP_NUM; ++i) {
        output += highPassFilterState[i] * highPassFilterCoeffs[i];
    }

    return output;
}*/

/****************************************************MOVING AVERAGE FILTER****************************************************/
float MAF(float new_sample) {
    static float samples[SAMPLE_SIZE] = {0};
    static int index = 0;
    static float sum = 0;
    float average = 0;

    // 이전 합계에서 가장 오래된 샘플 제거
    sum -= samples[index];
    // 새 샘플 추가
    samples[index] = new_sample;
    sum += new_sample;
    // 다음 샘플을 위한 인덱스 업데이트
    index = (index + 1) % SAMPLE_SIZE;

    // 평균 계산
    average = sum / SAMPLE_SIZE;

    return average;
}

/*****************************************EXPONENTIAL WEIGHTED MOVING AVERAGE FILTER******************************************/
float EWMAF(float new_measurement, float prev_ewma, float alpha) {
    return alpha * new_measurement + (1 - alpha) * prev_ewma;
}

/*void KMF_Init(KMF *kf, float init_estimate, float init_error_estimate, float error_measure) {
    kf->estimate = init_estimate;
    kf->error_estimate = init_error_estimate;
    kf->error_measure = error_measure;
}

void KMF_Update(KMF *kf, float measurement) {
    kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->error_measure);
    kf->estimate = kf->estimate + kf->kalman_gain * (measurement - kf->estimate);
    kf->error_estimate = (1 - kf->kalman_gain) * kf->error_estimate;
}*/

/************************************************NEURAL ACTIVATION CALCULATION************************************************/
float na = 0, nat1 = 0, nat2 = 0;

float NEURAL_ACTIVATION(float emg){
	float gma1 = -0.75, gma2 = -0.125;
	float bet1 = gma1 + gma2, bet2 = gma1*gma2, alp = 1 + bet1 + bet2;

	nat2 = nat1;
	nat1 = na;
	na = (alp*emg - bet1*nat1 - bet2*nat2)-4;
	if(na<0){
	  na=0;
	}

	return na;
}

/************************************************MUSCLE ACTIVATION CALCULATION************************************************/
float MUSCLE_ACTIVATION(float neural_activation){
	float A = -0.03, max_value = 33.81404;
	float ma = (exp(A*neural_activation) - 1)/(exp(A) - 1);
	float ma_cal = ma/max_value*100;

	return (int32_t)round(ma_cal);
}

/******************************************************TORQUE GENERATION******************************************************/
float FORCE_GENERATION(float muscle_activation, float muscle_fiber_length, float muscle_contraction_velocity){
	float q0 = -2.06, q1 = 6.16, q2 = -3.13, pa = 13.9*M_PI/180, lt = 34.6, lm0 = 7.6, Fm0 = 848.8, fA;
	float lm = lm0*0.5, l = lm/lm0, lmt = lt + lm*cosd(pa);

	if(l>=0.5 || l<=1.5){
		fA = q0 + q1*l + q2*powf(l, 2);
	}
	else{
		fA = 0;
	}
	float fP = exp(10*l-15);
	float fV = 1;

	float FmA = fA*fV*muscle_activation*Fm0;
	float FmP = fP*Fm0;
	float Fmt = (FmA + FmP)*cos(pa);

	return Fmt;
}

/***********************************************STRETCH SENSOR CAP CALCULATION************************************************/
/*float STRETCH_SENSOR(void){
	const float C_stray = 7, adc_max = 4096;
	float adc;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	sConfig.Channel = ADC_CHANNEL_12;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
		adc = HAL_ADC_GetValue(&hadc1);
	}

	float C = adc*C_stray/(adc_max-adc);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

	return C;
}*/
