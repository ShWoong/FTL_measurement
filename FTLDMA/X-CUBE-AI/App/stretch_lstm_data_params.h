/**
  ******************************************************************************
  * @file    stretch_lstm_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-01-09T01:48:46+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef STRETCH_LSTM_DATA_PARAMS_H
#define STRETCH_LSTM_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_STRETCH_LSTM_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_stretch_lstm_data_weights_params[1]))
*/

#define AI_STRETCH_LSTM_DATA_CONFIG               (NULL)


#define AI_STRETCH_LSTM_DATA_ACTIVATIONS_SIZES \
  { 516, }
#define AI_STRETCH_LSTM_DATA_ACTIVATIONS_SIZE     (516)
#define AI_STRETCH_LSTM_DATA_ACTIVATIONS_COUNT    (1)
#define AI_STRETCH_LSTM_DATA_ACTIVATION_1_SIZE    (516)



#define AI_STRETCH_LSTM_DATA_WEIGHTS_SIZES \
  { 4868, }
#define AI_STRETCH_LSTM_DATA_WEIGHTS_SIZE         (4868)
#define AI_STRETCH_LSTM_DATA_WEIGHTS_COUNT        (1)
#define AI_STRETCH_LSTM_DATA_WEIGHT_1_SIZE        (4868)



#define AI_STRETCH_LSTM_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_stretch_lstm_activations_table[1])

extern ai_handle g_stretch_lstm_activations_table[1 + 2];



#define AI_STRETCH_LSTM_DATA_WEIGHTS_TABLE_GET() \
  (&g_stretch_lstm_weights_table[1])

extern ai_handle g_stretch_lstm_weights_table[1 + 2];


#endif    /* STRETCH_LSTM_DATA_PARAMS_H */
