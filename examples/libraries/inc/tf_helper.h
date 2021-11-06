/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#ifndef _TF_HELPER_H
#define _TF_HELPER_H


#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "specg_small_compound.h"
// #include "spec_g_16_64_128_4_8-98.h"

// include platform specific log functions
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// #define PRIMARY_MODEL compound_model_tflite
#define PRIMARY_MODEL compound_model_tflite

#define SECONDARY_MODEL

#define INDEX_NOT_RAINING 0
#define INDEX_RAINING 1

// #define NUM_OF_DIMS 4
// #define NUM_SAMPLES 1
// #define NUM_HEIGHT 64
// #define NUM_WIDTH 64
// #define NUM_CHANNELS 1

#define NUM_OF_DIMS 2
#define NUM_ROWS 32
#define NUM_COLS 32
#define NUM_CELLS 32*NUM_ROWS
// #define NUM_HEIGHT 64
// #define NUM_WIDTH 64
// #define NUM_CHANNELS 1

#define NUM_OF_OUTPUT_DIMS 2
#define NUM_OUTPUTS 2


// #define POWER_TEST
// Expose a C friendly interface for main functions.
#ifdef __cplusplus
extern "C" {
#endif

uint8_t tf_init(void);

#ifdef QUANTISATION // quantisation runs with a float input
void set_input_tensor(float* buffer) ;
#else
void set_input_tensor(int16_t* buffer) ;
#endif

float* get_output_tensor(void);

uint8_t tf_tick(uint8_t* result);

bool is_raining(float* output);

#ifdef __cplusplus
}
#endif

#endif