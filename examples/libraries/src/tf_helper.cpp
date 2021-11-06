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

#include "tf_helper.h"


// Global variables, used to be compatible with Arduino style sketches.
namespace {
    tflite::ErrorReporter    *error_reporter = nullptr;
    const tflite::Model      *model          = nullptr;
    tflite::MicroInterpreter *interpreter    = nullptr;
    TfLiteTensor             *model_input    = nullptr;
    int                       input_length;


    // Create a memory area for input, output and intermediate arrays.
    // The size depends on the model you are using and may need to be determined
    // experimentally.
    // constexpr int kTensorArenaSize = 14 * 1024 + 1332; // ~15.3kb
    // constexpr int kTensorArenaSize = 88064; // 86kb * 1024b/kb
    // constexpr int kTensorArenaSize = 204800; // 200kb * 1024b/kb
    // constexpr int kTensorArenaSize = 184320; // 180kb * 1024b/kb
    // constexpr int kTensorArenaSize = 179200; // 175kb * 1024b/kb
    // constexpr int kTensorArenaSize = 32920; // 80kb * 1024b/kb
    // constexpr int kTensorArenaSize = 13684; // exact size of model
    
    constexpr int kTensorArenaSize = 131072; // 128kb 
    // constexpr int kTensorArenaSize = 196608; // 192kb
    // constexpr int kTensorArenaSize = 199680;// 195kb
    // constexpr int kTensorArenaSize = 204800; 
    uint8_t       tensor_arena[kTensorArenaSize];
#ifdef QUANTISATION
    float* model_input_buffer = nullptr;
#else
    int16_t* model_input_buffer = nullptr;
#endif
} // namespace



uint8_t tf_init(void) {
    NRF_LOG_INFO("initialising TF libs");
    NRF_LOG_FLUSH();
    tflite::InitializeTarget();

    static tflite::MicroErrorReporter micro_error_reporter;  // NOLINT
    error_reporter = &micro_error_reporter;

    // Map the model to the available data structure.
    // This does not involve any copying or parsing, which is a very lightweight
    // operation.
    NRF_LOG_INFO("Loading Model");
    NRF_LOG_FLUSH();
    model = tflite::GetModel(PRIMARY_MODEL);
    NRF_LOG_INFO("Checking Version");
    NRF_LOG_FLUSH();
    if (model->version() != TFLITE_SCHEMA_VERSION) {
    NRF_LOG_ERROR("Model provided is schema version %d not equal "
                            "to supported version %d.",
                            model->version(), TFLITE_SCHEMA_VERSION);
        return 1;
    }

    NRF_LOG_INFO("Loading operation resolver");
    NRF_LOG_FLUSH();
    

    // Only introduce the operation implementation we need.
    // It depends on the complete list of all operations required for this graph.
    // A simpler method is to use AllOpsResolver only,
    // but this will result in a loss of code space for op implementations that are not
    // needed in this figure.
#ifdef QUANTISATION // quantisation runs with a float input
    static tflite::MicroMutableOpResolver<11> micro_op_resolver;
    NRF_LOG_INFO("loading quantisation operations");
    NRF_LOG_FLUSH();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddDequantize();
#else
    static tflite::MicroMutableOpResolver<11> micro_op_resolver;  // NOLINT
#endif
    
    NRF_LOG_INFO("loading operations");
    NRF_LOG_FLUSH();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddCast();
    micro_op_resolver.AddResizeBilinear();
    micro_op_resolver.AddL2Normalization();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddDepthwiseConv2D();
    micro_op_resolver.AddFullyConnected();
    
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddDequantize();

    NRF_LOG_INFO("Building interpreter");
    NRF_LOG_FLUSH();
    
    // Build an interpreter to run the model.
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter = &static_interpreter;

    NRF_LOG_INFO("Allocating Tensors");
    NRF_LOG_FLUSH();
    // Allocate memory from tensor_arena for the tensor of the model.
  interpreter->AllocateTensors();
    NRF_LOG_INFO("verifying model");
    NRF_LOG_FLUSH();
    // Get a pointer to the input tensor of the model.
    model_input = interpreter->input(0);
    TfLiteTensor *model_output = interpreter->output(0);

    if ((model_input->dims->size != NUM_OF_DIMS) 
            || (model_input->dims->data[0] != NUM_ROWS)
            || (model_input->dims->data[1] != NUM_COLS)
            // || (model_input->dims->data[2] != NUM_WIDTH)
            // || (model_input->dims->data[3] != NUM_CHANNELS)
#ifdef QUANTISATION // quantisation runs with a float input
            || (model_input->type != kTfLiteFloat32) ) {
#else
            || (model_input->type != kTfLiteInt16) ) {
#endif
                
        NRF_LOG_ERROR("Bad input tensor parameters in model %d, [%d, %d] type %d", model_input->dims->size, model_input->dims->data[0], model_input->dims->data[1], model_input->type);
        NRF_LOG_ERROR("Potentially Bad output tensor parameters in model %d, type %d", model_output->dims->size, model_output->type);
        // return 2;
    }
    NRF_LOG_INFO("Input Tensor %d bytes, output %d bytes", model_input->bytes, model_output->bytes);

    
    if ((model_output->dims->size != NUM_OF_OUTPUT_DIMS)
            || ((model_output->type != kTfLiteFloat32) )) {
        NRF_LOG_ERROR("Bad output tensor parameters in model %d, type %d", model_output->dims->size, model_output->type);
        // return 3;
    }
    NRF_LOG_INFO("almost complete");
    NRF_LOG_FLUSH();
    input_length = model_input->bytes / sizeof(float);
#ifdef QUANTISATION // quantisation runs with a float input
    model_input_buffer = model_input->data.f;
#else
    model_input_buffer = model_input->data.i16;
#endif

    return 0;
}

#ifdef QUANTISATION // quantisation runs with a float input
void set_input_tensor(float* buffer) {
#else
void set_input_tensor(int16_t* buffer) {
#endif
    for (uint16_t i = 0; i < NUM_CELLS; i++) {
#ifdef QUANTISATION // quantisation runs with a float input
        model_input->data.f[i] = buffer[i];
#else
        model_input->data.i16[i] = buffer[i];
#endif
    }
}

float* get_output_tensor(void) {
    return interpreter->output(0)->data.f;
}

uint8_t tf_tick(uint8_t* result) {
    
    // Assign Data
    
    // inference start
    TfLiteStatus invoke_status = interpreter->Invoke();

    if (invoke_status != kTfLiteOk) {
        NRF_LOG_ERROR("Invoke failed %d \n", invoke_status);
        return 1;
    }

    // read the models' output
    bool raining = is_raining(interpreter->output(0)->data.f);

    // NRF_LOG_INFO("Device Inferred: %s.\n", (raining?"Raining":"Not Raining"))
    #ifdef POWER_TEST
    result[0] = 0; // don't turn on LED since LED will skew power consumption.
    #else
    // turn the LED on or off to indicate inference
    result[0] = raining;
    #endif

    return 0;
}

bool is_raining(float* output) {
    // check for the larger output: 0 - not raining, 1 - raining
    bool raining = false;
    // NRF_LOG_INFO("Recieved Raining %d vs Not raining %d", (int)(output[INDEX_RAINING]*100), (int)(output[INDEX_NOT_RAINING]*100))
    if (output[INDEX_NOT_RAINING] < output[INDEX_RAINING]) {
        raining = true;
    }
    return raining;
}