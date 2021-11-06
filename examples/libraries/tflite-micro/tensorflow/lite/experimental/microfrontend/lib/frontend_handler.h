
#ifndef _FRONTEND_HANDLER_H_
#define _FRONTEND_HANDLER_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "tensorflow/lite/experimental/microfrontend/lib/frontend_util.h"
#include "stdbool.h"
#include "string.h" // required for memcpy
#define SAMPLE_SIZE 10500
#define SAMPLE_RATE 16000
// #define SLICE_SIZE 480
// #define NUM_OF_SLICES 131

#define WINDOW_SIZE 512
#define SHIFT_LEN 312
#define NUM_OF_POSITIONS 32
#define NUM_OF_INTERATIONS 1024


struct FrontendConfig* init_frontend_config(void);
struct FrontendState* init_frontend_state(struct FrontendConfig* config);
struct FrontendOutput process_inputs(int16_t* audio_data, size_t* num_samples_read);
int16_t* get_spectograph(int16_t* audio_data, uint16_t* shiftsCompleted, size_t* sizeResult);


#ifdef __cplusplus
}
#endif
#endif