

#include "tensorflow/lite/experimental/microfrontend/lib/frontend_handler.h"


struct FrontendConfig frontend_config;
struct FrontendState frontend_state;

bool frontendInitd = false;
int16_t spec_buffer[32][32]; // 2kb

struct FrontendConfig* init_frontend_config(void) {
    FrontendFillConfigWithDefaults(&frontend_config);
    return &frontend_config;
}

struct FrontendState* init_frontend_state(struct FrontendConfig* config) {
  FrontendPopulateState(config, &frontend_state, SAMPLE_RATE);
  return &frontend_state;
}

uint16_t currentEnd;

struct FrontendOutput process_inputs(int16_t* audio_data, size_t* num_samples_read) {
    if (!frontendInitd) {
        init_frontend_config();
        init_frontend_state(&frontend_config);
        frontendInitd = true;
    } 

    return FrontendProcessSamples(&frontend_state, audio_data, currentEnd, num_samples_read);
}

int16_t* get_spectograph(int16_t* audio_data, uint16_t* shiftsCompleted, size_t* sizeResult) {
    // for each window of data, get the FFT
    sizeResult[0] = 0;
    shiftsCompleted[0] = 0;
    currentEnd = WINDOW_SIZE;

    if (frontendInitd) {
        FrontendReset(&frontend_state);
    }

    for (uint16_t i = 0; i < NUM_OF_INTERATIONS; i += 32) {
        // perform an input and store the results in the buffer
        size_t samplesRead = 0;
        int16_t* start = &(audio_data[i]);
        struct FrontendOutput result = process_inputs(start, &samplesRead);
        if (result.size % 32 != 0) {
            // the sample was not completely read as it must have run out.
            sizeResult[0] = result.size;
            return &spec_buffer[0][0];
        }
        currentEnd += SHIFT_LEN;
        
        // shift the result into the graph buffer to the corresponding line
        memcpy(&spec_buffer[shiftsCompleted[0]], result.values, sizeof(int16_t) * result.size);
        shiftsCompleted[0]++;
    }
    
    return &spec_buffer[0][0];
}



