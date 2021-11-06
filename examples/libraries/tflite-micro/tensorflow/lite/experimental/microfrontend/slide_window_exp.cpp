
#include "tensorflow/lite/micro/examples/micro_speech/main_functions.h"

#include "tensorflow/lite/micro/examples/micro_speech/audio_provider.h"
#include "tensorflow/lite/micro/examples/micro_speech/command_responder.h"
#include "tensorflow/lite/micro/examples/micro_speech/feature_provider.h"
#include "tensorflow/lite/micro/examples/micro_speech/micro_features/micro_model_settings.h"
#include "tensorflow/lite/micro/examples/micro_speech/recognize_commands.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/system_setup.h"

// Globals, used for compatibility with Arduino-style sketches.

TfLiteTensor* model_input = NULL;
// RecognizeCommands* recognizer = NULL;
int32_t previous_time = 0;
int32_t current_time = 0;
int kFeatureElementCount = 2;
// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
uint8_t tensor_arena[10 * 1024];
int8_t feature_buffer[kFeatureElementCount];
int8_t* model_input_buffer = NULL;

void PopFeatData(int32_t last_time_in_ms,
    int32_t time_in_ms, int* how_many_new_slices);
void getAudSam(int start_ms, int duration_ms,
                             int* audio_samples_size);
TfLiteStatus test(void);

TfLiteStatus test(void) {

    // FeatureProvider* feature_provider = NULL;
    int32_t previous_time = 0;
    current_time += 100;
    int how_many_new_slices = 0;
    PopFeatData(previous_time, current_time, &how_many_new_slices);

}

FeatureProvider::FeatureProvider(int feature_size, int8_t* feature_data)
    : feature_size_(feature_size),
      feature_data_(feature_data),
      is_first_run_(true) {
  // Initialize the feature data to default values.
  for (int n = 0; n < feature_size_; ++n) {
    feature_data_[n] = 0;
  }
}

int kFeatureSliceSize = 40;
int kFeatureSliceCount = 49;
int kFeatureElementCount = 40 * 49;
int kFeatureSliceStrideMs = 20;
int kFeatureSliceDurationMs = 30;
bool is_first_run_ = true;
void PopFeatData(int32_t last_time_in_ms, int32_t time_in_ms, int* how_many_new_slices) {
    
  // Quantize the time into steps as long as each window stride, so we can
  // figure out which audio data we need to fetch.
  const int last_step = (last_time_in_ms / kFeatureSliceStrideMs);
  const int current_step = (time_in_ms / kFeatureSliceStrideMs);

  int slices_needed = current_step - last_step;
  // If this is the first call, make sure we don't use any cached information.
  if (is_first_run_) {
    is_first_run_ = false;
    slices_needed = kFeatureSliceCount;
  }
  if (slices_needed > kFeatureSliceCount) {
    slices_needed = kFeatureSliceCount;
  }
  *how_many_new_slices = slices_needed;

  const int slices_to_keep = kFeatureSliceCount - slices_needed;
  const int slices_to_drop = kFeatureSliceCount - slices_to_keep;
  // If we can avoid recalculating some slices, just move the existing data
  // up in the spectrogram, to perform something like this:
  // last time = 80ms          current time = 120ms
  // +-----------+             +-----------+
  // | data@20ms |         --> | data@60ms |
  // +-----------+       --    +-----------+
  // | data@40ms |     --  --> | data@80ms |
  // +-----------+   --  --    +-----------+
  // | data@60ms | --  --      |  <empty>  |
  // +-----------+   --        +-----------+
  // | data@80ms | --          |  <empty>  |
  // +-----------+             +-----------+
  if (slices_to_keep > 0) {
    for (int dest_slice = 0; dest_slice < slices_to_keep; ++dest_slice) {
      // int8_t* dest_slice_data =
      //     feature_data_ + (dest_slice * kFeatureSliceSize);
      const int src_slice = dest_slice + slices_to_drop;
      // const int8_t* src_slice_data =
      //     feature_data_ + (src_slice * kFeatureSliceSize);
      for (int i = 0; i < kFeatureSliceSize; ++i) {
        // dest_slice_data[i] = src_slice_data[i];
      }
    }
  }
  // Any slices that need to be filled in with feature data have their
  // appropriate audio data pulled, and features calculated for that slice.
  if (slices_needed > 0) {
    for (int new_slice = slices_to_keep; new_slice < kFeatureSliceCount;
         ++new_slice) {
      const int new_step = (current_step - kFeatureSliceCount + 1) + new_slice;
      const int32_t slice_start_ms = (new_step * kFeatureSliceStrideMs);
      int16_t* audio_samples = nullptr;
      int audio_samples_size = 0;
      // TODO(petewarden): Fix bug that leads to non-zero slice_start_ms
      GetAudioSamples(error_reporter, (slice_start_ms > 0 ? slice_start_ms : 0),
                      kFeatureSliceDurationMs, &audio_samples_size,
                      &audio_samples);
      
      size_t num_samples_read;
      TfLiteStatus generate_status = GenerateMicroFeatures(
          error_reporter, audio_samples, audio_samples_size, kFeatureSliceSize,
          new_slice_data, &num_samples_read);
      if (generate_status != kTfLiteOk) {
        return generate_status;
      }
    }
  }
  return kTfLiteOk;
}

void getAudSam(int start_ms, int duration_ms, int* audio_samples_size) {
    *audio_samples_size = 512
}