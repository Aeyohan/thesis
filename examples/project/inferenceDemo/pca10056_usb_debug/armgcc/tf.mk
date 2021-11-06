TF_HC_DIR = C:/Users/Aeyohan/Documents/work/ENGG4811/code/Thesis/thesis/examples/libraries/tflite-micro
TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/c/common.c\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/activations.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/activations_common.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/resize_bilinear.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/l2norm.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/conv.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pooling.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depthwise_conv.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fully_connected.cpp\
  $(TF_HC_DIR)/tensorflow/lite/kernels/kernel_util.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/kernel_util.cpp\
  $(TF_HC_DIR)/tensorflow/lite/kernels/kernel_util.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/all_ops_resolver.cpp\
  $(TF_HC_DIR)/tensorflow/lite/core/api/error_reporter.cpp\
  $(TF_HC_DIR)/tensorflow/lite/core/api/flatbuffer_conversions.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/system_setup.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_error_reporter.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_interpreter.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_string.cpp\
  $(TF_HC_DIR)/tensorflow/lite/micro/debug_log.cpp\

#  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/activations_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/add_n_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/add_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arg_min_max_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/call_once_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cast_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceil_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/circular_buffer_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/comparisons_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/concatenation_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/conv_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/conv_test_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cumsum_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depthwise_conv_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depth_to_space_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/dequantize_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/detection_postprocess_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/elementwise_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/elementwise_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/elu_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/expand_dims_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/exp_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fill_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor_div_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor_mod_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fully_connected_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/gather_nd_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/gather_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/hard_swish_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/if_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/l2norm_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/l2_pool_2d_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/leaky_relu_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logical_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/log_softmax_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logistic_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/maximum_minimum_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/mul_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/neg_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pack_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pad_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pooling_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/prelu_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/quantization_util_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/quantize_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/reduce_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/reshape_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/resize_bilinear_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/resize_nearest_neighbor_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/flatbuffer_utils_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/memory_arena_threshold_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/round_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/shape_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/slice_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/softmax_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/space_to_batch_nd_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/space_to_depth_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/split_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/split_v_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/squeeze_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/strided_slice_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/sub_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/svdf_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/tanh_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/transpose_conv_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/transpose_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/unpack_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/zeros_like_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/memory_helpers_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_allocator_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_error_reporter_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_interpreter_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_mutable_op_resolver_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_resource_variable_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_string_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_time_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_utils_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/recording_micro_allocator_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/recording_simple_memory_allocator_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/simple_memory_allocator_test.cpp \





TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/activations.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/activations_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/add.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/add_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/add_n.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arg_min_max.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/assign_variable.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/batch_to_space_nd.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/batch_to_space_nd_test.cpp 

TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/call_once.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cast.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceil.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/circular_buffer.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/circular_buffer_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/circular_buffer_flexbuffers_generated_data.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/comparisons.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/concatenation.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/conv.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/conv_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cumsum.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depthwise_conv.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depthwise_conv_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/depth_to_space.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/detection_postprocess.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/detection_postprocess_flexbuffers_generated_data.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/elementwise.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/elu.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ethosu.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/expand_dims.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fill.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor_div.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/floor_mod.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fully_connected.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/fully_connected_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/gather.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/gather_nd.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/if.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/kernel_runner.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/kernel_util.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/l2norm.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/l2_pool_2d.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/leaky_relu.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/leaky_relu_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logical.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logical_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logistic.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/logistic_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/log_softmax.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/maximum_minimum.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/mul.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/mul_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/neg.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pack.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pad.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pooling.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/pooling_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/prelu.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/prelu_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/quantize.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/quantize_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/read_variable.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/reduce.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/reshape.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/resize_bilinear.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/resize_nearest_neighbor.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/round.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/shape.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/slice.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/softmax.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/softmax_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/space_to_batch_nd.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/space_to_depth.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/split.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/split_v.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/squeeze.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/strided_slice.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/sub.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/sub_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/svdf.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/svdf_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/tanh.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/transpose.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/transpose_conv.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/unpack.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/var_handle.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/zeros_like.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/hard_swish.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/hard_swish_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/dequantize.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/dequantize_common.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/kernels/exp.cpp \
  $(TF_HC_DIR)/tensorflow/lite/core/api/error_reporter.cpp \
  $(TF_HC_DIR)/tensorflow/lite/core/api/flatbuffer_conversions.cpp \
  $(TF_HC_DIR)/tensorflow/lite/core/api/op_resolver.cpp \
  $(TF_HC_DIR)/tensorflow/lite/core/api/tensor_utils.cpp \
  
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/fft_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/log_scale_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/window_test.cpp
TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/kernels/kernel_util.cpp \
  $(TF_HC_DIR)/tensorflow/lite/kernels/internal/quantization_util.cpp \
  $(TF_HC_DIR)/tensorflow/lite/kernels/internal/reference/portable_tensor_utils.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/all_ops_resolver.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/debug_log.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/flatbuffer_utils.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/memory_helpers.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_allocator.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_error_reporter.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_graph.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_interpreter.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_profiler.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_resource_variable.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_string.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_time.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/micro_utils.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/mock_micro_graph.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/recording_micro_allocator.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/recording_simple_memory_allocator.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/simple_memory_allocator.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/system_setup.cpp \

  # $(TF_HC_DIR)/tensorflow/lite/micro/testing_helpers_test.cpp \
  $(TF_HC_DIR)/tensorflow/lite/micro/test_helpers.cpp \

# TF_CPP_SRC += \
  # $(TF_HC_DIR)/tensorflow/lite/micro/arc_custom/micro_time.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/arc_custom/system_setup.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/arc_emsdp/debug_log.cpp 

  # $(TF_HC_DIR)/tensorflow/lite/micro/benchmarks/keyword_benchmark.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/benchmarks/keyword_benchmark_8bit.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/benchmarks/person_detection_benchmark.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/bluepill/debug_log.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/ceva/micro_time.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/ceva/system_setup.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/chre/debug_log.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/cortex_m_corstone_300/micro_time.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/cortex_m_corstone_300/system_setup.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/cortex_m_generic/debug_log.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/cortex_m_generic/micro_time.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/constants.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/hello_world_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/main_functions.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/output_handler.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/output_handler_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/esp/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/hello_world/zephyr_riscv/src/assert.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/accelerometer_handler.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/accelerometer_handler_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/gesture_predictor.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/gesture_predictor_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/magic_wand_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/main_functions.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/output_handler.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/output_handler_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/ring_micro_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/slope_micro_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/zephyr_riscv/src/accelerometer_handler.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/magic_wand/zephyr_riscv/src/assert.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/memory_footprint/baseline_memory_footprint.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/memory_footprint/interpreter_memory_footprint.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/audio_provider_mock.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/audio_provider_mock_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/audio_provider_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/command_responder.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/command_responder_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/feature_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/feature_provider_mock_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/feature_provider_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/main_functions.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_speech_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/recognize_commands.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/recognize_commands_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/ceva/audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/ceva/main_functions.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/disco_f746ng/audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/disco_f746ng/command_responder.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/disco_f746ng/timer.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/esp/audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/esp/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/micro_features_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/micro_features_generator_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/micro_model_settings.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/no_feature_data_slice.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/no_micro_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/yes_feature_data_slice.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/micro_features/yes_micro_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/osx/audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/model.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/no_power_spectrum_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/no_simple_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/simple_features_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/simple_features_generator_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/simple_model_settings.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/yes_power_spectrum_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/yes_simple_features_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/CMSIS/simple_features_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/simple_features/fixed_point/simple_features_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/spresense/src/spresense_audio_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/spresense/src/spresense_command_responder.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/network_tester/network_tester_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/detection_responder.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/detection_responder_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/image_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/image_provider_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/main_functions.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/model_settings.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/person_detection_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/esp/image_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/esp/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/spresense/src/spresense_image_provider.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/hexagon/micro_time.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/hexagon/system_setup.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/himax_we1_evb/debug_log.cpp 

  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/add.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/conv_slicing_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/depthwise_conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/depthwise_conv_slicing_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/fully_connected.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/fully_connected_slicing_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/mli_interface.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/mli_interface_mli_20.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/mli_slicers.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/pooling.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/pooling_slicing_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/scratch_buffers.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/arc_mli/scratch_buf_mgr.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/ceva_common.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/depthwise_conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/fully_connected.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/quantize.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ceva/softmax.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/add.cpp 
  
  # TF_CPP_SRC += \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/depthwise_conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/fully_connected.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/pooling.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/softmax.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/svdf.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/cmsis_nn/mul.cpp \

  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/ethos_u/ethosu.cpp 
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/integration_tests/seanet/conv/integration_tests.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/testdata/conv_test_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/test_data_generation/generate_circular_buffer_flexbuffers_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/test_data_generation/generate_detection_postprocess_flexbuffers_data.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/vexriscv/depthwise_conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/conv_hifi.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/conv_hifimini.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/conv_int8_reference.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/depthwise_conv.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/depthwise_conv_hifi.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/depthwise_conv_hifimini.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/fully_connected.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/lstm_eval.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/lstm_eval_hifi.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/pooling.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/quantize.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/softmax.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/softmax_int8_int16.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/svdf.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/kernels/xtensa/unidirectional_sequence_lstm.cpp 
  
TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/greedy_memory_planner.cpp \
  
#   $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/greedy_memory_planner_test.cpp \
#   $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/linear_memory_planner.cpp \
#   $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/linear_memory_planner_test.cpp \
#   $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/non_persistent_buffer_planner_shim.cpp \
#   $(TF_HC_DIR)/tensorflow/lite/micro/memory_planner/non_persistent_buffer_planner_shim_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/riscv32_mcu/debug_log.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/spresense/compiler_specific.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/spresense/debug_log.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/stm32f4/debug_log.cpp 
  # $(TF_HC_DIR)/tensorflow/lite/micro/testing/test_conv_model.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/testing/util_test.cpp \

# TF_CPP_SRC += \
#   $(TF_HC_DIR)/tensorflow/lite/micro/tools/ci_build/binary_size_test/binary_size_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/android/jni/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/conan/test_package/test_package.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/samples/greeter/client.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/samples/greeter/server.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/src/compiler/cpp_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/src/compiler/go_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/src/compiler/java_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/src/compiler/python_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/src/compiler/swift_generator.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/tests/grpctest.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/grpc/tests/message_builder_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/samples/sample_bfbs.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/samples/sample_binary.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/samples/sample_text.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/samples/android/jni/main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/code_generators.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/flatc.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/flatc_main.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/flathash.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_cpp.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_csharp.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_dart.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_fbs.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_go.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_grpc.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_java.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_json_schema.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_js_ts.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_kotlin.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_lobster.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_lua.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_php.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_python.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_rust.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_swift.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_gen_text.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/idl_parser.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/reflection.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/src/util.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/monster_test.grpc.fb.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/native_type_test_impl.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/test_assert.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/test_builder.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/cpp17/test_cpp17.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/fuzzer/flatbuffers_parser_fuzzer.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/fuzzer/flatbuffers_scalar_fuzzer.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/fuzzer/flatbuffers_verifier_fuzzer.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/flatbuffers/tests/fuzzer/scalar_debug.cpp \
  
TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/schema/schema_utils.cpp\

TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/c/common.c 
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/fft_io.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/filterbank_io.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_io.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_main.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/log_scale_io.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_io.c \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/window_io.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_memmap_main.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/filterbank_test.cpp \
  # $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_memmap_generator.c \

TF_CPP_SRC += \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/filterbank.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/filterbank_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_handler.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/frontend_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/log_lut.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/log_scale.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/log_scale_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/noise_reduction.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/window.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/window_util.c \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/fft.cpp \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/fft_util.cpp \
  $(TF_HC_DIR)/tensorflow/lite/experimental/microfrontend/lib/kiss_fft_int16.cpp \
  
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/micro_speech/esp/ringbuf.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/examples/person_detection/esp/app_camera_esp.c \

TF_CPP_SRC += \

  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/kiss_fft.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/benchfftw.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/benchkiss.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/doit.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/pstats.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/test_real.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/test_vs_dft.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/test/twotonetest.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/fftutil.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kfc.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fastfir.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftnd.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftndr.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftr.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/psdpng.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_assert/assert_backend_compile_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_base64/base64_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_boot_armv7m/core_init.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_checksum/crc16_ccitt_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_checksum/crc32_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_chrono/system_clock_facade_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_log/basic_log_test_plain_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_log_null/test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_status/status_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_sync/binary_semaphore_facade_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_sync/counting_semaphore_facade_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_sync/mutex_facade_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_sync/spin_lock_facade_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_tokenizer/argument_types_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_tokenizer/global_handlers_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_tokenizer/tokenize_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_tokenizer/py/elf_reader_test_binary.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_trace/trace_backend_compile_test_c.c \
  # $(TF_HC_DIR)/tensorflow/lite/micro/tools/make/downloads/pigweed/pw_varint/varint_test_c.c \
  
