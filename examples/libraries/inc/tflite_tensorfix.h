
#ifndef _TFLITE_TENSORFIX_H_
#define _TFLITE_TENSORFIX_H_

#include <cstdint>

#include "tensorflow/lite/c/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/kernels/internal/compatibility.h"
#include "tensorflow/lite/kernels/internal/types.h"
#include "tensorflow/lite/kernels/internal/runtime_shape.h"


namespace tflite {
namespace micro {
template <typename T>
const T* GetTensorDataFix(const TfLiteEvalTensor* tensor) {
    TFLITE_DCHECK(tensor != nullptr);
    return reinterpret_cast<const T*>(tensor->data.raw);
}

// Returns the shape of a TfLiteEvalTensor struct.
const RuntimeShape GetTensorShape(const TfLiteEvalTensor* tensor);

PaddingType RuntimePaddingType(TfLitePadding padding);
}} // micro // lite
#endif