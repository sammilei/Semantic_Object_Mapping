// Copyright 2020 Google LLC
// Modified by Nam Vu 2020

#include "model_utils.h"

#include <memory>

#include "tensorflow/lite/builtin_op_data.h"
#include "tensorflow/lite/kernels/register.h"

namespace coral {
std::unique_ptr<tflite::Interpreter>
buildEdgeTpuInterpreter(const tflite::FlatBufferModel& model,
                        edgetpu::EdgeTpuContext* edgetpu_context) {
  tflite::ops::builtin::BuiltinOpResolver resolver;
  resolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());
  std::unique_ptr<tflite::Interpreter> interpreter;
  if (tflite::InterpreterBuilder(model, resolver)(&interpreter) != kTfLiteOk) {
    std::cerr << "Failed to build interpreter." << std::endl;
  }
  // Bind given context with interpreter.
  interpreter->SetExternalContext(kTfLiteEdgeTpuContext, edgetpu_context);
  interpreter->SetNumThreads(1);
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    std::cerr << "Failed to allocate tensors." << std::endl;
  }
  return interpreter;
}

std::unique_ptr<tflite::Interpreter>
buildInterpreter(const tflite::FlatBufferModel& model) {
  std::unique_ptr<tflite::Interpreter> interpreter;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  if (tflite::InterpreterBuilder(model, resolver)(&interpreter) != kTfLiteOk) {
    std::cerr << "Failed to build interpreter." << std::endl;
  }
  interpreter->SetNumThreads(1);
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    std::cerr << "Failed to allocate tensors." << std::endl;
  }
  return interpreter;
}

std::vector<coral::BoundingBox>
runInference(const std::vector<uint8_t>& input_data,
             tflite::Interpreter* interpreter) {
  uint8_t* input = interpreter->typed_input_tensor<uint8_t>(0);

  std::memcpy(input, input_data.data(), input_data.size());

  interpreter->Invoke();

  const auto& output_indices = interpreter->outputs();
  assert(output_indices.size() == 4);

  coral::BoundingBox bb;

  const auto* bb_data_tensor = interpreter->tensor(output_indices[0]);
  const auto* labels_tensor = interpreter->tensor(output_indices[1]);
  const auto* scores_tensor = interpreter->tensor(output_indices[2]);
  const auto* num_dets_tensor = interpreter->tensor(output_indices[3]);

  assert((bb_data_tensor != nullptr) && (labels_tensor != nullptr) &&
         (scores_tensor != nullptr) && (num_dets_tensor != nullptr));
  assert((bb_data_tensor->type == kTfLiteFloat32) &&
         (labels_tensor->type == kTfLiteFloat32) &&
         (scores_tensor->type == kTfLiteFloat32) &&
         (num_dets_tensor->type == kTfLiteFloat32));

  const float* bb_data = interpreter->typed_output_tensor<float>(0);
  const float* labels = interpreter->typed_output_tensor<float>(1);
  const float* scores = interpreter->typed_output_tensor<float>(2);
  // Convert from quantized value
  int num_dets = (int)interpreter->typed_output_tensor<float>(3)[0];
  std::vector<coral::BoundingBox> output_data(num_dets);

  // Check for enough values
  assert((bb_data_tensor->bytes / sizeof(float) >= num_dets * 4) &&
         (labels_tensor->bytes / sizeof(float) >= num_dets) &&
         (scores_tensor->bytes / sizeof(float) >= num_dets));
  for (int i = 0; i < num_dets; ++i) {
    bb.ymin = bb_data[4 * i];
    bb.xmin = bb_data[4 * i + 1];
    bb.ymax = bb_data[4 * i + 2];
    bb.xmax = bb_data[4 * i + 3];
    bb.label = (int)labels[i];
    bb.score = scores[i];
    output_data.at(i) = bb;
  }
  return output_data;
}

std::array<int, 3> getInputShape(const tflite::Interpreter& interpreter,
                                 int index) {
  const int tensor_index = interpreter.inputs()[index];
  const TfLiteIntArray* dims = interpreter.tensor(tensor_index)->dims;
  return std::array<int, 3>{dims->data[1], dims->data[2], dims->data[3]};
}

} // namespace coral
