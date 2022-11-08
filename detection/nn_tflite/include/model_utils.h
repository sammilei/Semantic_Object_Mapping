// Copyright 2020 Google LLC
// Modified by Nam Vu 2020

#ifndef EDGETPU_CPP_EXAMPLES_MODEL_UTILS_H_
#define EDGETPU_CPP_EXAMPLES_MODEL_UTILS_H_

#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "edgetpu.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

#include <ros/ros.h>

namespace coral {

//! Bounding box of the detected object.
typedef struct {
  float xmin, ymin, xmax, ymax, score;
  int label;
} BoundingBox;

// Builds tflite Interpreter capable of running Edge TPU model.
std::unique_ptr<tflite::Interpreter>
buildEdgeTpuInterpreter(const tflite::FlatBufferModel& model,
                        edgetpu::EdgeTpuContext* edgetpu_context);

// Builds tflite Interpreter for normal CPU model.
std::unique_ptr<tflite::Interpreter>
buildInterpreter(const tflite::FlatBufferModel& model);

// Runs inference using given `interpreter`
std::vector<BoundingBox> runInference(const std::vector<uint8_t>& input_data,
                                      tflite::Interpreter* interpreter);

// Returns input tensor shape in the form {height, width, channels}.
std::array<int, 3> getInputShape(const tflite::Interpreter& interpreter,
                                 int index);

} // namespace coral
#endif // EDGETPU_CPP_EXAMPLES_MODEL_UTILS_H_
