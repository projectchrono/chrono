// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// Function that converts uchar RGBA8 image to float3 image.
/// @param bufIn Input device pointer in uchar format.
/// @param bufOut Output device pointer in float format.
/// @param h The height of the image.
/// @param w The width of the image.
void preprocess_RGBA8_to_FLOAT3(void* bufIn, void* bufOut, int h, int w);

/// Function that converts uchar RGBA8 image to float4 image.
/// @param bufIn Input device pointer in uchar format.
/// @param bufOut Output device pointer in float format.
/// @param num_entries The number of entries in the image (c*h*w).
void preprocess_RGBA8_to_FLOAT4(void* bufIn, void* bufOut, int num_entries);

/// Function that converts float4 image to uchar RGBA8 image.
/// @param bufIn Input device pointer in float format.
/// @param bufOut Output device pointer in uchar format.
/// @param num_entries The number of entries in the image (c*h*w).
void postprocess_FLOAT4_to_RGBA8(void* bufIn, void* bufOut, int num_entries);

/// Function that converts uchar RGBA8 image to float4 image with channels first.
/// @param bufIn Input device pointer in uchar format.
/// @param bufOut Output device pointer in float format.
/// @param c Number of channels in the image.
/// @param h Height of the image.
/// @param w Width of the image.
void preprocess_RGBA8_to_FLOAT4_CHW(void* bufIn, void* bufOut, int c, int h, int w);

/// Function that normalized input data in float format.
/// @param bufIn Input device pointer in float format.
/// @param c Number of channels in the image.
/// @param h Height of the image.
/// @param w Width of the image.
void preprocess_normalize_float(void* buf, float add, float mult, int c, int h, int w);

/// Function that converts float4 image to uchar RGBA8 image with channels first.
/// @param bufIn Input device pointer in float format.
/// @param bufOut Output device pointer in uchar format.
/// @param c Number of channels in the image.
/// @param h Height of the image.
/// @param w Width of the image.
void postprocess_FLOAT4_to_RGBA8_CHW(void* bufIn, void* bufOut, int c, int h, int w);

/// @}

}  // namespace sensor
}  // namespace chrono
