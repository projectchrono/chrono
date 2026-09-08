// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
// Owns the MTLDevice + command queue. The header is pure C++ (Metal handles are
// exposed as opaque void* = id<MTLDevice>/id<MTLCommandQueue>) so it can be
// included from ordinary C++ translation units; only the .mm touches Metal.
// =============================================================================

#ifndef CH_METAL_RT_DEVICE_H
#define CH_METAL_RT_DEVICE_H

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/metal/ChMetalRTDefinitions.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class CH_SENSOR_API ChMetalRTDevice {
  public:
    explicit ChMetalRTDevice(const ChMetalRTDeviceConfig& config = {});
    ~ChMetalRTDevice();

    ChMetalRTDevice(const ChMetalRTDevice&) = delete;
    ChMetalRTDevice& operator=(const ChMetalRTDevice&) = delete;

    bool IsValid() const { return m_device != nullptr; }

    /// Opaque id<MTLDevice>; cast with (__bridge id<MTLDevice>) inside .mm code.
    void* GetMTLDevice() const { return m_device; }
    /// Opaque id<MTLCommandQueue>.
    void* GetMTLQueue() const { return m_queue; }

  private:
    ChMetalRTDeviceConfig m_config;
    void* m_device = nullptr;  // retained id<MTLDevice>
    void* m_queue = nullptr;   // retained id<MTLCommandQueue>
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
