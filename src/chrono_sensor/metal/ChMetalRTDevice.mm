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

#import <Metal/Metal.h>

#include <iostream>

#include "chrono_sensor/metal/ChMetalRTDevice.h"

namespace chrono {
namespace sensor {

ChMetalRTDevice::ChMetalRTDevice(const ChMetalRTDeviceConfig& config) : m_config(config) {
    id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
    if (!dev) {
        if (m_config.verbose)
            std::cerr << "Chrono::Sensor Metal RT: no Metal device available\n";
        return;
    }
    if (!dev.supportsRaytracing) {
        if (m_config.verbose)
            std::cerr << "Chrono::Sensor Metal RT: device does not support hardware ray tracing\n";
        // keep the device for the (future) CPU fallback path
    }
    id<MTLCommandQueue> queue = [dev newCommandQueue];
    // Hand ownership to the opaque void* members (ARC-bridged, released in dtor).
    m_device = (__bridge_retained void*)dev;
    m_queue = (__bridge_retained void*)queue;
    if (m_config.verbose)
        std::cout << "Chrono::Sensor Metal RT device: " << [dev.name UTF8String] << std::endl;
}

ChMetalRTDevice::~ChMetalRTDevice() {
    if (m_queue) {
        id<MTLCommandQueue> queue = (__bridge_transfer id<MTLCommandQueue>)m_queue;
        (void)queue;
        m_queue = nullptr;
    }
    if (m_device) {
        id<MTLDevice> dev = (__bridge_transfer id<MTLDevice>)m_device;
        (void)dev;
        m_device = nullptr;
    }
}

}  // namespace sensor
}  // namespace chrono
