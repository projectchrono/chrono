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
// Lightweight on-window performance overlay shared by the Metal visualize
// filters. Tracks display FPS and real-time factor (RTF = d(sim)/d(wall)) and
// draws a compact HUD in the top-left corner (also echoed to stdout ~1/s).
// Pure C++ interface (PIMPL) so it can be included by ObjC++ .mm files; the
// Metal/AppKit types live entirely in the .mm.
// =============================================================================

#ifndef CH_METAL_PERF_OVERLAY_H
#define CH_METAL_PERF_OVERLAY_H

#include <string>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class MetalPerfOverlay {
  public:
    MetalPerfOverlay();
    ~MetalPerfOverlay();

    /// Call once per displayed frame. simTime is the buffer's sim timestamp (s);
    /// label names the window (e.g. "driver POV"). Updates FPS/RTF estimates.
    void Tick(double simTime, const std::string& label);

    /// Draw the HUD into an active render encoder. device and encoder are
    /// id<MTLDevice> / id<MTLRenderCommandEncoder>; drawableW/H are in pixels.
    void Draw(void* device, void* encoder, double drawableW, double drawableH);

    double FPS() const;
    double RTF() const;

  private:
    struct Impl;
    Impl* p = nullptr;
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
