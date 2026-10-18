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
// macOS-specific helper for the Chrono Irrlicht module.
//
// Irrlicht is consumed as a prebuilt library, so its Cocoa device cannot be patched from Chrono. This helper reaches
// the NSView that Irrlicht renders into and adjusts it from the outside. It is implemented in Objective-C++
// (ChIrrMacOS.mm) and exposed here with a plain C++ interface, so that the rest of the module can remain plain C++.
//
// Only compiled and called on Apple platforms.
// =============================================================================

#ifndef CH_IRR_MACOS_H
#define CH_IRR_MACOS_H

namespace chrono {
namespace irrlicht {

/// Make the OpenGL surface Irrlicht renders into match the logical (point) size of its window, which on a Retina
/// display it otherwise exceeds by backingScaleFactor. Call once the device exists, with its GL context current.
/// Returns the backing scale factor, or 0 if the view could not be located and nothing was changed.
double ChIrrMacOSMatchGLSurfaceToWindowSize();

/// Promote this process to a regular foreground application and give Irrlicht's window keyboard focus; an unbundled
/// executable is otherwise background-only and its window, never being key, receives no key events. Call once the
/// device exists. Returns true if the process is a regular foreground application on return.
bool ChIrrMacOSActivateApplication();

}  // namespace irrlicht
}  // namespace chrono

#endif
