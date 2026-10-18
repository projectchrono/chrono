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
// macOS-specific helper for the Chrono Irrlicht module. See ChIrrMacOS.h.
// =============================================================================

#include "chrono_irrlicht/ChIrrMacOS.h"

#import <Cocoa/Cocoa.h>

namespace chrono {
namespace irrlicht {

// Locate the NSView that Irrlicht's OpenGL driver draws into.
//
// Irrlicht 1.8's SExposedVideoData has no macOS member (only Win32 and X11), so the view cannot be obtained from
// IVideoDriver::getExposedVideoData(). Cocoa can supply it instead: CIrrDeviceMacOSX creates an NSOpenGLContext,
// attaches it to its window's content view and makes it current, and leaves it current. The current context's view is
// therefore exactly Irrlicht's render target.
static NSView* FindIrrlichtGLView() {
    NSOpenGLContext* context = [NSOpenGLContext currentContext];
    if (context) {
        NSView* view = [context view];
        if (view)
            return view;
    }

    // Fallback, in case a future Irrlicht leaves a different context current: Irrlicht's is the only window this
    // process owns when the device is created.
    for (NSWindow* window in [NSApp windows]) {
        if ([window isVisible] && [window contentView])
            return [window contentView];
    }

    return nil;
}

double ChIrrMacOSMatchGLSurfaceToWindowSize() {
    double scale = 0.0;

    @autoreleasepool {
        NSView* view = FindIrrlichtGLView();
        if (!view)
            return 0.0;

        NSWindow* window = [view window];
        scale = window ? (double)[window backingScaleFactor] : 1.0;

        // Drop the high-resolution backing store: this shrinks the surface back to the window's point size, so
        // Irrlicht's viewport covers all of it and macOS scales the result up to fill the window. A no-op on a
        // non-Retina display, where the surface already matches the window's point size.
        [view setWantsBestResolutionOpenGLSurface:NO];

        // Force the context to reallocate its drawable at the new size.
        NSOpenGLContext* context = [NSOpenGLContext currentContext];
        if (context)
            [context update];
    }

    return scale;
}

bool ChIrrMacOSActivateApplication() {
    @autoreleasepool {
        // Irrlicht has already created the shared application; this just retrieves it.
        NSApplication* app = [NSApplication sharedApplication];

        // NSApplicationActivationPolicyRegular is what makes this a normal GUI application, after which its window
        // can take focus like any other. LaunchServices registers a bundle-less executable as background-only
        // ("type=BackgroundOnly" in lsappinfo), and such a process can never become active.
        if ([app activationPolicy] != NSApplicationActivationPolicyRegular)
            [app setActivationPolicy:NSApplicationActivationPolicyRegular];

        [app activateIgnoringOtherApps:YES];

        // Being the active application is not enough on its own: the window has to be the key window before AppKit
        // will route key events to it.
        NSView* view = FindIrrlichtGLView();
        NSWindow* window = view ? [view window] : nil;
        if (window)
            [window makeKeyAndOrderFront:nil];

        return [app activationPolicy] == NSApplicationActivationPolicyRegular;
    }
}

}  // namespace irrlicht
}  // namespace chrono
