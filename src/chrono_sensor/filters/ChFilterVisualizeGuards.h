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
//
// Private helpers shared by the GLFW-based Chrono::Sensor visualization filters.
//
// Chrono::Sensor visualization windows are secondary debug views. They are frequently opened inside an application
// that already owns a graphics toolkit and a run loop -- the Chrono::Vehicle demos, for instance, drive an Irrlicht
// window on the same thread that later runs the sensor filter graph. The guards below make sure that opening and
// drawing into those debug windows never leaves global state belonging to the host toolkit disturbed.
//
// This header is internal to the Chrono::Sensor build; it is not installed and must not be included from a public
// Chrono header.
//
// =============================================================================

#ifndef CHFILTERVISUALIZEGUARDS_H
#define CHFILTERVISUALIZEGUARDS_H

#if defined(USE_SENSOR_GLFW) && defined(__APPLE__)
    // CGLGetCurrentContext / CGLSetCurrentContext are formally deprecated along with the rest of macOS OpenGL, but
    // they remain the only way to observe and restore the context that a non-GLFW toolkit made current.
    #ifndef GL_SILENCE_DEPRECATION
        #define GL_SILENCE_DEPRECATION
    #endif
    #include <OpenGL/OpenGL.h>

    #include <dlfcn.h>
    #include <objc/message.h>
    #include <objc/runtime.h>

    #include <cstring>
#endif

namespace chrono {
namespace sensor {

#if defined(USE_SENSOR_GLFW) && defined(__APPLE__)

namespace detail {

/// Address of AppKit's NSApp global, or nullptr when AppKit is not loaded or no NSApplication exists yet.
/// Deliberately avoids +[NSApplication sharedApplication], which would instantiate an NSApplication as a side effect.
inline id GetRunningNSApp() {
    static id* nsapp = reinterpret_cast<id*>(dlsym(RTLD_DEFAULT, "NSApp"));
    return nsapp ? *nsapp : nullptr;
}

inline id GetNSAppDelegate(id app) {
    using MsgSend = id (*)(id, SEL);
    return reinterpret_cast<MsgSend>(objc_msgSend)(app, sel_registerName("delegate"));
}

inline void SetNSAppDelegate(id app, id delegate) {
    using MsgSend = void (*)(id, SEL, id);
    reinterpret_cast<MsgSend>(objc_msgSend)(app, sel_registerName("setDelegate:"), delegate);
}

/// True if this Cocoa window belongs to GLFW.
///
/// GLFW installs a content view of its own class in every window it creates, so the class name of the content view
/// identifies the owner without needing GLFW's native-access header or a registry of our own windows.
inline bool IsGlfwWindow(id window) {
    if (!window)
        return false;
    using MsgSend = id (*)(id, SEL);
    id content = reinterpret_cast<MsgSend>(objc_msgSend)(window, sel_registerName("contentView"));
    if (!content)
        return false;
    const char* name = class_getName(object_getClass(content));
    return name && std::strncmp(name, "GLFW", 4) == 0;
}

}  // namespace detail

/// Restore the NSApplication delegate that was installed when the guard was created.
/// A macOS process has exactly one application delegate, and glfwInit() unconditionally installs its own, so opening a
/// sensor window alongside a toolkit that already drives the Cocoa run loop silently steals that toolkit's delegate.
class CocoaAppDelegateGuard {
  public:
    CocoaAppDelegateGuard() : m_app(detail::GetRunningNSApp()), m_delegate(m_app ? detail::GetNSAppDelegate(m_app) : nullptr) {}

    ~CocoaAppDelegateGuard() {
        // Irrlicht, for one, sends -isQuit to whatever delegate is installed from CIrrDeviceMacOSX::run(); reaching
        // GLFW's delegate there kills the application with an NSInvalidArgumentException ("unrecognized selector").
        // When nothing else had claimed the delegate there is nothing to put back, and GLFW keeps its own.
        if (!m_app || !m_delegate)
            return;
        if (detail::GetNSAppDelegate(m_app) != m_delegate)
            detail::SetNSAppDelegate(m_app, m_delegate);
    }

    CocoaAppDelegateGuard(const CocoaAppDelegateGuard&) = delete;
    CocoaAppDelegateGuard& operator=(const CocoaAppDelegateGuard&) = delete;

  private:
    id m_app;
    id m_delegate;
};

/// Restore the OpenGL context that was current on this thread when the guard was created.
/// The filter graph runs on the thread that calls ChSensorManager::Update(), usually the one driving the host
/// toolkit's renderer, so leaving a GLFW context current there sends the toolkit's next draw into a foreign context.
class GLContextGuard {
  public:
    GLContextGuard() : m_prev(CGLGetCurrentContext()) {}

    ~GLContextGuard() {
        // With Irrlicht that means drawing with buffer object names from another context, which segfaults inside the
        // GL driver. If no context was current beforehand -- the standalone Chrono::Sensor case -- there is nothing to
        // restore and the GLFW context stays current exactly as before.
        if (m_prev && CGLGetCurrentContext() != m_prev)
            CGLSetCurrentContext(m_prev);
    }

    GLContextGuard(const GLContextGuard&) = delete;
    GLContextGuard& operator=(const GLContextGuard&) = delete;

  private:
    CGLContextObj m_prev;
};

/// Pump the Cocoa event queue for the GLFW debug windows without consuming another toolkit's events.
/// glfwPollEvents() drains the whole shared NSApplication queue, including events belonging to windows GLFW does not
/// own, which breaks a host toolkit that polls the same queue itself. Dispatch only GLFW's own events and put the rest
/// back for the host toolkit to collect. With no foreign window present every event is dispatched as GLFW would.
inline void PollGlfwEventsPreservingHostEvents() {
    id app = detail::GetRunningNSApp();
    if (!app) {
        glfwPollEvents();
        return;
    }

    static id* default_mode = reinterpret_cast<id*>(dlsym(RTLD_DEFAULT, "NSDefaultRunLoopMode"));
    if (!default_mode) {
        glfwPollEvents();
        return;
    }

    using NextEvent = id (*)(id, SEL, unsigned long, id, id, bool);
    using MsgSend = id (*)(id, SEL);
    using SendEvent = void (*)(id, SEL, id);
    using PostEvent = void (*)(id, SEL, id, bool);

    id distant_past = reinterpret_cast<MsgSend>(objc_msgSend)(reinterpret_cast<id>(objc_getClass("NSDate")), sel_registerName("distantPast"));

    // Irrlicht polls this same queue from CIrrDeviceMacOSX::run(), so whichever of the two runs first in a frame wins.
    // Roughly half of all keystrokes were swallowed by GLFW instead of reaching Irrlicht, and because a bare NSWindow
    // has no responder implementing keyDown:, AppKit answered each stolen key with -[NSResponder noResponderFor:] and
    // the system alert sound: a demo that beeped on every keypress and obeyed only some of them.
    //
    // Events belonging to the host toolkit, held until the drain loop has finished. Deferring the re-post is what
    // keeps this terminating; re-posting inside the loop would immediately hand the event back to us.
    const int kMaxDeferred = 256;
    id deferred[kMaxDeferred];
    int num_deferred = 0;

    for (;;) {
        id event = reinterpret_cast<NextEvent>(objc_msgSend)(app, sel_registerName("nextEventMatchingMask:untilDate:inMode:dequeue:"), ~0UL, distant_past, *default_mode, true);
        if (!event)
            break;

        id window = reinterpret_cast<MsgSend>(objc_msgSend)(event, sel_registerName("window"));

        // No window (application-level events) or one of ours: dispatch now, exactly as GLFW would.
        if (!window || detail::IsGlfwWindow(window) || num_deferred == kMaxDeferred) {
            reinterpret_cast<SendEvent>(objc_msgSend)(app, sel_registerName("sendEvent:"), event);
            continue;
        }

        // Someone else's window: retain it across the loop and hand it back afterwards.
        deferred[num_deferred++] = reinterpret_cast<MsgSend>(objc_msgSend)(event, sel_registerName("retain"));
    }

    for (int i = 0; i < num_deferred; i++) {
        reinterpret_cast<PostEvent>(objc_msgSend)(app, sel_registerName("postEvent:atStart:"), deferred[i], false);
        reinterpret_cast<MsgSend>(objc_msgSend)(deferred[i], sel_registerName("release"));
    }
}

#elif defined(USE_SENSOR_GLFW)

/// Elsewhere no other toolkit shares the GLFW event queue, so plain polling is correct.
inline void PollGlfwEventsPreservingHostEvents() {
    glfwPollEvents();
}

#endif

#if !(defined(USE_SENSOR_GLFW) && defined(__APPLE__))

/// No-op outside macOS: no other platform has a process-wide application delegate to protect.
/// The user-provided destructor keeps -Wunused-variable quiet at the use sites.
struct CocoaAppDelegateGuard {
    ~CocoaAppDelegateGuard() {}
};

/// No-op outside macOS. The same context-leak issue exists in principle on WGL and GLX, but Chrono::Sensor only
/// shares a thread with another renderer on the Metal/Vulkan back ends, and only the macOS path is exercised here.
struct GLContextGuard {
    ~GLContextGuard() {}
};

#endif

}  // namespace sensor
}  // namespace chrono

#endif
