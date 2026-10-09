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

#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>

#include "chrono_sensor/metal/ChFilterMetalVisualize.h"
#include "chrono_sensor/metal/ChMetalPerfOverlay.h"
#include "chrono_sensor/metal/ChOrbitCameraControl.h"
#include "chrono_sensor/sensors/ChSensor.h"

namespace chrono {
namespace sensor {

static const char* kBlitMSL = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct VOut { float4 pos [[position]]; float2 uv; };
vertex VOut vmain(uint vid [[vertex_id]]) {
    float2 p[3] = { float2(-1,-1), float2(3,-1), float2(-1,3) };
    VOut o; o.pos = float4(p[vid], 0, 1);
    // Sensor host buffers are BOTTOM-UP (row 0 = bottom), matching OptiX's camera_raygen.cu and
    // what ChFilterSave assumes (it calls stbi_flip_vertically_on_write). Clip-space y is already
    // bottom-up, so sample straight through -- no vertical flip here.
    float2 t = p[vid]*0.5 + 0.5; o.uv = t; return o;
}
fragment float4 fmain(VOut in [[stage_in]], texture2d<float> tex [[texture(0)]]) {
    constexpr sampler s(filter::linear); return tex.sample(s, in.uv);
}
)MSL";

struct ChFilterMetalVisualize::Impl {
    NSWindow* window = nil;
    CAMetalLayer* layer = nil;
    id<MTLDevice> dev = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLRenderPipelineState> pso = nil;
    id<MTLTexture> tex = nil;
    int texW = 0, texH = 0;
    MetalPerfOverlay overlay;
    std::string label;
    // interactive orbit input (accumulated in pump(), consumed each frame)
    ChOrbitCameraControl orbit;
    double inDX = 0, inDY = 0, inScroll = 0;
    bool dragActive = false;  // true only while a left-drag that STARTED in the content view is in progress
    void consumeInput(double& dx, double& dy, double& sc) {
        dx = inDX;
        dy = inDY;
        sc = inScroll;
        inDX = inDY = inScroll = 0;
    }

    void ensureWindow(int w, int h, const std::string& title) {
        if (window)
            return;
        label = title;
        dev = MTLCreateSystemDefaultDevice();
        queue = [dev newCommandQueue];
        NSError* e = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:@(kBlitMSL) options:nil error:&e];
        MTLRenderPipelineDescriptor* rpd = [MTLRenderPipelineDescriptor new];
        rpd.vertexFunction = [lib newFunctionWithName:@"vmain"];
        rpd.fragmentFunction = [lib newFunctionWithName:@"fmain"];
        rpd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        pso = [dev newRenderPipelineStateWithDescriptor:rpd error:&e];

        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        NSRect fr = NSMakeRect(0, 0, w, h);
        window = [[NSWindow alloc] initWithContentRect:fr
                                             styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable)
                                               backing:NSBackingStoreBuffered
                                                 defer:NO];
        [window setTitle:[NSString stringWithUTF8String:title.c_str()]];
        [window center];
        NSView* v = [[NSView alloc] initWithFrame:fr];
        v.wantsLayer = YES;
        CGFloat s = [window backingScaleFactor];
        if (s < 1)
            s = 1;
        layer = [CAMetalLayer layer];
        layer.device = dev;
        layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
        layer.framebufferOnly = YES;
        layer.contentsScale = s;
        layer.drawableSize = CGSizeMake(w * s, h * s);
        v.layer = layer;
        layer.frame = v.bounds;                                                   // a hosted CAMetalLayer does NOT auto-track the view; without this
        v.layerContentsRedrawPolicy = NSViewLayerContentsRedrawDuringViewResize;  // the drawable composites into only part of the window
        window.contentView = v;
        [window makeKeyAndOrderFront:nil];
        [NSApp activateIgnoringOtherApps:YES];
    }

    void pump() {
        NSEvent* e;
        while ((e = [NSApp nextEventMatchingMask:NSEventMaskAny untilDate:[NSDate distantPast] inMode:NSDefaultRunLoopMode dequeue:YES])) {
            // Only orbit for drags that BEGIN inside the content view -- otherwise dragging the title bar to
            // move the window would also spin the camera (all LeftMouseDragged events land in this queue).
            if (e.type == NSEventTypeLeftMouseDown) {
                dragActive = (e.window == window) && NSPointInRect(e.locationInWindow, window.contentView.frame);
            } else if (e.type == NSEventTypeLeftMouseUp) {
                dragActive = false;
            } else if (e.type == NSEventTypeLeftMouseDragged) {
                if (dragActive) {
                    inDX += e.deltaX;
                    inDY += e.deltaY;
                }
            } else if (e.type == NSEventTypeScrollWheel) {
                inScroll += e.scrollingDeltaY;
            }
            [NSApp sendEvent:e];
        }
    }

    bool visible() { return window && [window isVisible]; }

    void display(const uint8_t* rgba, int w, int h, double simTime) {
        @autoreleasepool {
            overlay.Tick(simTime, label);
            // Keep the Metal layer sized to the (possibly resized) window each frame. The layer's frame and
            // drawableSize must both track the content view or the image only fills a sub-region of the window.
            if (window) {
                NSView* cv = window.contentView;
                CGFloat s = window.backingScaleFactor;
                if (s < 1)
                    s = 1;
                layer.frame = cv.bounds;
                CGSize want = CGSizeMake(cv.bounds.size.width * s, cv.bounds.size.height * s);
                if (want.width >= 1 && want.height >= 1 && (layer.drawableSize.width != want.width || layer.drawableSize.height != want.height))
                    layer.drawableSize = want;
            }
            if (!tex || texW != w || texH != h) {
                MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:w height:h mipmapped:NO];
                td.usage = MTLTextureUsageShaderRead;
                td.storageMode = MTLStorageModeShared;
                tex = [dev newTextureWithDescriptor:td];
                texW = w;
                texH = h;
            }
            [tex replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0 withBytes:rgba bytesPerRow:w * 4];

            id<CAMetalDrawable> dr = [layer nextDrawable];
            if (!dr) {
                pump();
                return;
            }
            MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
            rp.colorAttachments[0].texture = dr.texture;
            rp.colorAttachments[0].loadAction = MTLLoadActionClear;
            rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
            rp.colorAttachments[0].storeAction = MTLStoreActionStore;
            id<MTLCommandBuffer> cb = [queue commandBuffer];
            id<MTLRenderCommandEncoder> re = [cb renderCommandEncoderWithDescriptor:rp];
            [re setRenderPipelineState:pso];
            [re setFragmentTexture:tex atIndex:0];
            [re drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
            overlay.Draw((__bridge void*)dev, (__bridge void*)re, layer.drawableSize.width, layer.drawableSize.height);
            [re endEncoding];
            [cb presentDrawable:dr];
            [cb commit];
            pump();
        }
    }
};

ChFilterMetalVisualize::ChFilterMetalVisualize(int w, int h, std::string name) : ChFilter(name), m_w(w), m_h(h), m_win_name(std::move(name)) {
    p = new Impl();
}
ChFilterMetalVisualize::~ChFilterMetalVisualize() {
    delete p;
}

void ChFilterMetalVisualize::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_sensor = pSensor.get();  // non-owning; used to drive the offset pose in orbit mode
    // Accept whichever host buffer the sensor produced; colorize to RGBA8 in Apply.
    m_rgba8 = std::dynamic_pointer_cast<SensorHostRGBA8Buffer>(bufferInOut);
    m_semantic = std::dynamic_pointer_cast<SensorHostSemanticBuffer>(bufferInOut);
    m_di = std::dynamic_pointer_cast<SensorHostDIBuffer>(bufferInOut);
    // pass the buffer through unchanged
}

void ChFilterMetalVisualize::Apply() {
    unsigned int w = 0, h = 0;
    const uint8_t* rgba = nullptr;
    double simTime = 0.0;

    if (m_rgba8 && m_rgba8->Buffer) {
        w = m_rgba8->Width;
        h = m_rgba8->Height;
        simTime = m_rgba8->TimeStamp;
        rgba = reinterpret_cast<const uint8_t*>(m_rgba8->Buffer.get());
    } else if (m_semantic && m_semantic->Buffer) {
        w = m_semantic->Width;
        h = m_semantic->Height;
        simTime = m_semantic->TimeStamp;
        m_tmp.assign((size_t)w * h * 4, 0);
        for (size_t i = 0; i < (size_t)w * h; ++i) {
            unsigned short id = m_semantic->Buffer[i].class_id;
            unsigned char r = 0, g = 0, b = 0;
            if (id) {
                unsigned int q = id * 2654435761u;
                r = 64 + (q & 127);
                g = 64 + ((q >> 8) & 127);
                b = 64 + ((q >> 16) & 127);
            }
            m_tmp[i * 4] = r;
            m_tmp[i * 4 + 1] = g;
            m_tmp[i * 4 + 2] = b;
            m_tmp[i * 4 + 3] = 255;
        }
        rgba = m_tmp.data();
    } else if (m_di && m_di->Buffer) {
        w = m_di->Width;
        h = m_di->Height;
        simTime = m_di->TimeStamp;
        m_tmp.assign((size_t)w * h * 4, 0);
        float mx = 0;
        for (size_t i = 0; i < (size_t)w * h; ++i) {
            float rg = m_di->Buffer[i].range;
            if (rg > 0 && rg < 1e6f)
                mx = std::max(mx, rg);
        }
        if (mx <= 0)
            mx = 1;
        for (size_t i = 0; i < (size_t)w * h; ++i) {
            float rg = m_di->Buffer[i].range;
            unsigned char v = (rg > 0 && rg < 1e6f) ? (unsigned char)(255.f * std::max(0.f, 1.f - rg / mx)) : 0;
            m_tmp[i * 4] = v;
            m_tmp[i * 4 + 1] = v;
            m_tmp[i * 4 + 2] = v;
            m_tmp[i * 4 + 3] = 255;
        }
        rgba = m_tmp.data();
    }

    if (!rgba || w == 0 || h == 0)
        return;
    p->ensureWindow(m_w, m_h, m_win_name);
    p->display(rgba, (int)w, (int)h, simTime);

    // interactive orbit: consume the pointer input gathered while pumping events and
    // drive the sensor's offset pose (takes effect on the next rendered frame).
    if (m_orbit && m_sensor) {
        double dx, dy, sc;
        p->consumeInput(dx, dy, sc);
        p->orbit.ApplyInput(-dx, dy, sc);  // "grab the scene" feel: drag follows the view
        m_sensor->SetOffsetPose(p->orbit.OffsetPose());
    }
}

void ChFilterMetalVisualize::EnableOrbitControl(float dist, float tx, float ty, float tz) {
    m_orbit = true;
    p->orbit.dist = dist;
    p->orbit.target = ChVector3d(tx, ty, tz);
}

bool ChFilterMetalVisualize::WindowOpen() const {
    return !p->window || p->visible();
}

}  // namespace sensor
}  // namespace chrono
