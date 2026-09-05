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

#include <cmath>
#include <algorithm>

#include "chrono_sensor/metal/ChFilterMetalVisualizePointCloud.h"
#include "chrono_sensor/metal/ChMetalPerfOverlay.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

namespace chrono {
namespace sensor {

static const char* kPointMSL = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct VOut { float4 pos [[position]]; float psize [[point_size]]; float3 col; };
vertex VOut vmain(uint vid [[vertex_id]], device const float* pts [[buffer(0)]],
                  constant float4x4& mvp [[buffer(1)]], constant float& psize [[buffer(2)]]) {
    float3 p = float3(pts[vid*6], pts[vid*6+1], pts[vid*6+2]);
    VOut o; o.pos = mvp * float4(p, 1.0); o.psize = psize;
    o.col = float3(pts[vid*6+3], pts[vid*6+4], pts[vid*6+5]); return o;
}
fragment float4 fmain(VOut in [[stage_in]]) { return float4(in.col, 1.0); }
)MSL";

// column-major 4x4 helpers
static void mat_mul(const float* a, const float* b, float* o) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            float s = 0;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + r] * b[c * 4 + k];
            o[c * 4 + r] = s;
        }
}
static void mat_persp(float fovy, float aspect, float znear, float zfar, float* m) {
    for (int i = 0; i < 16; i++)
        m[i] = 0;
    float f = 1.f / std::tan(fovy * 0.5f);
    m[0] = f / aspect;
    m[5] = f;
    m[10] = zfar / (znear - zfar);
    m[11] = -1.f;
    m[14] = (znear * zfar) / (znear - zfar);
}
static void mat_lookat(const float* eye, const float* ctr, const float* up, float* m) {
    float z[3] = {eye[0] - ctr[0], eye[1] - ctr[1], eye[2] - ctr[2]};
    float zl = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    for (int i = 0; i < 3; i++)
        z[i] /= zl;
    float x[3] = {up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]};
    float xl = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int i = 0; i < 3; i++)
        x[i] /= xl;
    float y[3] = {z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]};
    m[0] = x[0];
    m[1] = y[0];
    m[2] = z[0];
    m[3] = 0;
    m[4] = x[1];
    m[5] = y[1];
    m[6] = z[1];
    m[7] = 0;
    m[8] = x[2];
    m[9] = y[2];
    m[10] = z[2];
    m[11] = 0;
    m[12] = -(x[0] * eye[0] + x[1] * eye[1] + x[2] * eye[2]);
    m[13] = -(y[0] * eye[0] + y[1] * eye[1] + y[2] * eye[2]);
    m[14] = -(z[0] * eye[0] + z[1] * eye[1] + z[2] * eye[2]);
    m[15] = 1;
}

struct ChFilterMetalVisualizePointCloud::Impl {
    NSWindow* window = nil;
    CAMetalLayer* layer = nil;
    id<MTLTexture> depthTex = nil;
    id<MTLDevice> dev = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLRenderPipelineState> pso = nil;
    id<MTLDepthStencilState> dss = nil;
    MetalPerfOverlay overlay;
    std::string label;
    id<MTLBuffer> vbuf = nil;
    size_t vcap = 0;
    float yaw = 0.6f;

    void ensureWindow(int w, int h, const std::string& title) {
        if (window)
            return;
        dev = MTLCreateSystemDefaultDevice();
        queue = [dev newCommandQueue];
        NSError* e = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:@(kPointMSL) options:nil error:&e];
        MTLRenderPipelineDescriptor* rpd = [MTLRenderPipelineDescriptor new];
        rpd.vertexFunction = [lib newFunctionWithName:@"vmain"];
        rpd.fragmentFunction = [lib newFunctionWithName:@"fmain"];
        rpd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        rpd.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        pso = [dev newRenderPipelineStateWithDescriptor:rpd error:&e];
        MTLDepthStencilDescriptor* dd = [MTLDepthStencilDescriptor new];
        dd.depthCompareFunction = MTLCompareFunctionLess;
        dd.depthWriteEnabled = YES;
        dss = [dev newDepthStencilStateWithDescriptor:dd];
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        NSRect fr = NSMakeRect(0, 0, w, h);
        window = [[NSWindow alloc] initWithContentRect:fr
                                             styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable)
                                               backing:NSBackingStoreBuffered
                                                 defer:NO];
        [window setTitle:[NSString stringWithUTF8String:title.c_str()]];
        [window center];
        label = title;
        NSView* v = [[NSView alloc] initWithFrame:fr];
        v.wantsLayer = YES;
        CGFloat s = [window backingScaleFactor];
        if (s < 1)
            s = 1;
        layer = [CAMetalLayer layer];
        layer.device = dev;
        layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
        layer.framebufferOnly = NO;
        layer.contentsScale = s;
        layer.drawableSize = CGSizeMake(w * s, h * s);
        v.layer = layer;
        window.contentView = v;
        [window makeKeyAndOrderFront:nil];
        [NSApp activateIgnoringOtherApps:YES];
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                      width:(NSUInteger)(w * s)
                                                                                     height:(NSUInteger)(h * s)
                                                                                  mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget;
        td.storageMode = MTLStorageModePrivate;
        depthTex = [dev newTextureWithDescriptor:td];
    }
    void pump() {
        NSEvent* e;
        while ((e = [NSApp nextEventMatchingMask:NSEventMaskAny untilDate:[NSDate distantPast] inMode:NSDefaultRunLoopMode dequeue:YES]))
            [NSApp sendEvent:e];
    }
    bool visible() { return window && [window isVisible]; }

    void draw(const std::vector<float>& pts, float ptsize, double simTime) {
        @autoreleasepool {
            overlay.Tick(simTime, label);
            size_t n = pts.size() / 6;
            if (n == 0) {
                pump();
                return;
            }
            size_t bytes = pts.size() * sizeof(float);
            if (!vbuf || vcap < bytes) {
                vbuf = [dev newBufferWithLength:bytes options:MTLResourceStorageModeShared];
                vcap = bytes;
            }
            memcpy(vbuf.contents, pts.data(), bytes);

            id<CAMetalDrawable> dr = [layer nextDrawable];
            if (!dr) {
                pump();
                return;
            }
            if (!depthTex || depthTex.width != dr.texture.width || depthTex.height != dr.texture.height) {
                MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                              width:dr.texture.width
                                                                                             height:dr.texture.height
                                                                                          mipmapped:NO];
                td.usage = MTLTextureUsageRenderTarget;
                td.storageMode = MTLStorageModePrivate;
                depthTex = [dev newTextureWithDescriptor:td];
            }
            // orbiting camera
            yaw += 0.01f;
            float R = 20.f, hgt = 11.f;
            float eye[3] = {R * std::cos(yaw), R * std::sin(yaw), hgt}, ctr[3] = {0, 0, 0.5f}, up[3] = {0, 0, 1};
            float proj[16], view[16], mvp[16];
            float aspect = (float)dr.texture.width / (float)dr.texture.height;
            mat_persp(45.f * 3.14159265f / 180.f, aspect, 0.2f, 300.f, proj);
            mat_lookat(eye, ctr, up, view);
            mat_mul(proj, view, mvp);

            MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
            rp.colorAttachments[0].texture = dr.texture;
            rp.colorAttachments[0].loadAction = MTLLoadActionClear;
            rp.colorAttachments[0].clearColor = MTLClearColorMake(0.05, 0.05, 0.07, 1);
            rp.colorAttachments[0].storeAction = MTLStoreActionStore;
            rp.depthAttachment.texture = depthTex;
            rp.depthAttachment.loadAction = MTLLoadActionClear;
            rp.depthAttachment.clearDepth = 1.0;
            rp.depthAttachment.storeAction = MTLStoreActionDontCare;
            id<MTLCommandBuffer> cb = [queue commandBuffer];
            id<MTLRenderCommandEncoder> re = [cb renderCommandEncoderWithDescriptor:rp];
            [re setRenderPipelineState:pso];
            [re setDepthStencilState:dss];
            [re setVertexBuffer:vbuf offset:0 atIndex:0];
            [re setVertexBytes:mvp length:64 atIndex:1];
            [re setVertexBytes:&ptsize length:4 atIndex:2];
            [re drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:(NSUInteger)n];
            overlay.Draw((__bridge void*)dev, (__bridge void*)re, dr.texture.width, dr.texture.height);
            [re endEncoding];
            [cb presentDrawable:dr];
            [cb commit];
            pump();
        }
    }
};

ChFilterMetalVisualizePointCloud::ChFilterMetalVisualizePointCloud(int w, int h, float point_size, std::string name)
    : ChFilter(name), m_w(w), m_h(h), m_ptsize(point_size), m_name(std::move(name)) {
    p = new Impl();
}
ChFilterMetalVisualizePointCloud::~ChFilterMetalVisualizePointCloud() {
    delete p;
}

void ChFilterMetalVisualizePointCloud::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_di = std::dynamic_pointer_cast<SensorHostDIBuffer>(bufferInOut);
    m_radar = std::dynamic_pointer_cast<SensorHostRadarBuffer>(bufferInOut);
    if (auto ld = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
        m_hfov = ld->GetHFOV();
        m_vmin = ld->GetMinVertAngle();
        m_vmax = ld->GetMaxVertAngle();
        m_beam_w = ld->GetWidth();
        m_beam_h = ld->GetHeight();
    }
}

void ChFilterMetalVisualizePointCloud::Apply() {
    auto jet = [](float t, float& r, float& g, float& b) {
        t = std::clamp(t, 0.f, 1.f);
        r = std::clamp(1.5f - std::fabs(4.f * t - 3.f), 0.f, 1.f);
        g = std::clamp(1.5f - std::fabs(4.f * t - 2.f), 0.f, 1.f);
        b = std::clamp(1.5f - std::fabs(4.f * t - 1.f), 0.f, 1.f);
    };
    m_pts.clear();
    double simTime = 0.0;

    if (m_radar && m_radar->Buffer) {  // radar: az/el/range stored per return; color by |Doppler|
        simTime = m_radar->TimeStamp;
        const size_t n = (size_t)m_radar->Width * m_radar->Height;
        m_pts.reserve(n * 6);
        for (size_t k = 0; k < n; ++k) {
            const RadarReturn& rr = m_radar->Buffer[k];
            if (rr.range <= 0 || rr.range > 1e5f)
                continue;
            float x = std::cos(rr.elevation) * std::cos(rr.azimuth) * rr.range;
            float y = std::cos(rr.elevation) * std::sin(rr.azimuth) * rr.range;
            float z = std::sin(rr.elevation) * rr.range;
            float dop =
                std::sqrt(rr.doppler_velocity[0] * rr.doppler_velocity[0] + rr.doppler_velocity[1] * rr.doppler_velocity[1] + rr.doppler_velocity[2] * rr.doppler_velocity[2]);
            float r, g, b;
            jet(dop / 8.f, r, g, b);  // moving returns pop in warm colors
            if (dop < 0.05f) {
                r = g = b = 0.55f;
            }  // static returns: gray
            m_pts.insert(m_pts.end(), {x, y, z, r, g, b});
        }
    } else if (m_di && m_di->Buffer && m_beam_w != 0) {  // lidar: reconstruct az/el, color by height
        simTime = m_di->TimeStamp;
        const unsigned int W = m_beam_w, H = m_beam_h;
        m_pts.reserve((size_t)W * H * 6);
        for (unsigned int j = 0; j < H; ++j) {
            float el = m_vmin + ((H > 1) ? (float)j / (float)(H - 1) : 0.5f) * (m_vmax - m_vmin);
            for (unsigned int i = 0; i < W; ++i) {
                float rg = m_di->Buffer[j * W + i].range;
                if (rg <= 0 || rg > 1e5f)
                    continue;
                float az = -m_hfov * 0.5f + ((float)i + 0.5f) / (float)W * m_hfov;
                float x = std::cos(el) * std::cos(az) * rg;
                float y = std::cos(el) * std::sin(az) * rg;
                float z = std::sin(el) * rg;
                float r, g, b;
                jet((z + 1.5f) / 4.0f, r, g, b);
                m_pts.insert(m_pts.end(), {x, y, z, r, g, b});
            }
        }
    } else {
        return;
    }
    p->ensureWindow(m_w, m_h, m_name);
    p->draw(m_pts, m_ptsize, simTime);
}

bool ChFilterMetalVisualizePointCloud::WindowOpen() const {
    return !p->window || p->visible();
}

}  // namespace sensor
}  // namespace chrono
