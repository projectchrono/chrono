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

#include <cstdio>
#include <chrono>

#include "chrono_sensor/metal/ChMetalPerfOverlay.h"

namespace chrono {
namespace sensor {

static const char* kOverlayMSL = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct VOut { float4 pos [[position]]; float2 uv; };
vertex VOut ov_v(uint vid [[vertex_id]], constant float4* quad [[buffer(0)]]) {
    VOut o; o.pos = float4(quad[vid].xy, 0, 1); o.uv = quad[vid].zw; return o;
}
fragment float4 ov_f(VOut in [[stage_in]], texture2d<float> t [[texture(0)]]) {
    constexpr sampler s(filter::linear);
    return t.sample(s, in.uv);
}
)MSL";

struct MetalPerfOverlay::Impl {
    // timing
    std::chrono::steady_clock::time_point lastWall{};
    double lastSim = 0.0;
    bool haveLast = false;
    int frames = 0;
    double accWall = 0.0;  // wall seconds accumulated in the current window
    double accSim = 0.0;   // sim seconds accumulated in the current window
    double fps = 0.0, rtf = 0.0;
    std::string label;
    std::string text;     // current HUD text
    std::string texText;  // text currently baked into the texture

    // GPU
    id<MTLRenderPipelineState> pso = nil;
    id<MTLTexture> tex = nil;
    int texW = 400, texH = 52;

    void ensurePipeline(id<MTLDevice> dev) {
        if (pso)
            return;
        NSError* e = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:@(kOverlayMSL) options:nil error:&e];
        MTLRenderPipelineDescriptor* rpd = [MTLRenderPipelineDescriptor new];
        rpd.vertexFunction = [lib newFunctionWithName:@"ov_v"];
        rpd.fragmentFunction = [lib newFunctionWithName:@"ov_f"];
        rpd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        rpd.colorAttachments[0].blendingEnabled = YES;
        rpd.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        rpd.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        rpd.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
        rpd.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        pso = [dev newRenderPipelineStateWithDescriptor:rpd error:&e];
    }

    void bakeText(id<MTLDevice> dev) {
        if (tex && texText == text)
            return;
        texText = text;
        NSDictionary* attrs = @{
            NSFontAttributeName : [NSFont monospacedSystemFontOfSize:19 weight:NSFontWeightSemibold],
            NSForegroundColorAttributeName : [NSColor colorWithRed:0.75 green:1.0 blue:0.8 alpha:1.0]
        };
        NSString* s = [NSString stringWithUTF8String:text.c_str()];
        NSSize sz = [s sizeWithAttributes:attrs];
        // size the HUD to fit the text so nothing (e.g. the RTF value) is ever clipped
        int W = (int)(sz.width + 0.5) + 26;
        if (W < 120)
            W = 120;
        if (W > 1200)
            W = 1200;
        int H = (int)(sz.height + 0.5) + 14;
        if (H < 34)
            H = 34;
        texW = W;
        texH = H;
        NSBitmapImageRep* rep = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:NULL
                                                                        pixelsWide:W
                                                                        pixelsHigh:H
                                                                     bitsPerSample:8
                                                                   samplesPerPixel:4
                                                                          hasAlpha:YES
                                                                          isPlanar:NO
                                                                    colorSpaceName:NSCalibratedRGBColorSpace
                                                                       bytesPerRow:W * 4
                                                                      bitsPerPixel:32];
        NSGraphicsContext* ctx = [NSGraphicsContext graphicsContextWithBitmapImageRep:rep];
        [NSGraphicsContext saveGraphicsState];
        [NSGraphicsContext setCurrentContext:ctx];
        // rounded translucent backdrop
        [[NSColor colorWithWhite:0.0 alpha:0.0] set];
        NSRectFill(NSMakeRect(0, 0, W, H));
        NSBezierPath* bg = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(2, 2, W - 4, H - 4) xRadius:8 yRadius:8];
        [[NSColor colorWithWhite:0.0 alpha:0.55] set];
        [bg fill];
        [s drawAtPoint:NSMakePoint(13, (H - sz.height) * 0.5) withAttributes:attrs];
        [ctx flushGraphics];
        [NSGraphicsContext restoreGraphicsState];

        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:W height:H mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        tex = [dev newTextureWithDescriptor:td];
        [tex replaceRegion:MTLRegionMake2D(0, 0, W, H) mipmapLevel:0 withBytes:[rep bitmapData] bytesPerRow:W * 4];
    }
};

MetalPerfOverlay::MetalPerfOverlay() {
    p = new Impl();
}
MetalPerfOverlay::~MetalPerfOverlay() {
    delete p;
}

void MetalPerfOverlay::Tick(double simTime, const std::string& label) {
    p->label = label;
    auto now = std::chrono::steady_clock::now();
    if (!p->haveLast) {
        p->lastWall = now;
        p->lastSim = simTime;
        p->haveLast = true;
        p->text = label + "  |  -- fps  |  RTF --";
        return;
    }
    double dw = std::chrono::duration<double>(now - p->lastWall).count();
    double ds = simTime - p->lastSim;
    p->lastWall = now;
    p->lastSim = simTime;
    if (dw <= 0)
        return;
    p->frames += 1;
    p->accWall += dw;
    p->accSim += ds;
    if (p->accWall >= 0.4) {  // refresh estimate a few times a second
        p->fps = p->frames / p->accWall;
        p->rtf = p->accSim / p->accWall;
        char buf[160];
        snprintf(buf, sizeof(buf), "%s  |  %.1f fps  |  RTF %.2f", label.c_str(), p->fps, p->rtf);
        p->text = buf;
        p->frames = 0;
        p->accWall = 0.0;
        p->accSim = 0.0;
    }
}

void MetalPerfOverlay::Draw(void* device, void* encoder, double drawableW, double drawableH) {
    id<MTLDevice> dev = (__bridge id<MTLDevice>)device;
    id<MTLRenderCommandEncoder> enc = (__bridge id<MTLRenderCommandEncoder>)encoder;
    if (!dev || !enc || drawableW <= 0 || drawableH <= 0)
        return;
    p->ensurePipeline(dev);
    p->bakeText(dev);
    if (!p->pso || !p->tex)
        return;

    const double m = 12.0;  // margin in pixels
    double Wp = p->texW, Hp = p->texH;
    double left = -1.0 + 2.0 * m / drawableW;
    double right = -1.0 + 2.0 * (m + Wp) / drawableW;
    double top = 1.0 - 2.0 * m / drawableH;
    double bot = 1.0 - 2.0 * (m + Hp) / drawableH;
    // triangle strip: (l,t) (l,b) (r,t) (r,b); uv y flipped (bitmap row 0 = top)
    float quad[16] = {
        (float)left, (float)top, 0.f, 0.f, (float)left, (float)bot, 0.f, 1.f, (float)right, (float)top, 1.f, 0.f, (float)right, (float)bot, 1.f, 1.f,
    };
    [enc setRenderPipelineState:p->pso];
    [enc setVertexBytes:quad length:sizeof(quad) atIndex:0];
    [enc setFragmentTexture:p->tex atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
}

double MetalPerfOverlay::FPS() const {
    return p->fps;
}
double MetalPerfOverlay::RTF() const {
    return p->rtf;
}

}  // namespace sensor
}  // namespace chrono
