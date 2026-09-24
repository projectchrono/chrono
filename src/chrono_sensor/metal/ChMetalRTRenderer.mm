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
// Ported from the standalone Metal ray-tracing prototype. Consumes a
// MetalRenderScene, builds one BLAS per geometry + a TLAS of instances, and runs
// the single-source (Slang-generated) inline-ray-query MSL kernel.
// =============================================================================

#import <Metal/Metal.h>
#import <ImageIO/ImageIO.h>
#import <CoreGraphics/CoreGraphics.h>

#include <vector>
#include <cmath>
#include <cstring>
#include <string>

#include "chrono_thirdparty/stb/stb_image.h"  // stbi_loadf (impl already in the sensor lib)
#include "chrono_sensor/metal/ChMetalRTRenderer.h"
#include "chrono_sensor/metal/shaders/ChMetalRTShaderMSL.h"  // kRaytraceMSL

namespace chrono {
namespace sensor {

struct GeomGPU {
    int blasIndex;
    int triBase;
    int triCount;
    bool dynamic;
    id<MTLBuffer> vbuf;
    id<MTLBuffer> scratch;
};

struct ChMetalRTRenderer::Impl {
    id<MTLDevice> dev;
    id<MTLCommandQueue> queue;
    id<MTLComputePipelineState> pso, dpso;
    id<MTLBuffer> uni, instBuf, instRot, tintBuf, matBuf, nBaseBuf, gNormal, gAlbedo, gUVBuf, gTexId, gOpacity, gRoughness, gMetallicBuf, gRoughTexIdBuf, gMetalTexIdBuf,
        gOpacityTexIdBuf, gTangentBuf, gNormalTexIdBuf, gSpecularBuf, gEmissiveBuf, gTexScaleBuf, gKsTexIdBuf, gKeTexIdBuf, gBlendKdTexIdBuf, gBlendWeightTexIdBuf, tlasScratch,
        lightsBuf, instIdsBuf;
    id<MTLTexture> offTex, offTex2, whiteTex, envTex;
    bool envValid = false;

    void setEnvMap(const std::string& path) {
        envValid = false;
        envTex = nil;
        if (path.empty())
            return;
        int w = 0, h = 0, n = 0;
        float* data = stbi_loadf(path.c_str(), &w, &h, &n, 4);
        if (!data) {
            return;
        }
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA32Float width:w height:h mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        envTex = [dev newTextureWithDescriptor:td];
        [envTex replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0 withBytes:data bytesPerRow:(NSUInteger)w * 16];
        stbi_image_free(data);
        envValid = true;
    }
    id<MTLSamplerState> samp;
    id<MTLAccelerationStructure> tlas;
    MTLInstanceAccelerationStructureDescriptor* idesc;
    std::vector<GeomGPU> geoms;
    std::vector<id<MTLAccelerationStructure>> blas;
    std::vector<id<MTLBuffer>> retain;
    std::vector<id<MTLTexture>> texList;

    // Scene topology captured at the last build(). Every per-instance buffer, the BLAS list and the
    // TLAS descriptor are sized from these, so if the counts change (bodies added or removed at
    // runtime, e.g. ChSensorManager::ReconstructScenes) updateDynamic() would index past the end of
    // buffers built for the old scene. Detected in UpdateDynamic(), which rebuilds instead.
    size_t builtGeomCount = 0, builtInstCount = 0, builtTexCount = 0;
    bool topologyChanged(const MetalRenderScene& s) const {
        return s.geometries.size() != builtGeomCount || s.instances.size() != builtInstCount || s.texturePaths.size() != builtTexCount;
    }

    static void c3(float* o, float x, float y, float z) {
        o[0] = x;
        o[1] = y;
        o[2] = z;
    }

    id<MTLBuffer> mkbuf(std::vector<float>& v) {
        if (v.empty())
            return [dev newBufferWithLength:16 options:MTLResourceStorageModeShared];  // v.data() may be null when empty
        return [dev newBufferWithBytes:v.data() length:v.size() * 4 options:MTLResourceStorageModeShared];
    }
    id<MTLBuffer> mkbufI(std::vector<int>& v) {
        if (v.empty())
            return [dev newBufferWithLength:16 options:MTLResourceStorageModeShared];
        return [dev newBufferWithBytes:v.data() length:v.size() * 4 options:MTLResourceStorageModeShared];
    }
    id<MTLTexture> offscreen(int w, int h) {
        if (offTex && (int)offTex.width == w && (int)offTex.height == h)
            return offTex;
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA32Float width:w height:h mipmapped:NO];
        td.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        offTex = [dev newTextureWithDescriptor:td];
        return offTex;
    }
    id<MTLTexture> loadTex(const std::string& path) {
        CGImageSourceRef src = CGImageSourceCreateWithURL((__bridge CFURLRef)[NSURL fileURLWithPath:[NSString stringWithUTF8String:path.c_str()]], NULL);
        if (!src)
            return whiteTex;
        CGImageRef img = CGImageSourceCreateImageAtIndex(src, 0, NULL);
        CFRelease(src);
        if (!img)
            return whiteTex;
        size_t w = CGImageGetWidth(img), h = CGImageGetHeight(img);
        std::vector<uint8_t> d(w * h * 4, 0);  // init transparent, so PNG alpha (foliage cutout) survives
        CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
        CGContextRef c = CGBitmapContextCreate(d.data(), w, h, 8, w * 4, cs, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
        CGContextDrawImage(c, CGRectMake(0, 0, w, h), img);
        CGContextRelease(c);
        CGColorSpaceRelease(cs);
        CGImageRelease(img);
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:w height:h mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        id<MTLTexture> t = [dev newTextureWithDescriptor:td];
        [t replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0 withBytes:d.data() bytesPerRow:w * 4];
        return t;
    }
    id<MTLAccelerationStructure> buildBLAS(id<MTLBuffer> vb, int tc, bool refit, id<MTLBuffer>* scr) {
        MTLAccelerationStructureTriangleGeometryDescriptor* g = [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
        g.vertexBuffer = vb;
        g.vertexStride = 12;
        g.vertexFormat = MTLAttributeFormatFloat3;
        g.triangleCount = tc;
        MTLPrimitiveAccelerationStructureDescriptor* pd = [MTLPrimitiveAccelerationStructureDescriptor descriptor];
        pd.geometryDescriptors = @[ g ];
        // Dynamic geometry: refittable (rebuilt cheaply each frame). Static geometry (terrain + trees, the bulk):
        // build a higher-quality BVH once -> faster ray traversal every frame (helps supersampling most).
        pd.usage = refit ? MTLAccelerationStructureUsageRefit : MTLAccelerationStructureUsagePreferFastIntersection;
        MTLAccelerationStructureSizes sz = [dev accelerationStructureSizesWithDescriptor:pd];
        id<MTLAccelerationStructure> as = [dev newAccelerationStructureWithSize:sz.accelerationStructureSize];
        id<MTLBuffer> bs = [dev newBufferWithLength:MAX(sz.buildScratchBufferSize, 16) options:MTLResourceStorageModePrivate];
        if (refit && scr)
            *scr = [dev newBufferWithLength:MAX(sz.refitScratchBufferSize, 16) options:MTLResourceStorageModePrivate];
        id<MTLCommandBuffer> cb = [queue commandBuffer];
        id<MTLAccelerationStructureCommandEncoder> e = [cb accelerationStructureCommandEncoder];
        [e buildAccelerationStructure:as descriptor:pd scratchBuffer:bs scratchBufferOffset:0];
        [e endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        return as;
    }

    void build(const MetalRenderScene& scene) {
        @autoreleasepool {
            geoms.clear();
            blas.clear();
            retain.clear();
            texList.clear();
            std::vector<float> gn, ga, guv, go, grh, gmet, gtan, gspec, gemis, gts;
            std::vector<int> gt, grtx, gmtx, gopx, gntx, gktx, getx, gbktx, gbwtx;
            // append a per-geometry int array, padding with -1 to the triangle count so a geometry that never set
            // it can't shorten the concatenated buffer (which the shader indexes by triangle -> would read OOB)
            auto appendI = [](std::vector<int>& dst, const std::vector<int>& src, int n) {
                if ((int)src.size() == n)
                    dst.insert(dst.end(), src.begin(), src.end());
                else
                    dst.insert(dst.end(), (size_t)n, -1);
            };
            for (auto& g : scene.geometries) {
                GeomGPU gg;
                gg.triBase = (int)(gn.size() / 9);
                gg.triCount = g.triCount();
                gg.dynamic = g.dynamic;
                gg.blasIndex = (int)blas.size();
                appendI(gbktx, g.blendKdTexId, gg.triCount);
                appendI(gbwtx, g.blendWeightTexId, gg.triCount);
                gn.insert(gn.end(), g.normals.begin(), g.normals.end());
                ga.insert(ga.end(), g.colors.begin(), g.colors.end());
                gtan.insert(gtan.end(), g.tangents.begin(), g.tangents.end());
                gntx.insert(gntx.end(), g.normalTexId.begin(), g.normalTexId.end());
                gspec.insert(gspec.end(), g.specular.begin(), g.specular.end());
                gemis.insert(gemis.end(), g.emissive.begin(), g.emissive.end());
                gts.insert(gts.end(), g.texScale.begin(), g.texScale.end());
                gktx.insert(gktx.end(), g.ksTexId.begin(), g.ksTexId.end());
                getx.insert(getx.end(), g.keTexId.begin(), g.keTexId.end());
                guv.insert(guv.end(), g.uv.begin(), g.uv.end());
                gt.insert(gt.end(), g.texId.begin(), g.texId.end());
                grtx.insert(grtx.end(), g.roughTexId.begin(), g.roughTexId.end());
                gmtx.insert(gmtx.end(), g.metalTexId.begin(), g.metalTexId.end());
                gopx.insert(gopx.end(), g.opacityTexId.begin(), g.opacityTexId.end());
                go.insert(go.end(), g.opacity.begin(), g.opacity.end());
                grh.insert(grh.end(), g.roughness.begin(), g.roughness.end());
                gmet.insert(gmet.end(), g.metallic.begin(), g.metallic.end());
                id<MTLBuffer> vb = [dev newBufferWithBytes:g.verts.data() length:MAX(g.verts.size() * 4, (size_t)16) options:MTLResourceStorageModeShared];
                id<MTLBuffer> scr = nil;
                id<MTLAccelerationStructure> as = buildBLAS(vb, gg.triCount, g.dynamic, &scr);
                gg.vbuf = vb;
                gg.scratch = scr;
                blas.push_back(as);
                retain.push_back(vb);
                geoms.push_back(gg);
            }
            gNormal = mkbuf(gn);
            gAlbedo = mkbuf(ga);
            gUVBuf = mkbuf(guv);
            gTexId = mkbufI(gt);
            gOpacity = mkbuf(go);
            gRoughness = mkbuf(grh);
            gMetallicBuf = mkbuf(gmet);
            gRoughTexIdBuf = mkbufI(grtx);
            gMetalTexIdBuf = mkbufI(gmtx);
            gOpacityTexIdBuf = mkbufI(gopx);
            gTangentBuf = mkbuf(gtan);
            gNormalTexIdBuf = mkbufI(gntx);
            gSpecularBuf = mkbuf(gspec);
            gEmissiveBuf = mkbuf(gemis);
            gTexScaleBuf = mkbuf(gts);
            gKsTexIdBuf = mkbufI(gktx);
            gKeTexIdBuf = mkbufI(getx);
            gBlendKdTexIdBuf = mkbufI(gbktx);
            gBlendWeightTexIdBuf = mkbufI(gbwtx);
            for (auto& pth : scene.texturePaths)
                texList.push_back(loadTex(pth));
            int N = (int)scene.instances.size();
            if (N < 1)
                N = 1;
            instBuf = [dev newBufferWithLength:N * 64 options:MTLResourceStorageModeShared];
            instRot = [dev newBufferWithLength:N * 9 * 4 options:MTLResourceStorageModeShared];
            tintBuf = [dev newBufferWithLength:N * 12 options:MTLResourceStorageModeShared];
            matBuf = [dev newBufferWithLength:N * 4 options:MTLResourceStorageModeShared];
            nBaseBuf = [dev newBufferWithLength:N * 4 options:MTLResourceStorageModeShared];
            instIdsBuf = [dev newBufferWithLength:N * 2 * 4 options:MTLResourceStorageModeShared];
            float* tb = (float*)tintBuf.contents;
            uint32_t* mb = (uint32_t*)matBuf.contents;
            uint32_t* nb = (uint32_t*)nBaseBuf.contents;
            uint32_t* ib = (uint32_t*)instIdsBuf.contents;
            for (size_t i = 0; i < scene.instances.size(); ++i) {
                auto& in = scene.instances[i];
                tb[i * 3] = in.tint[0];
                tb[i * 3 + 1] = in.tint[1];
                tb[i * 3 + 2] = in.tint[2];
                mb[i] = in.mat;
                nb[i] = geoms[in.geom].triBase;
                ib[i * 2] = in.classId;
                ib[i * 2 + 1] = in.instanceId;
            }
            NSMutableArray* arr = [NSMutableArray array];
            for (auto a : blas)
                [arr addObject:a];
            idesc = [MTLInstanceAccelerationStructureDescriptor descriptor];
            idesc.instancedAccelerationStructures = arr;
            idesc.instanceDescriptorBuffer = instBuf;
            idesc.instanceDescriptorStride = 64;
            idesc.instanceCount = (int)scene.instances.size();
            MTLAccelerationStructureSizes ts = [dev accelerationStructureSizesWithDescriptor:idesc];
            tlas = [dev newAccelerationStructureWithSize:ts.accelerationStructureSize];
            tlasScratch = [dev newBufferWithLength:MAX(ts.buildScratchBufferSize, 16) options:MTLResourceStorageModePrivate];
            builtGeomCount = scene.geometries.size();
            builtInstCount = scene.instances.size();
            builtTexCount = scene.texturePaths.size();
            updateDynamic(scene);
        }
    }

    void updateDynamic(const MetalRenderScene& scene) {
        @autoreleasepool {
            // Defensive: never index the build-time arrays with a scene they were not built for.
            if (topologyChanged(scene))
                return;
            for (size_t i = 0; i < scene.instances.size(); ++i) {
                auto& in = scene.instances[i];
                if (in.geom >= (int)geoms.size())
                    continue;
                float* f = (float*)((char*)instBuf.contents + i * 64);
                for (int k = 0; k < 12; k++)
                    f[k] = in.xform[k];
                uint32_t* u = (uint32_t*)((char*)instBuf.contents + i * 64 + 48);
                u[0] = 0;
                u[1] = 0xff;
                u[2] = 0;
                u[3] = (uint32_t)geoms[in.geom].blasIndex;
                float* R = (float*)instRot.contents + i * 9;
                for (int k = 0; k < 9; k++)
                    R[k] = in.rot[k];
            }
            id<MTLCommandBuffer> cb = [queue commandBuffer];
            id<MTLAccelerationStructureCommandEncoder> ae = [cb accelerationStructureCommandEncoder];
            for (size_t gi = 0; gi < scene.geometries.size(); ++gi) {
                auto& g = scene.geometries[gi];
                if (!g.dynamic)
                    continue;
                auto& gg = geoms[gi];
                memcpy(gg.vbuf.contents, g.verts.data(), g.verts.size() * 4);
                memcpy((char*)gNormal.contents + (size_t)gg.triBase * 9 * 4, g.normals.data(), g.normals.size() * 4);
                MTLAccelerationStructureTriangleGeometryDescriptor* gd = [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
                gd.vertexBuffer = gg.vbuf;
                gd.vertexStride = 12;
                gd.vertexFormat = MTLAttributeFormatFloat3;
                gd.triangleCount = gg.triCount;
                MTLPrimitiveAccelerationStructureDescriptor* pd = [MTLPrimitiveAccelerationStructureDescriptor descriptor];
                pd.geometryDescriptors = @[ gd ];
                pd.usage = MTLAccelerationStructureUsageRefit;
                [ae refitAccelerationStructure:blas[gg.blasIndex] descriptor:pd destination:blas[gg.blasIndex] scratchBuffer:gg.scratch scratchBufferOffset:0];
            }
            [ae endEncoding];
            id<MTLAccelerationStructureCommandEncoder> te = [cb accelerationStructureCommandEncoder];
            [te buildAccelerationStructure:tlas descriptor:idesc scratchBuffer:tlasScratch scratchBufferOffset:0];
            [te endEncoding];
            // Commit WITHOUT blocking: the render command buffer is submitted to the same queue right after, so it
            // executes after this AS build completes (in-order) -- no CPU stall needed here. render() waits once.
            [cb commit];
        }
    }

    void fillUniforms(const MetalCameraParams& cam, int tw, int th, int numLights) {
        float* uf = (float*)uni.contents;
        c3(uf, cam.origin[0], cam.origin[1], cam.origin[2]);
        c3(uf + 4, cam.right[0], cam.right[1], cam.right[2]);
        c3(uf + 8, cam.up[0], cam.up[1], cam.up[2]);
        c3(uf + 12, cam.forward[0], cam.forward[1], cam.forward[2]);
        c3(uf + 16, cam.ambient[0], cam.ambient[1], cam.ambient[2]);
        uf[20] = cam.tanHalfV;
        uint32_t* ui = (uint32_t*)uni.contents;
        ui[21] = tw;
        ui[22] = th;
        ui[23] = (uint32_t)(cam.aa < 1 ? 1 : cam.aa);
        ui[24] = (uint32_t)(numLights < 0 ? 0 : numLights);
        ui[25] = (uint32_t)(cam.mode < 0 ? 0 : cam.mode);
        uf[26] = cam.lidarHFov;
        uf[27] = cam.lidarVMin;
        uf[28] = cam.lidarVMax;
        uf[29] = cam.maxDist;
        ui[30] = (uint32_t)(cam.lidarSampleRadius < 1 ? 1 : cam.lidarSampleRadius);
        uf[31] = cam.lidarHDiv;
        uf[32] = cam.lidarVDiv;
        ui[33] = (uint32_t)(cam.lidarReturnMode < 0 ? 0 : cam.lidarReturnMode);
        ui[34] = (uint32_t)(cam.lensModel < 0 ? 0 : cam.lensModel);
        uf[35] = cam.dk1;
        uf[36] = cam.dk2;
        uf[37] = cam.dk3;
        ui[38] = envValid ? 1u : 0u;
        ui[39] = (uint32_t)(cam.bgMode < 0 ? 0 : cam.bgMode);
        uf[40] = cam.bgZenith[0];
        uf[41] = cam.bgZenith[1];
        uf[42] = cam.bgZenith[2];
        uf[43] = cam.bgHorizon[0];
        uf[44] = cam.bgHorizon[1];
        uf[45] = cam.bgHorizon[2];
        uf[46] = cam.fogColor[0];
        uf[47] = cam.fogColor[1];
        uf[48] = cam.fogColor[2];
        uf[49] = cam.fogScatter;
        ui[50] = cam.useGi ? 1u : 0u;
        uf[51] = cam.exposure;
        uf[52] = cam.vignette;
        uf[53] = cam.apertureR;
        uf[54] = cam.focalDist;
        uf[55] = cam.noiseSigma;
        uf[56] = cam.envIntensity > 0.f ? cam.envIntensity : 1.f;
        uf[57] = cam.gamma > 0.f ? cam.gamma : 2.2f;
        uf[58] = cam.clipNear > 0.f ? cam.clipNear : 0.f;
        ui[59] = cam.rngSeedLo;
        ui[60] = cam.rngSeedHi;
        ui[61] = cam.integratorPath ? 1u : 0u;
        ui[62] = (uint32_t)(cam.hitLimit > 0 ? cam.hitLimit : 1);
        ui[63] = (uint32_t)cam.lidarBeamShape;
    }
    void encodeTrace(id<MTLCommandBuffer> cb, id<MTLTexture> tex, int tw, int th) {
        id<MTLComputeCommandEncoder> e = [cb computeCommandEncoder];
        [e setComputePipelineState:pso];
        [e setBuffer:uni offset:0 atIndex:0];
        [e setBuffer:gNormal offset:0 atIndex:1];
        [e setBuffer:gAlbedo offset:0 atIndex:2];
        [e setBuffer:instRot offset:0 atIndex:3];
        [e setAccelerationStructure:tlas atBufferIndex:4];
        [e setBuffer:nBaseBuf offset:0 atIndex:5];
        [e setBuffer:matBuf offset:0 atIndex:6];
        [e setBuffer:tintBuf offset:0 atIndex:7];
        [e setBuffer:gUVBuf offset:0 atIndex:8];
        [e setBuffer:gTexId offset:0 atIndex:9];
        [e setBuffer:lightsBuf offset:0 atIndex:10];
        [e setBuffer:instIdsBuf offset:0 atIndex:11];
        [e setBuffer:gOpacity offset:0 atIndex:12];
        [e setBuffer:gRoughness offset:0 atIndex:13];
        [e setBuffer:gMetallicBuf offset:0 atIndex:14];
        [e setBuffer:gRoughTexIdBuf offset:0 atIndex:15];
        [e setBuffer:gMetalTexIdBuf offset:0 atIndex:16];
        [e setBuffer:gOpacityTexIdBuf offset:0 atIndex:17];
        [e setBuffer:gTangentBuf offset:0 atIndex:18];
        [e setBuffer:gNormalTexIdBuf offset:0 atIndex:19];
        [e setBuffer:gSpecularBuf offset:0 atIndex:20];
        [e setBuffer:gEmissiveBuf offset:0 atIndex:21];
        [e setBuffer:gTexScaleBuf offset:0 atIndex:22];
        [e setBuffer:gKsTexIdBuf offset:0 atIndex:23];
        [e setBuffer:gKeTexIdBuf offset:0 atIndex:24];
        [e setBuffer:gBlendKdTexIdBuf offset:0 atIndex:25];
        [e setBuffer:gBlendWeightTexIdBuf offset:0 atIndex:26];
        id<MTLTexture> tarr[64];
        for (int k = 0; k < 64; k++)
            tarr[k] = (k < (int)texList.size()) ? texList[k] : whiteTex;
        [e setTextures:tarr withRange:NSMakeRange(0, 64)];
        [e setSamplerState:samp atIndex:0];
        for (auto t : texList)
            [e useResource:t usage:MTLResourceUsageRead];
        for (auto a : blas)
            [e useResource:a usage:MTLResourceUsageRead];
        [e setTexture:tex atIndex:64];
        [e setTexture:(envValid && envTex ? envTex : whiteTex) atIndex:65];
        [e dispatchThreadgroups:MTLSizeMake((tw + 7) / 8, (th + 7) / 8, 1) threadsPerThreadgroup:MTLSizeMake(8, 8, 1)];
        [e endEncoding];
    }
    void render(const MetalCameraParams& cam, const MetalLightGPU* lights, int numLights, int w, int h, float* out) {
        @autoreleasepool {
            int cap = numLights > 0 ? numLights : 1;
            if (!lightsBuf || (int)(lightsBuf.length / (int)sizeof(MetalLightGPU)) < cap)
                lightsBuf = [dev newBufferWithLength:cap * sizeof(MetalLightGPU) options:MTLResourceStorageModeShared];
            if (numLights > 0)
                memcpy(lightsBuf.contents, lights, numLights * sizeof(MetalLightGPU));
            id<MTLTexture> tex = offscreen(w, h);
            fillUniforms(cam, w, h, numLights);
            id<MTLCommandBuffer> cb = [queue commandBuffer];
            encodeTrace(cb, tex, w, h);
            id<MTLTexture> outTex = tex;
            if (cam.useDenoiser && dpso && cam.mode == 0) {  // portable despeckle/denoise pass (color mode only)
                if (!offTex2 || (int)offTex2.width != w || (int)offTex2.height != h) {
                    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA32Float width:w height:h mipmapped:NO];
                    td.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
                    td.storageMode = MTLStorageModeShared;
                    offTex2 = [dev newTextureWithDescriptor:td];
                }
                uint32_t dims[2] = {(uint32_t)w, (uint32_t)h};
                id<MTLComputeCommandEncoder> de = [cb computeCommandEncoder];
                [de setComputePipelineState:dpso];
                [de setTexture:tex atIndex:0];
                [de setTexture:offTex2 atIndex:1];
                [de setBytes:dims length:8 atIndex:0];
                [de dispatchThreadgroups:MTLSizeMake((w + 7) / 8, (h + 7) / 8, 1) threadsPerThreadgroup:MTLSizeMake(8, 8, 1)];
                [de endEncoding];
                outTex = offTex2;
            }
            [cb commit];
            [cb waitUntilCompleted];
            [outTex getBytes:out bytesPerRow:w * 16 fromRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0];  // RGBA32F
        }
    }
};

ChMetalRTRenderer::ChMetalRTRenderer(void* mtlDevice, void* mtlQueue) {
    p = new Impl();
    p->dev = (__bridge id<MTLDevice>)mtlDevice;
    p->queue = (__bridge id<MTLCommandQueue>)mtlQueue;
    if (!p->dev || !p->queue)
        return;
    NSError* e = nil;
    MTLCompileOptions* co = [MTLCompileOptions new];
    if (@available(macOS 14.0, *))
        co.languageVersion = MTLLanguageVersion3_1;
    else
        co.languageVersion = MTLLanguageVersion2_4;
    id<MTLLibrary> lib = [p->dev newLibraryWithSource:@(kRaytraceMSL) options:co error:&e];
    if (!lib) {
        NSLog(@"Chrono::Sensor Metal RT shader error: %@", e);
        return;
    }
    p->pso = [p->dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"computeMain"] error:&e];
    p->dpso = [p->dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"denoiseMain"] error:&e];  // despeckle/denoise pass
    MTLSamplerDescriptor* sd = [MTLSamplerDescriptor new];
    sd.minFilter = MTLSamplerMinMagFilterLinear;
    sd.magFilter = MTLSamplerMinMagFilterLinear;
    sd.mipFilter = MTLSamplerMipFilterLinear;
    sd.sAddressMode = MTLSamplerAddressModeRepeat;
    sd.tAddressMode = MTLSamplerAddressModeRepeat;
    p->samp = [p->dev newSamplerStateWithDescriptor:sd];
    p->uni = [p->dev newBufferWithLength:256 options:MTLResourceStorageModeShared];
    MTLTextureDescriptor* wd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:1 height:1 mipmapped:NO];
    wd.usage = MTLTextureUsageShaderRead;
    p->whiteTex = [p->dev newTextureWithDescriptor:wd];
    uint8_t wp[4] = {255, 255, 255, 255};
    [p->whiteTex replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:wp bytesPerRow:4];
}

ChMetalRTRenderer::~ChMetalRTRenderer() {
    delete p;
}
bool ChMetalRTRenderer::Valid() const {
    return p && p->dev && p->pso;
}
void ChMetalRTRenderer::Build(const MetalRenderScene& scene) {
    if (Valid())
        p->build(scene);
}
void ChMetalRTRenderer::UpdateDynamic(const MetalRenderScene& scene) {
    if (!Valid())
        return;
    // Bodies added or removed at runtime (ChSensorManager::ReconstructScenes) change the scene
    // topology; the accel structures and per-instance buffers must be rebuilt, not refitted.
    if (p->topologyChanged(scene))
        p->build(scene);
    else
        p->updateDynamic(scene);
}
void ChMetalRTRenderer::Render(const MetalCameraParams& cam, const MetalLightGPU* lights, int numLights, int w, int h, float* out) {
    if (Valid())
        p->render(cam, lights, numLights, w, h, out);
}
void ChMetalRTRenderer::SetEnvMap(const std::string& path) {
    if (Valid())
        p->setEnvMap(path);
}

}  // namespace sensor
}  // namespace chrono
