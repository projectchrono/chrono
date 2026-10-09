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
// Metal Shading Language kernel for Chrono::Sensor Metal RT cameras.
// Inline ray query (hardware ray tracing). Modes: 0 color, 1 depth, 2 normal,
// 3 segmentation, 4 lidar, 5 radar, 6 phys-camera RGBD. Output is RGBA32Float;
// the filter converts to the sensor's host buffer format.
// =============================================================================

#ifndef CH_METAL_RT_SHADER_MSL_H
#define CH_METAL_RT_SHADER_MSL_H

static const char* kRaytraceMSL = R"MSLGEN(
#include <metal_stdlib>
#include <metal_raytracing>
using namespace metal;
using namespace raytracing;

struct Uniforms {
    float3 camPos; float3 camRight; float3 camUp; float3 camForward; float3 ambient;
    float tanHalfFov; uint width; uint height; uint aa; uint numLights; uint mode;
    float lidarHFov; float lidarVMin; float lidarVMax; float maxDist;
    uint lidarSampleRadius; float lidarHDiv; float lidarVDiv; uint lidarReturnMode;
    uint lensModel; float dk1; float dk2; float dk3;  // 0 pinhole, 1 FOV(fisheye), 2 radial
    uint hasEnv;                                       // 1 = sample envTex (HDR equirect) for sky/reflections
    uint bgMode;                                       // background when no env: 0 procedural, 1 gradient, 2 solid
    packed_float3 bgZenith; packed_float3 bgHorizon;   // gradient (zenith/horizon) or solid (zenith)
    packed_float3 fogColor; float fogScatter;          // exponential fog (0 scatter = off)
    uint useGi;                                        // 1 = path-traced global illumination
    float exposure;                                    // linear exposure/gain (1 = none)
    float vignette;                                    // vignette strength (0 = none)
    float apertureR;                                   // lens aperture radius for depth of field (0 = pinhole)
    float focalDist;                                   // DoF focal distance
    float noiseSigma;                                  // gaussian sensor noise stddev (0 = none)
    float envIntensity;                                // environment-map radiance scale (OptiX AddEnvironmentLight intensity_scale)
    float gamma;                                       // output gamma (OptiX camera.gamma; 2.2 = sRGB, 1 = linear)
    float clipNear;                                    // lidar/radar near clip: returns closer than this are ignored
                                                       // (OptiX passes ChLidarSensor::GetClipNear() as optixTrace tmin,
                                                       //  which is what lets a sensor sit inside its own housing)
    uint rngSeedLo; uint rngSeedHi;                    // halves of ChSensorManager::GetDeterministicSeed for this render
    uint integratorPath;                               // 1 = Integrator::PATH. OptiX dispatches on the integrator
                                                       // (camera_shader.cuh), not on use_gi, so LEGACY + use_gi must
                                                       // stay in the legacy shader rather than switch to path tracing.
    uint hitLimit;                                     // camera-ray surface-hit budget. OptiX's DefaultCameraPRD
                                                       // starts PRIMARY rays at depth 2 and only spawns a child
                                                       // while depth+1 < max_depth, so a camera ray gets
                                                       // max_depth-2 hits, and radiance past the budget is
                                                       // dropped to black rather than falling through to the
                                                       // background. ChFilterVulkanRTRender.cpp encodes the same
                                                       // rule in OptiXCameraHitLimit().
    uint lidarBeamShape;                               // 0 = RECTANGULAR, 1 = ELLIPTICAL
};

// Fold both halves of the 64-bit ChSensorManager stream seed into this shader's 32-bit hash domain,
// so two cameras in one scene do not draw the same samples and a fixed base seed reproduces a run.
// Matches the role of camera_rng_key() in the Vulkan RT raygen shader.
static inline uint cameraRngKey(constant Uniforms& u) {
    return u.rngSeedLo ^ (u.rngSeedHi * 0x9e3779b9u);
}

// Camera ray direction for a normalized image coord (nx includes aspect, ny does not).
static inline float3 camRayDir(float nx, float ny, constant Uniforms& u) {  // nx/ny may be remapped below
    float3 right=float3(u.camRight), up=float3(u.camUp), fwd=float3(u.camForward);
    if (u.lensModel==1u) {
        // FOV_LENS. camera_raygen.cu remaps the image-plane coordinate and then projects it through the
        // ordinary pinhole, rather than using an equidistant fisheye. Its uv carries the aspect ratio on
        // Y where nx/ny carry it on X, so convert into its convention, remap, and convert back.
        float aspect = float(u.width) / float(u.height);
        float2 uv = float2(nx, ny) / aspect;
        // NB: the guard really is asymmetric upstream -- uv.x is compared without abs() -- so the
        // horizontal centre line of the left half is left undistorted. Reproduced deliberately.
        if (uv.x > 1e-5 || abs(uv.y) > 1e-5) {
            float tanH = aspect * u.tanHalfFov;             // tan(hFOV/2)
            float focal = 1.0 / tanH;
            float2 uvn = uv / focal;
            float rd = length(uvn);
            if (rd > 1e-9) {
                float hFOV = 2.0 * atan(tanH);
                float ru = tan(rd * hFOV) / (2.0 * tanH);
                uv = uvn * (ru / rd) * focal;
            }
        }
        nx = uv.x * aspect;
        ny = uv.y * aspect;
    }
    float k=1.0;
    if (u.lensModel==2u) { float r2=nx*nx+ny*ny; k=1.0+u.dk1*r2+u.dk2*r2*r2+u.dk3*r2*r2*r2; }  // RADIAL
    float px=nx*k*u.tanHalfFov, py=ny*k*u.tanHalfFov;
    return normalize(px*right+py*up+fwd);
}
// type: 0 point, 1 directional, 2 spot, 3 disk, 4 rectangle. KEEP IN SYNC with MetalLightGPU
// in metal/ChMetalRTRenderer.h (same 16 floats / 64 bytes, uploaded raw).
// attenScale mirrors OptiX's atten_scale (max_range > 0 ? 0.01*max_range^2 : 1) and constColor
// mirrors OptiX's const_color (!=0 -> no distance attenuation at all).
struct Light { packed_float3 pos; float range; packed_float3 color; float type; packed_float3 dir; float cosOuter; float cosInner; float p0; float attenScale; float constColor; };

static inline float3 skycol(float3 d, texture2d<float> env, sampler samp, constant Uniforms& u){
  float3 dn=normalize(d);
  if(u.hasEnv!=0u){                                      // HDR equirectangular environment map (matches OptiX miss.cu)
    float uu=atan2(dn.y,dn.x)*(0.5/M_PI_F)+0.5;
    float v=acos(clamp(dn.z,-1.0,1.0))*(1.0/M_PI_F);     // z-up: zenith -> v=0 (top of image)
    // .hdr is linear radiance -> sample linearly and scale by intensity, matching OptiX's environment LIGHT
    // (ChOptixEnvironmentLight: L = tex.rgb * intensity_scale). NB: OptiX's LEGACY background (miss.cu) does a
    // pow(2.2) instead, but scenes that use the env for lighting (e.g. teleopcity, GI) read the map linearly
    // and brighter -- that is what the driving demos compare against, so we match the light path here.
    return max(env.sample(samp,float2(uu,v)).rgb,0.0) * u.envIntensity;
  }
  if(u.bgMode==1u){ float m=max(0.0,dn.z); return m*float3(u.bgZenith)+(1.0-m)*float3(u.bgHorizon); }  // GRADIENT (OptiX miss.cu)
  return float3(u.bgZenith);                                                                            // SOLID (OptiX default = black)
}

// Microfacet BRDF terms ported verbatim from Chrono's OptiX shader_utils.cuh (GGX D, Hammon-Smith G;
// the 1/pi in D and the 4*NdV*NdL in G are omitted exactly as in the OptiX code).
static inline float NormalDist(float NdH, float rough){ float r2=rough*rough; float d=NdH*NdH*(r2-1.0)+1.0; return r2/(d*d); }
static inline float HammonSmith(float NdV, float NdL, float rough){ float den=mix(2.0*abs(NdV)*abs(NdL), abs(NdL)+abs(NdV), rough); return 0.5/den; }

// PCG hash RNG + cosine-weighted hemisphere sampling for the path-traced GI integrator.
static inline uint pcg(thread uint& s){ s=s*747796405u+2891336453u; uint w=((s>>((s>>28u)+4u))^s)*277803737u; return (w>>22u)^w; }
static inline float rndf(thread uint& s){ return float(pcg(s))*(1.0/4294967296.0); }
static inline float3 cosineHemisphere(float3 n, float u1, float u2){
  float r=sqrt(u1), th=6.28318530718*u2;
  float3 t=normalize(cross((abs(n.x)>0.9?float3(0,1,0):float3(1,0,0)), n)); float3 b=cross(n,t);
  return normalize(t*(r*cos(th)) + b*(r*sin(th)) + n*sqrt(max(0.0,1.0-u1)));
}
static inline float gaussf(thread uint& s){ float u1=max(rndf(s),1e-6), u2=rndf(s); return sqrt(-2.0*log(u1))*cos(6.28318530718*u2); }
// Physical-camera post: linear exposure/gain, radial vignette, sRGB gamma, then gaussian sensor noise.
static inline float3 cameraPost(float3 lin, uint2 tid, constant Uniforms& u, thread uint& seed){
  lin *= (u.exposure>0.0 ? u.exposure : 1.0);
  if(u.vignette>0.0){ float2 p=(float2(float(tid.x),float(tid.y))+0.5)/float2(float(u.width),float(u.height))*2.0-1.0; lin *= max(0.0, 1.0 - u.vignette*dot(p,p)); }
  float3 o = pow(max(lin,0.0), 1.0/max(u.gamma,0.01));   // output gamma (OptiX camera.gamma; default 2.2)
  if(u.noiseSigma>0.0){ o += float3(gaussf(seed),gaussf(seed),gaussf(seed))*u.noiseSigma; }
  return clamp(o,0.0,1.0);
}
// Physics-based camera (mode 6) post: gamma ONLY, unclamped -- ports phys_cam_raygen.cu, whose last line is
//   rgbd_buffer[px] = make_half4(pow(c.x,1/gamma), pow(c.y,1/gamma), pow(c.z,1/gamma), distance).
// Everything else (exposure time, aperture number, vignetting, noise, ISO/CRF) is modelled downstream by the
// ChFilterPhysCamera* stages, so the scene-level exposure/vignette/noise/DoF knobs must NOT be applied here.
static inline float3 physCamPost(float3 lin, constant Uniforms& u){
  return pow(max(lin,0.0), 1.0/max(u.gamma,0.01));
}

struct Hit {
    bool sky;
    float3 albedo;
    float3 n;
    float3 pos;
    uint mat;
    float dist;
    uint inst;
    float opacity;
    float rough;
    float metallic;
    float3 ks;
    float useSpec;
    float3 emissive;
};

static Hit trace(ray r, instance_acceleration_structure accel,
   device const packed_float3* gN, device const packed_float3* gA, device const packed_float3* tint,
   device const float* iR, device const uint* nBase, device const uint* matI,
   device const float* gUV, device const int* gTexId, device const float* gOpacity, device const float* gRough, device const float* gMetallic, device const int* gRoughTexId,
   device const int* gMetalTexId, device const int* gOpacityTexId, device const packed_float3* gTangent, device const int* gNormalTexId, device const float* gSpecular,
   device const float* gEmissive, device const float* gTexScale, device const int* gKsTexId, device const int* gKeTexId, device const int* gBlendKdTexId,
   device const int* gBlendWeightTexId, array<texture2d<float>, 64> texs, sampler samp, texture2d<float> env, constant Uniforms& u){
  Hit h;
  h.sky=true;
  h.albedo=float3(0);
  h.n=float3(0,0,1);
  h.pos=r.origin;
  h.mat=0;
  h.dist=0;
  h.inst=0;
  h.opacity=1.0;
  h.rough=1.0;
  h.metallic=0.0;
  h.ks=float3(0);
  h.useSpec=0.0;
  h.emissive=float3(0);
  intersector<triangle_data,instancing> it; it.assume_geometry_type(geometry_type::triangle); it.force_opacity(forced_opacity::opaque);
  // OptiX (material_shaders.cu) uses the geometric world normal as-is -- it does NOT face-forward toward
  // the viewer. Face-forwarding made foliage-card shading view-dependent (trees lit only when zoomed out).
  float3 o0=r.origin;                                    // original origin (for true hit distance)
  for(int skip=0; skip<24; skip++){                      // loop to pass through alpha-cut-out (foliage) texels
    auto res=it.intersect(r,accel);
    if(res.type==intersection_type::none){ h.sky=true; h.albedo=skycol(r.direction,env,samp,u); return h; }
    uint id=res.instance_id, prim=res.primitive_id; float2 bc=res.triangle_barycentric_coord; float w0=1.0-bc.x-bc.y;
    uint tri=nBase[id]+prim;
    float3 on=normalize(w0*float3(gN[tri*3])+bc.x*float3(gN[tri*3+1])+bc.y*float3(gN[tri*3+2]));     // object normal
    float3 ot=normalize(w0*float3(gTangent[tri*3])+bc.x*float3(gTangent[tri*3+1])+bc.y*float3(gTangent[tri*3+2]));  // object tangent
    uint b=id*9u; float3 c0=float3(iR[b],iR[b+1],iR[b+2]),c1=float3(iR[b+3],iR[b+4],iR[b+5]),c2=float3(iR[b+6],iR[b+7],iR[b+8]);
    float3 hit=r.origin+res.distance*r.direction;
    uint mat=matI[id];
    float2 uv=w0*float2(gUV[tri*6],gUV[tri*6+1])+bc.x*float2(gUV[tri*6+2],gUV[tri*6+3])+bc.y*float2(gUV[tri*6+4],gUV[tri*6+5]); uv.y=1.0-uv.y;
    uv *= float2(gTexScale[tri*2],gTexScale[tri*2+1]);    // per-material UV scale (OptiX tex_scale)
    int texId=gTexId[tri]; float3 base;
    if(texId>=0){ float4 t=texs[texId].sample(samp,uv);
      if(t.a < 0.1){ r.origin=hit+r.direction*max(5e-3,length(hit)*3e-5); continue; }  // alpha cutout (coord-scaled advance)
      base=pow(t.rgb/max(t.a,1e-3),2.2); } else base=float3(gA[tri]);  // un-premultiply, then linearize sRGB tex (OptiX Pow 2.2)
    // Normal map (object space, matches OptiX material_shaders.cu): perturb the object normal, then transform to world.
    int ntx=gNormalTexId[tri];
    if(ntx>=0){ float3 bit=normalize(cross(on,ot)); float3 nd=texs[ntx].sample(samp,uv).rgb*2.0-1.0; on=normalize(nd.x*ot+nd.y*bit+nd.z*on); }
    float3 n=normalize(c0*on.x+c1*on.y+c2*on.z);          // world normal (after any normal-map perturbation)
    float3 albedo=base*float3(tint[id]);
    // Weight-blended materials (OptiX): mix in the 2nd-layer albedo by its weight texture (foliage/splatting)
    int bkt=gBlendKdTexId[tri], bwt=gBlendWeightTexId[tri];
    if(bkt>=0 && bwt>=0){ float bw=clamp(texs[bwt].sample(samp,uv).r,0.0,1.0);
      float4 b1=texs[bkt].sample(samp,uv); float3 kd1=pow(b1.rgb/max(b1.a,1e-3),2.2)*float3(tint[id]);
      albedo=mix(albedo,kd1,bw); }
    // Roughness/metallic come from PBR maps when present (OptiX samples map_Pr/map_Pm) -- this is what makes
    // the Audi paint glossy metallic; scalar GetRoughness()/GetMetallic() are only the fallback. Data maps
    // are linear (no sRGB), so sample .r directly.
    float rough=gRough[tri]; int rtx=gRoughTexId[tri]; if(rtx>=0) rough=texs[rtx].sample(samp,uv).r;
    float metal=gMetallic[tri]; int mtx=gMetalTexId[tri]; if(mtx>=0) metal=texs[mtx].sample(samp,uv).r;
    float opac=gOpacity[tri]; int otx=gOpacityTexId[tri]; if(otx>=0) opac=texs[otx].sample(samp,uv).r;  // map_d opacity (OptiX: opacity_tex overrides) -> glass/windows
    // Specular workflow (Ks/ks_tex, not sRGB-linearized -- matches OptiX legacy) + emissive (Ke*power, ke_tex)
    int ktx=gKsTexId[tri]; float3 ksv = ktx>=0 ? texs[ktx].sample(samp,uv).rgb : float3(gSpecular[tri*4],gSpecular[tri*4+1],gSpecular[tri*4+2]);
    int etx=gKeTexId[tri]; float3 kev = etx>=0 ? texs[etx].sample(samp,uv).rgb : float3(gEmissive[tri*4],gEmissive[tri*4+1],gEmissive[tri*4+2]);
    h.sky=false; h.dist=length(hit-o0); h.inst=id; h.albedo=albedo; h.n=n; h.pos=hit; h.mat=mat; h.opacity=opac; h.rough=rough; h.metallic=metal;
    h.ks=ksv; h.useSpec=gSpecular[tri*4+3]; h.emissive=kev*gEmissive[tri*4+3];
    return h;
  }
  h.sky=true; h.albedo=skycol(r.direction,env,samp,u); return h;
}

// Alpha-aware shadow ray: transparent (alpha-cutout) leaf/sign texels do NOT block light, exactly like
// Chrono's OptiX ShadowShader (shadow_shader.cuh: tex.w<1e-6 -> transparency 0 -> passes). Treating the
// foliage cards as opaque here caused dense canopy self-shadowing -> trees black at the top.
static float shadowRay(float3 origin, float3 dir, float maxd, float minD, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale,
   array<texture2d<float>, 64> texs, sampler samp){
  intersector<triangle_data,instancing> sit; sit.assume_geometry_type(geometry_type::triangle); sit.force_opacity(forced_opacity::opaque);
  float remaining = maxd; float atten = 1.0;
  for(int i=0;i<16;i++){
    ray sr; sr.origin=origin; sr.direction=dir; sr.min_distance=minD; sr.max_distance=remaining;
    auto res=sit.intersect(sr,accel);
    if(res.type==intersection_type::none) return atten;      // reached the light unobstructed
    uint tri=nBase[res.instance_id]+res.primitive_id; int texId=gTexId[tri];
    float2 bc=res.triangle_barycentric_coord; float w0=1.0-bc.x-bc.y;
    float2 uv=w0*float2(gUV[tri*6],gUV[tri*6+1])+bc.x*float2(gUV[tri*6+2],gUV[tri*6+3])+bc.y*float2(gUV[tri*6+4],gUV[tri*6+5]);
    uv.y=1.0-uv.y; uv*=float2(gTexScale[tri*2],gTexScale[tri*2+1]);
    float opac=gOpacity[tri];
    // shadow_shader.cuh: an alpha-cut-out texel (kd alpha ~ 0) drops transparency to 0 so light passes;
    // an opacity texture overrides the material value outright.
    if(texId>=0 && texs[texId].sample(samp,uv).a < 1e-6) opac=0.0;
    int otx=gOpacityTexId[tri]; if(otx>=0) opac=texs[otx].sample(samp,uv).r;
    // OptiX attenuates CONTINUOUSLY -- atten = 1 - opacity, multiplied along the ray -- rather than
    // treating a surface as either fully blocking or fully clear. A 0.35-opacity pane passes 65% of the
    // light, and two of them pass 42%.
    atten *= (1.0 - opac);
    if(atten <= 0.01) return 0.0;                             // params.importance_cutoff
    float adv=res.distance+minD; origin+=dir*adv; remaining-=adv;
    if(remaining<=minD) return atten;
  }
  return atten;
}

// Direct-illumination shading, ported from Chrono's OptiX legacy shader (camera_legacy_shader.cuh):
// GGX Cook-Torrance specular + (1-F) Lambert diffuse per light, plus the OptiX "flashlight" ambient
// (a view-facing NdV term + an up-hemisphere term). No foliage/material special-casing -- the NdV
// ambient is exactly what keeps vertical tree cards from going dark under an overhead sun.
// Per-light direct illumination only (no ambient) -- used by both the legacy shader and the GI integrator.
static float3 directLighting(Hit h, float3 view, constant Uniforms& u, device const Light* lights, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale,
   array<texture2d<float>, 64> texs, sampler samp, thread uint& seed){
  float3 N = h.n;
  float NdV = max(dot(N, view), 0.0);
  // Raw, as OptiX passes mat.roughness straight to NormalDist/HammonSmith with no clamp.
  float rough = h.rough;
  float metallic = clamp(h.metallic, 0.0, 1.0);
  // OptiX supports two workflows: specular (F0 = Ks*0.08) or metallic/roughness (default).
  bool specWF = h.useSpec > 0.5;
  float3 F0 = specWF ? (h.ks*0.08) : (metallic*h.albedo + (1.0-metallic)*float3(0.04));
  float3 diffAlbedo = specWF ? h.albedo : (h.albedo * (1.0-metallic));  // metals have no subsurface diffuse
  float3 col = float3(0.0);
  for(uint i=0;i<u.numLights;i++){
    Light L=lights[i]; float3 toL; float dL; float atten=1.0;
    if(L.type>3.5){ // RECTANGLE area light (OptiX ChOptixRectangleLight): center=pos, edges=dir & (cosOuter,cosInner,p0)
      // ChOptixRectangleLight picks ONE of two strategies per sample and then does NOT divide by the
      // mixture pdf it computes -- camera_legacy_shader.cuh only tests ls.pdf > 0 as a validity flag.
      // The strategy choice therefore scales the estimate rather than reducing its variance, so the
      // mixture has to be reproduced as written. Note geom_term carries no area/dist^2 factor: an
      // area-sampling estimator that divides by its pdf would be physically better and would NOT match.
      float3 e1=float3(L.dir), e2=float3(L.cosOuter,L.cosInner,L.p0);      // full edge vectors
      float3 lpos=float3(L.pos), lnrm=normalize(cross(e1,e2));
      float area=length(cross(e1,e2));
      float3 toC=lpos-h.pos; float dC=length(toC);
      float cosLc=max(0.0, dot(lnrm, -(toC/max(dC,1e-6))));
      float omega=area*cosLc/max(dC*dC,1e-6);
      float Plight=clamp(omega/(omega+1.0), 0.10, 0.90);                  // "magic formula", clamped as upstream
      float cosL;
      if(rndf(seed) < Plight){                                            // strategy 1: uniform point on the area
        float3 p=lpos + (rndf(seed)-0.5)*e1 + (rndf(seed)-0.5)*e2;
        float3 d=p-h.pos; dL=length(d); toL=d/max(dL,1e-6);
        cosL=dot(lnrm,-toL);
        if(dot(N,toL)<0.0 || cosL<0.0) continue;
      } else {                                                            // strategy 2: cosine-hemisphere direction
        toL=cosineHemisphere(N, rndf(seed), rndf(seed));
        cosL=dot(lnrm,-toL);
        if(cosL<0.0) continue;
        float denom=dot(toL,lnrm);                                        // analytic ray/rectangle intersection
        if(abs(denom)<1e-9) continue;
        float t=dot(lpos-h.pos,lnrm)/denom;
        if(t<=0.0) continue;
        float3 rel=(h.pos+t*toL)-lpos;
        float a=dot(rel,e1)/max(dot(e1,e1),1e-12), b=dot(rel,e2)/max(dot(e2,e2),1e-12);
        if(abs(a)>0.5 || abs(b)>0.5) continue;                            // direction misses: no contribution
        dL=t;
      }
      atten = cosL * ((L.constColor>0.5) ? 1.0 : L.attenScale/max(dL*dL,1e-4));
    } else if(L.type>2.5){ // DISK area light (OptiX ChOptixDiskLight): center=pos, normal=dir, radius=p0
      float3 n=normalize(float3(L.dir));
      float3 t=normalize(cross(abs(n.z)<0.99?float3(0,0,1):float3(1,0,0), n)), b=cross(n,t);
      float rr=L.p0*sqrt(rndf(seed)), ph=6.28318530718*rndf(seed);         // uniform disk sample
      float3 p=float3(L.pos)+(t*cos(ph)+b*sin(ph))*rr;
      float3 d=p-h.pos; dL=length(d); toL=d/max(dL,1e-4);
      float cosL=dot(n,-toL); if(cosL<=0.0) continue;
      float area=3.14159265*L.p0*L.p0; atten=cosL*area/max(dL*dL,1e-6);
      if(L.constColor<0.5) atten *= L.attenScale/max(dL*dL,1e-4);          // see the rectangle case above
    } else if(L.type>1.5){ // SPOT: point-light falloff * angular cone falloff (OptiX ChOptixSpotLight)
      float3 d=float3(L.pos)-h.pos; dL=length(d); toL=d/max(dL,1e-4);
      float ang=acos(clamp(dot(-toL, normalize(float3(L.dir))),-1.0,1.0)); // angle from spot axis
      float angleRange=L.cosOuter, attenRate=L.cosInner;                   // (repurposed) full cone angle, 1/falloff-width
      if(2.0*ang > angleRange) atten=0.0;                                  // outside the cone (hard cutoff, always)
      else if(L.constColor<0.5) {
        // OptiX: geom_term = const_color ? 1 : (atten_scale/dist^2 * angle_atten^2). const_color skips BOTH
        // the distance and the angular falloff -- only the cone cutoff above survives (ChOptixSpotLight).
        float ai=(attenRate<0.0)?1.0:clamp(attenRate*(angleRange-2.0*ang),0.0,1.0);  // linear-in-angle, squared
        atten = (L.attenScale/max(dL*dL,1e-4)) * ai*ai;
      }
    } else if(L.type>0.5){ toL=normalize(-float3(L.pos)); dL=1e4; }       // DIRECTIONAL (no distance falloff)
    // POINT (OptiX ChOptixPointLight): geom_term = const_color ? 1 : atten_scale/dist^2
    else { float3 d=float3(L.pos)-h.pos; dL=length(d); toL=d/max(dL,1e-4); if(L.constColor<0.5) atten=L.attenScale/max(dL*dL,1e-4); }
    float NdL = dot(N, toL);
    if(NdL <= 0.0) continue;                                     // light below the surface (OptiX: NdL<0 -> L=0)
    // Coordinate-scaled shadow epsilon: the NADS course spans ~1600 m, where a fixed 1e-3 offset is
    // below float precision -> shadow acne -> spurious self-shadowing -> "extremely dark". OptiX hides
    // this with EnableDynamicOrigin (recenters coords); we scale the epsilon with |hit| instead.
    float eps = max(5e-3, length(h.pos)*3e-5);
    float sh = shadowRay(h.pos+N*eps, toL, dL-eps, eps, accel, nBase, gTexId, gUV, gOpacity, gOpacityTexId, gTexScale, texs, samp);
    // OptiX folds one NdL into ls.L inside every Ch*Light sampler and the shader applies another in
    // incoming_light_ray = ls.L * attenuation * NdL, so the cosine appears squared. The Vulkan RT
    // backend reproduces the same square deliberately. Match both.
    float3 incoming = float3(L.color) * (atten*sh) * NdL * NdL;   // ls.L (carries one NdL) * atten * NdL
    float3 hv = normalize(toL+view); float NdH=max(dot(N,hv),0.0), VdH=max(dot(view,hv),0.0);
    // OptiX applies Schlick ONLY in the specular workflow; the metallic workflow uses F0 directly
    // (camera_legacy_shader.cuh: F = metallic*Kd + (1-metallic)*0.04, with no angular term).
    float3 F = specWF ? (F0 + (float3(1.0)-F0)*pow(1.0-VdH,5.0)) : F0;
    col += (float3(1.0)-F) * diffAlbedo * incoming;              // diffuse
    col += F * NormalDist(NdH,rough) * HammonSmith(NdV,NdL,rough) * incoming;  // Cook-Torrance specular
  }
  return col;
}

// Legacy shading = OptiX "flashlight" ambient (view-facing NdV + up-hemisphere) + direct lighting.
static float3 lighting(Hit h, float3 view, constant Uniforms& u, device const Light* lights, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale,
   array<texture2d<float>, 64> texs, sampler samp, thread uint& seed){
  // Unclamped, as camera_legacy_shader.cuh leaves it. OptiX never face-forwards its normals, so a
  // surface seen from behind gets NdV ~ -1 and the ambient term goes NEGATIVE. That is reachable
  // whenever a back face is shaded through a transparent surface.
  float NdV = dot(h.n, view);
  float3 amb = u.ambient * (NdV + (dot(h.n,float3(0,0,1))*0.5+0.5)) * h.albedo;
  return amb + directLighting(h,view,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp,seed);
}

kernel void computeMain(uint2 tid [[thread_position_in_grid]], constant Uniforms& u [[buffer(0)]],
  device const packed_float3* gN [[buffer(1)]], device const packed_float3* gA [[buffer(2)]], device const float* iR [[buffer(3)]],
  instance_acceleration_structure accel [[buffer(4)]], device const uint* nBase [[buffer(5)]], device const uint* matI [[buffer(6)]],
  device const packed_float3* tint [[buffer(7)]], device const float* gUV [[buffer(8)]], device const int* gTexId [[buffer(9)]],
  device const Light* lights [[buffer(10)]], device const uint* instIds [[buffer(11)]], device const float* gOpacity [[buffer(12)]],
  device const float* gRough [[buffer(13)]], device const float* gMetallic [[buffer(14)]],
  device const int* gRoughTexId [[buffer(15)]], device const int* gMetalTexId [[buffer(16)]], device const int* gOpacityTexId [[buffer(17)]],
  device const packed_float3* gTangent [[buffer(18)]], device const int* gNormalTexId [[buffer(19)]],
  device const float* gSpecular [[buffer(20)]], device const float* gEmissive [[buffer(21)]], device const float* gTexScale [[buffer(22)]],
  device const int* gKsTexId [[buffer(23)]], device const int* gKeTexId [[buffer(24)]],
  device const int* gBlendKdTexId [[buffer(25)]], device const int* gBlendWeightTexId [[buffer(26)]],
  array<texture2d<float>,64> texs [[texture(0)]], sampler samp [[sampler(0)]], texture2d<float, access::write> outTex [[texture(64)]], texture2d<float> envTex [[texture(65)]]) {
  if(tid.x>=u.width||tid.y>=u.height) return; float aspect=float(u.width)/float(u.height);

  // primary ray for the pixel center (used by all non-color modes)
  float ncx=(2.0*(float(tid.x)+0.5)/float(u.width)-1.0)*aspect;
  float ncy=(2.0*(float(tid.y)+0.5)/float(u.height)-1.0);  // row 0 = BOTTOM, matching
                                                          // OptiX's camera_raygen.cu; the shared
                                                          // filters assume that convention
  float3 cdir=camRayDir(ncx,ncy,u);

  if(u.mode==1u){ // DEPTH: min(max_depth, distance); a miss returns max_depth.
    // Matches OptiX: depth_cam_shader.cuh does prd->depth = fminf(prd->max_depth, ray_dist) and
    // miss.cu sets prd->depth = prd->max_depth. u.maxDist carries ChDepthCamera::GetMaxDepth();
    // 0 means "unlimited", in which case a miss stays at 0 as before.
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
        gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
    float md=u.maxDist;
    float d = h.sky ? md : ((md>0.0) ? min(md, h.dist) : h.dist);
    outTex.write(float4(d,0,0,1), tid); return;
  }
  if(u.mode==2u){ // NORMAL (world-space; 0 = sky/miss)
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
        gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
    outTex.write(h.sky?float4(0,0,0,1):float4(h.n,1.0), tid); return;
  }
  if(u.mode==3u){ // SEGMENTATION (class id, instance id)
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
        gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
    float cls=0, inst=0; if(!h.sky){ cls=float(instIds[h.inst*2u]); inst=float(instIds[h.inst*2u+1u]); }
    outTex.write(float4(cls,inst,0,1), tid); return;
  }
  if(u.mode==4u){ // LIDAR: beam grid with sub-sampling (divergence) + return-mode reduction
    float3 fwd=float3(u.camForward), up=float3(u.camUp), leftv=-float3(u.camRight); // sensor +Y = -camRight
    // Beam angles follow OptiX lidar_raygen.cu exactly:
    //     theta = (i / (W-1)) * hFOV - hFOV/2
    //     phi   = (j / (H-1)) * (vMax - vMin) + vMin
    // Endpoint-INCLUSIVE on both axes, so the first and last beams sit on the FOV edges. This is
    // not the pixel-centre convention a camera would use, and the difference is half a beam
    // spacing at each end with opposite sign, which is enough to put a beam on the wrong side of
    // a depth discontinuity. Matching OptiX matters more than being conventional: a lidar's beam
    // table is data users compare across backends.
    float baseAz = float(tid.x)/float(max(1u,u.width-1u)) * u.lidarHFov - u.lidarHFov*0.5;
    float baseEl = float(tid.y)/float(max(1u,u.height-1u)) * (u.lidarVMax-u.lidarVMin) + u.lidarVMin;
    uint rad = min(max(u.lidarSampleRadius,1u),4u); uint n = 2u*rad-1u;
    const uint MAXS = 49u;                       // n <= 7, i.e. sample_radius <= 4
    float sR[49], sI[49]; uint ns=0u;
    float firstR=1e9, firstI=0.0, lastR=0.0, sumR=0.0, sumI=0.0; uint hits=0u;
    for(uint sj=0;sj<n;sj++) for(uint si=0;si<n;si++){
      // Bin-centred across the full divergence, as lidar_raygen.cu does: frac = ((s+0.5)/n)*2-1,
      // theta = frac * div/2. An endpoint-inclusive spread puts the outermost samples on the beam
      // edge instead of inside it, which widens the footprint by a further n/(n-1).
      float2 frac = float2((float(si)+0.5)/float(n)*2.0-1.0, (float(sj)+0.5)/float(n)*2.0-1.0);
      float oaz, oel;
      // The two branches in lidar_raygen.cu are swapped relative to their own comments: the
      // ELLIPTICAL case is the plain rectangular grid, and the RECTANGULAR case is the ellipse-radius
      // math -- which additionally swaps the horizontal/vertical axes and swaps sin/cos, so a
      // horizontal grid step produces a vertical angular offset. Reproduced as written, not as
      // commented, because parity follows the code.
      if(u.lidarBeamShape==1u){
        oaz = frac.x*u.lidarHDiv*0.5;
        oel = frac.y*u.lidarVDiv*0.5;
      } else {
        float angle = atan2(frac.y, frac.x);
        float ring  = max(abs(frac.x), abs(frac.y));
        float2 axis = float2(u.lidarVDiv*0.5*ring, u.lidarHDiv*0.5*ring);
        float radius = 0.0;
        if(!(axis.x==0.0 && axis.y==0.0)){
          float sa=sin(angle), ca=cos(angle);
          radius = (axis.x*axis.y)/sqrt(axis.x*axis.x*sa*sa + axis.y*axis.y*ca*ca);
        }
        oaz = radius*sin(angle);
        oel = radius*cos(angle);
      }
      float az=baseAz+oaz, el=baseEl+oel;
      float3 dir=normalize(cos(el)*(cos(az)*fwd + sin(az)*leftv) + sin(el)*up);
      ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=max(1e-3,u.clipNear); r.max_distance=(u.maxDist>0.0)?u.maxDist:INFINITY;
      Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
          gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
      if(!h.sky){ hits++;
        float inten=abs(dot(h.n,-dir));                                          // OptiX lidar: lidar_intensity(=1) * |N.V|
        sumR+=h.dist; sumI+=inten; lastR=max(lastR,h.dist);
        if(h.dist<firstR){ firstR=h.dist; firstI=inten; }
        if(ns<MAXS){ sR[ns]=h.dist; sI[ns]=inten; ns++; } }
    }
    // STRONGEST_RETURN is not an intensity argmax. lidar_reduce.cu's strong_reduce_kernel scores each
    // sample by how many OTHER samples agree with it in RANGE within 5 cm, weighted by their
    // intensity, so on a smooth surface the winner is a central sample rather than the most face-on
    // one, which is always a footprint edge.
    float strongR=0.0, strongI=0.0, bestScore=-1.0;
    for(uint a=0;a<ns;a++){
      float sc=sI[a];
      for(uint b=0;b<ns;b++){
        if(b==a) continue;
        float dr=abs(sR[b]-sR[a]);
        if(dr<0.05) sc += ((0.05-dr)/0.05)*sI[b];
      }
      sc /= float(n*n);
      if(sc>bestScore){ bestScore=sc; strongR=sR[a]; strongI=sc; }
    }
    if(u.lidarReturnMode==4u){ // DUAL_RETURN: first + strongest, packed (firstR,firstI,strongR,strongI)
      outTex.write(hits>0u?float4(firstR,firstI,strongR,strongI):float4(0,0,0,0), tid); return;
    }
    float outR=0.0, outI=0.0;
    if(hits>0u){ uint rm=u.lidarReturnMode; float mI=sumI/float(n*n);   // lidar_reduce.cu divides by d*d, not by hits
      if(rm==2u){ outR=firstR; outI=mI; }        // FIRST_RETURN
      else if(rm==3u){ outR=lastR; outI=mI; }    // LAST_RETURN
      else if(rm==1u){ outR=sumR/float(hits); outI=mI; } // MEAN_RETURN
      else { outR=strongR; outI=strongI; }       // STRONGEST_RETURN
    }
    outTex.write(float4(outR,outI,0,1), tid); return;
  }

  if(u.mode==5u){ // RADAR: beam grid -> range, amplitude, hit instance index (Doppler resolved on host)
    // The direction a radar beam is TRACED along, which upstream is not the same as the angle it
    // REPORTS. OptiX radar_raygen.cu builds the ray from a pixel-centre grid,
    //     d     = ((i,j) + 0.5) / (W,H) * 2 - 1        (radar_raygen.cu:43)
    //     theta = d.x * hFOV/2                          == (i+0.5)/W * hFOV - hFOV/2
    //     phi   = -vFOV/2 + (d.y*0.5 + 0.5) * vFOV      == (j+0.5)/H * vFOV - vFOV/2
    // and then, thirty lines later, writes DIFFERENT angles into the output buffer, without the
    // half-pixel offset (radar_raygen.cu:83-84). So an OptiX radar return is a range measured
    // along one beam, labelled with the angle of a beam half a step away. That is a bug, but it
    // is the behaviour every existing consumer of Chrono radar data has been reading, so this
    // backend reproduces both halves of it rather than quietly correcting one of them. The
    // reported angles are filled in on the host; see ChFilterMetalRTRender.mm.
    //
    // Note also that lidar does not do this: lidar_raygen.cu traces endpoint-inclusive beams and
    // reports no angles at all. The two sensors genuinely differ upstream.
    float az = (float(tid.x)+0.5)/float(u.width) * u.lidarHFov - u.lidarHFov*0.5;
    float el = (float(tid.y)+0.5)/float(u.height) * (u.lidarVMax-u.lidarVMin) + u.lidarVMin;
    float3 fwd=float3(u.camForward), up=float3(u.camUp), leftv=-float3(u.camRight);
    float3 dir=normalize(cos(el)*(cos(az)*fwd + sin(az)*leftv) + sin(el)*up);
    ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=max(1e-3,u.clipNear); r.max_distance=(u.maxDist>0.0)?u.maxDist:INFINITY;
    Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
        gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
    if(h.sky){ outTex.write(float4(0,0,-1,0), tid); return; }
    float amp=abs(dot(h.n,-dir));   // OptiX radar: radar_backscatter(=1) * |N.V|
    outTex.write(float4(h.dist, amp, float(h.inst), 1.0), tid); return;  // b = hit instance index
  }

  // LEGACY + GI (OptiX camera_legacy_shader.cuh). use_gi does NOT switch OptiX to path tracing: it
  // REPLACES the ambient term with a single cosine-sampled indirect bounce --
  //     color = mirror + direct + refracted;  color += use_gi ? gi_reflection : ambient;
  // -- and recurses with the contribution carried DOWNWARD via prd_reflection.contrib_to_pixel, so each
  // level's own direct lighting is scaled by the product of the BRDF weights above it. The recursion is
  // unrolled into a loop here.
  if(u.useGi!=0u){
    uint spp=max(u.aa*u.aa,1u); uint seed=((tid.y*u.width+tid.x)*9781u+1u)^cameraRngKey(u);
    float3 acc=float3(0.0);
    for(uint s=0;s<spp;s++){
      float jx=rndf(seed), jy=rndf(seed);
      float nx=(2.0*(float(tid.x)+jx)/float(u.width)-1.0)*aspect, ny=(2.0*(float(tid.y)+jy)/float(u.height)-1.0);
      ray r; r.origin=float3(u.camPos); r.direction=camRayDir(nx,ny,u); r.min_distance=1e-3; r.max_distance=INFINITY;
      float3 contrib=float3(1.0), color=float3(0.0);
      for(int d=0; d<5; d++){
        Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
            gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
        if(h.sky){ color += contrib*h.albedo; break; }
        float3 view=-r.direction;
        color += contrib * (directLighting(h,view,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp,seed) + h.emissive*abs(dot(h.n,view)));
        // The GI bounce, standing in for ambient: cosine-sampled direction, weighted by the same
        // Cook-Torrance + Lambert split the direct loop uses (CalculateGIReflectionColor).
        float3 nd=cosineHemisphere(h.n, rndf(seed), rndf(seed));
        float NdL=dot(h.n,nd); if(NdL<=0.0) break;
        float3 hv=normalize(nd+view);
        float NdH=max(dot(h.n,hv),0.0), VdH=max(dot(view,hv),0.0), NdV=max(dot(h.n,view),0.0);
        float mt=clamp(h.metallic,0.0,1.0); bool sw=h.useSpec>0.5;
        float3 F0 = sw ? (h.ks*0.08) : (mt*h.albedo + (1.0-mt)*float3(0.04));
        float3 dA = sw ? h.albedo : (h.albedo*(1.0-mt));
        float3 F  = sw ? (F0 + (float3(1.0)-F0)*pow(1.0-VdH,5.0)) : F0;
        float3 fct = F * NormalDist(NdH,h.rough) * HammonSmith(NdV,NdL,h.rough);
        // weight = transparency * contrib_to_pixel * mat_blend_weight (blend weight 1 for one material),
        // halved when the mirror term is negligible. OptiX tests mirror_reflection_color < 1e-6 on all three
        // channels -- note that is the opposite of what its comment says, and the code is what parity follows.
        // This branch traces no mirror ray, so that test holds trivially here.
        contrib *= 0.5 * h.opacity * (fct*NdL + (float3(1.0)-F)*dA*NdL);
        // OptiX gates on luminance(), not the max component (device_utils.cuh). Since max >= luminance,
        // a max-component test keeps recursing where OptiX stops, which biases the result brighter.
        if(dot(contrib, float3(0.30,0.59,0.11)) <= 0.01) break;   // params.importance_cutoff
        float eps=max(5e-3,length(h.pos)*3e-5);
        r.origin=h.pos+h.n*eps; r.direction=nd; r.min_distance=1e-3; r.max_distance=INFINITY;
      }
      acc+=color;
    }
    outTex.write(float4(cameraPost(acc/float(spp),tid,u,seed),1.0),tid); return;
  }

  // GI (mode 0 with use_gi): iterative path tracer -- NEE direct lighting per bounce + cosine-sampled
  // diffuse indirect bounces, Russian roulette, sky/env as the light. Ports camera_path_shader.cuh
  // (recursion unrolled to a loop). aa^2 = samples/pixel; opt-in (noisy at low spp, needs high spp or
  // a denoiser for a clean image), so the fast legacy path stays the default.
  if(u.integratorPath!=0u){
    uint spp=max(u.aa*u.aa,1u); uint seed=((tid.y*u.width+tid.x)*9781u+1u)^cameraRngKey(u); float3 acc=float3(0.0);
    float physDist=0.0;   // primary-hit distance of the last sample (mode 6 alpha channel)
    for(uint s=0;s<spp;s++){
      float jx=rndf(seed), jy=rndf(seed);
      // row 0 = BOTTOM, matching OptiX camera_raygen.cu (uv = (idx+jitter)/size*2-1)
      float nx=(2.0*(float(tid.x)+jx)/float(u.width)-1.0)*aspect, ny=(2.0*(float(tid.y)+jy)/float(u.height)-1.0);
      ray r; r.origin=float3(u.camPos); r.direction=camRayDir(nx,ny,u); r.min_distance=1e-3; r.max_distance=INFINITY;
      float3 thru=float3(1.0), rad=float3(0.0);
      for(int bnc=0;bnc<5;bnc++){
        Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
            gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
        if(bnc==0) physDist = h.sky?0.0:h.dist;                                    // primary-surface distance (mode 6)
        if(h.sky){ rad += thru*h.albedo; break; }                                  // sky/env is the light source
        float3 view=-r.direction;
        rad += thru * h.emissive * abs(dot(h.n,view));                             // emissive
        rad += thru * directLighting(h,view,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp,seed);  // NEE direct lighting
        float3 nd=cosineHemisphere(h.n, rndf(seed), rndf(seed));                   // indirect diffuse bounce
        thru *= h.albedo * (1.0-clamp(h.metallic,0.0,1.0));                        // cosine estimator -> *albedo
        if(bnc>=2){ float p=clamp(max(max(thru.x,thru.y),thru.z),0.05,0.95); if(rndf(seed)>p) break; thru/=p; }  // Russian roulette
        float eps=max(5e-3,length(h.pos)*3e-5); r.origin=h.pos+h.n*eps; r.direction=nd; r.min_distance=1e-3; r.max_distance=INFINITY;
      }
      acc+=rad;   // (GI noise is handled by Russian roulette + the optional denoiser, like OptiX)
    }
    if(u.mode==6u){ outTex.write(float4(physCamPost(acc/float(spp),u),physDist),tid); return; }
    outTex.write(float4(cameraPost(acc/float(spp),tid,u,seed),1.0),tid); return;
  }

  // COLOR (mode 0) / PHYS CAMERA RGBD (mode 6): antialiased, lit, alpha-composited through transparent layers.
  // Mode 6 differs only in the final write: linear radiance with gamma only, and the primary-surface distance
  // carried in alpha for the downstream ChFilterPhysCameraDefocusBlur stage.
  uint aa=max(u.aa,1u); float3 acc=float3(0.0); uint pseed=((tid.y*u.width+tid.x)*40503u+13u)^cameraRngKey(u);
  float physDist=0.0;
  for(uint sy=0;sy<aa;sy++) for(uint sx=0;sx<aa;sx++){
    float ox=(float(sx)+0.5)/float(aa), oy=(float(sy)+0.5)/float(aa);
    float nx=(2.0*(float(tid.x)+ox)/float(u.width)-1.0)*aspect;
    float ny=(2.0*(float(tid.y)+oy)/float(u.height)-1.0);  // row 0 = BOTTOM (OptiX camera_raygen.cu)
    float3 dir=camRayDir(nx,ny,u);
    ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=1e-3; r.max_distance=INFINITY;
    if(u.apertureR>0.0 && u.mode!=6u){   // thin-lens depth of field: jitter the ray origin on the aperture, aim at the focal plane
      float3 fp=r.origin + dir*u.focalDist;
      float a1=rndf(pseed), a2=rndf(pseed), rad=u.apertureR*sqrt(a1), ang=6.28318530718*a2;
      r.origin += float3(u.camRight)*(rad*cos(ang)) + float3(u.camUp)*(rad*sin(ang));
      r.direction=normalize(fp - r.origin);
    }
    float3 color=float3(0.0); float trans=1.0; float primDist=-1.0;
    for(int layer=0; layer<int(max(u.hitLimit,1u)) && trans>0.02; layer++){
      Hit h=trace(r, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId, gSpecular,
          gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
      if(primDist<0.0 && !h.sky) primDist=h.dist;   // first surface distance (for fog)
      if(h.sky){ color += trans*h.albedo; trans=0.0; break; }
      float3 shaded = lighting(h,-r.direction,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp,pseed);
      shaded += h.emissive * abs(dot(h.n,-r.direction));   // emissive (OptiX: emissive_power*Ke*abs(NdV))
      if(h.opacity < 0.999){
        // Colored/semi-transparent surface (illum 9), exactly like OptiX legacy: the shaded surface color is
        // weighted by opacity and (1-opacity) passes straight through with no bend (CalculateRefractedColor).
        color += trans*h.opacity*shaded;      // opacity-weighted surface color (keeps tint)
        trans *= (1.0-h.opacity);             // remaining clear transmission
        r.origin = h.pos + r.direction*max(5e-3,length(h.pos)*3e-5);  // continue straight through (coord-scaled)
        continue;
      }
      // Mirror reflection -- ported verbatim from OptiX CameraLegacyShader::CalculateContributionToPixel.
      // The reflected color is weighted by the full Cook-Torrance BRDF (F*D*G*NdL/4pi) times the mirror
      // correction (1-rough)^2*metallic^2, CLAMPED to [0,1], and ADDED on top of the surface shading (not
      // lerped in). This BRDF weighting + clamp is exactly what keeps reflected high-frequency foliage from
      // aliasing into "crackle" speckle the way a raw mix() of the full-brightness reflection does.
      {
        float rr_rough = h.rough;              // raw, as OptiX passes mat.roughness
        float rr_metal = clamp(h.metallic, 0.0, 1.0);
        bool  rr_spec  = h.useSpec > 0.5;
        float3 rd = reflect(r.direction, h.n);
        float3 vv = -r.direction;
        float NdV = dot(h.n, vv);
        float NdL = dot(h.n, rd);
        float3 hw = normalize(rd + vv);          // halfway = normalize(next_dir - ray_dir)
        float NdH = dot(h.n, hw);
        float VdH = dot(vv, hw);
        float3 F;
        if(rr_spec){ float3 F0=h.ks*0.08; F = clamp(F0 + (float3(1.0)-F0)*pow(max(0.0,1.0-VdH),5.0), F0, float3(1.0)); }
        else       { F = rr_metal*h.albedo + (1.0-rr_metal)*float3(0.04); }
        float3 f_ct = F * NormalDist(NdH,rr_rough) * HammonSmith(NdV,NdL,rr_rough);
        float mirrorCorr = (1.0-h.rough)*(1.0-h.rough) * h.metallic*h.metallic;
        float3 w = clamp(mirrorCorr * f_ct * NdL / (4.0*3.14159265), 0.0, 1.0);
        if(dot(w, float3(0.30,0.59,0.11)) > 0.01){   // OptiX importance_cutoff
          // Single sharp mirror ray, exactly like OptiX legacy CalculateContributionToPixel. NB: on curved
          // low-roughness panels this reflects the car's own silhouette against the sky as a hard boundary
          // ("wavy" self-reflection); OptiX legacy has the same behaviour. (A glossy/blurred variant fixes
          // the look but costs many extra rays.)
          ray rr; rr.origin=h.pos+h.n*max(1e-2,length(h.pos)*3e-5); rr.direction=rd; rr.min_distance=1e-3; rr.max_distance=INFINITY;
          Hit hr=trace(rr, accel, gN, gA, tint, iR, nBase, matI, gUV, gTexId, gOpacity, gRough, gMetallic, gRoughTexId, gMetalTexId, gOpacityTexId, gTangent, gNormalTexId,
              gSpecular, gEmissive, gTexScale, gKsTexId, gKeTexId, gBlendKdTexId, gBlendWeightTexId, texs, samp, envTex, u);
          float3 rc=hr.sky?hr.albedo:lighting(hr,-rd,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp,pseed);
          shaded += w * rc;                          // ADDITIVE, BRDF-weighted (OptiX: color = mirror_reflection_color + ...)
        }
      }
      color += trans*shaded; trans=0.0; break;                   // opaque -> stop
    }
    // Whatever transmission is still unresolved when the hit budget runs out is DROPPED, not resolved
    // against the background: OptiX leaves refracted_color at float3(0) once depth+1 == max_depth
    // (camera_legacy_shader.cuh). A sky or opaque hit has already zeroed trans and broken out above, so
    // reaching here with trans > 0 means the budget was exhausted.
    // Exponential fog. A ray that reaches the background is fogged too: miss.cu applies the same blend
    // with optixGetRayTmax(), which is effectively infinite there, so the background resolves to very
    // nearly pure fog colour rather than staying unfogged.
    if(u.fogScatter>0.0){
      float fogDist = (primDist>0.0) ? primDist : 1e16;
      float ba=exp(-u.fogScatter*fogDist);
      color=ba*color+(1.0-ba)*float3(u.fogColor);
    }
    physDist = max(primDist, 0.0);                         // sky -> 0, matching phys_cam_raygen.cu's default prd.distance
    acc+=color; }
  if(u.mode==6u){ outTex.write(float4(physCamPost(acc/float(aa*aa),u),physDist), tid); return; }
  outTex.write(float4(cameraPost(acc/float(aa*aa),tid,u,pseed),1.0), tid);
}

// Portable despeckle/denoise pass (OptiX uses its AI denoiser; this is an edge-preserving spatial filter):
// clamp each pixel to its 3x3 neighbours' range to kill isolated speckle/fireflies, plus a light average
// blend to smooth residual noise. Removes the aa=1 edge shimmer on thin metallic frames + GI fireflies.
kernel void denoiseMain(uint2 tid [[thread_position_in_grid]],
   texture2d<float, access::read> inTex [[texture(0)]], texture2d<float, access::write> dstTex [[texture(1)]],
   constant uint2& dims [[buffer(0)]]) {
  if(tid.x>=dims.x||tid.y>=dims.y) return;
  float4 c4=inTex.read(tid);
  float3 c=c4.rgb, mn=float3(1e9), mx=float3(-1e9), avg=c; int cnt=1;
  for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++){
    if(dx==0&&dy==0) continue;
    int2 p=clamp(int2(int(tid.x)+dx,int(tid.y)+dy), int2(0,0), int2(int(dims.x)-1,int(dims.y)-1));
    float3 s=inTex.read(uint2(p)).rgb; mn=min(mn,s); mx=max(mx,s); avg+=s; cnt++;
  }
  avg/=float(cnt);
  // Adaptive: smooth toward the local mean proportional to how much the neighbourhood varies. Flat regions
  // (small range) are left crisp; noisy/z-fighting bands & 1-spp aliasing (large range) get smoothed out.
  float3 rng=mx-mn; float r=max(rng.x,max(rng.y,rng.z));
  float k=clamp((r-0.05)*3.5, 0.0, 0.9);
  dstTex.write(float4(mix(c, avg, k), c4.a), tid);   // alpha passes through untouched (mode 6 carries depth there)
}
)MSLGEN";

#endif
