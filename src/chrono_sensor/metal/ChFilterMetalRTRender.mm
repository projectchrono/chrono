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
// First filter in a Metal-rendered camera's graph. Allocates the RGBA8 host
// buffer and renders the scene with Metal hardware ray tracing into it, using
// the sensor's world pose (Chrono::Sensor camera convention: +X forward, -Y
// right, +Z up, horizontal FOV).
// =============================================================================

#include <cmath>
#include <vector>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/metal/ChFilterMetalRTRender.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

namespace chrono {
namespace sensor {

static inline float clampf(float x) {
    return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}

ChFilterMetalRTRender::ChFilterMetalRTRender(std::shared_ptr<ChMetalRTDevice> device, std::shared_ptr<ChMetalRTScene> scene)
    : ChFilter("MetalRTRenderer"), m_device(std::move(device)), m_scene(std::move(scene)) {}

ChFilterMetalRTRender::~ChFilterMetalRTRender() {}

void ChFilterMetalRTRender::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    auto metal_sensor = std::dynamic_pointer_cast<ChMetalSensor>(pSensor);
    if (!metal_sensor) {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
        return;
    }
    m_sensor = metal_sensor;
    // One RNG stream per rendered buffer, keyed on (manager, sensor, filter, purpose). Without this
    // every Metal camera in a scene draws the identical sample sequence, which correlates their
    // noise, and ChSensorManager::SetRandomSeed has no effect on the render.
    m_rng_seed = ChSensorManager::GetDeterministicSeed(
        pSensor, metal_sensor->GetPipelineType() == MetalPipelineType::PHYS_CAMERA ? RngUsage::MetalPhysCameraRaygen : RngUsage::MetalCameraRaygen, GetRngStreamIndex());
    const unsigned int width = metal_sensor->GetWidth();
    const unsigned int height = metal_sensor->GetHeight();
    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

    switch (metal_sensor->GetPipelineType()) {
        case MetalPipelineType::DEPTH_CAMERA:
            m_buffer_depth = chrono_types::make_shared<SensorHostDepthBuffer>();
            m_buffer_depth->Width = width;
            m_buffer_depth->Height = height;
            m_buffer_depth->Buffer = std::shared_ptr<PixelDepth[]>(new PixelDepth[count]);
            bufferInOut = m_buffer_depth;
            break;
        case MetalPipelineType::NORMAL_CAMERA:
            m_buffer_normal = chrono_types::make_shared<SensorHostNormalBuffer>();
            m_buffer_normal->Width = width;
            m_buffer_normal->Height = height;
            m_buffer_normal->Buffer = std::shared_ptr<PixelNormal[]>(new PixelNormal[count]);
            bufferInOut = m_buffer_normal;
            break;
        case MetalPipelineType::SEGMENTATION:
            m_buffer_semantic = chrono_types::make_shared<SensorHostSemanticBuffer>();
            m_buffer_semantic->Width = width;
            m_buffer_semantic->Height = height;
            m_buffer_semantic->Buffer = std::shared_ptr<PixelSemantic[]>(new PixelSemantic[count]);
            bufferInOut = m_buffer_semantic;
            break;
        case MetalPipelineType::LIDAR_SINGLE:
        case MetalPipelineType::LIDAR_MULTI: {
            bool dual = false;
            if (auto ld = std::dynamic_pointer_cast<ChLidarSensor>(metal_sensor))
                dual = (ld->GetReturnMode() == LidarReturnMode::DUAL_RETURN);
            size_t mult = dual ? 2 : 1;
            m_buffer_di = chrono_types::make_shared<SensorHostDIBuffer>();
            m_buffer_di->Width = width;
            m_buffer_di->Height = height;
            m_buffer_di->Buffer = std::shared_ptr<PixelDI[]>(new PixelDI[count * mult]);
            m_buffer_di->Beam_return_count = (unsigned int)(count * mult);
            m_buffer_di->Dual_return = dual;
            bufferInOut = m_buffer_di;
            break;
        }
        case MetalPipelineType::PHYS_CAMERA:
            // Physics-based camera: the ChFilterPhysCamera* chain consumes an RGB-D half4 buffer
            // (linear radiance + primary-surface distance), exactly like the OptiX rgbd_buffer.
            m_buffer_rgbd = chrono_types::make_shared<SensorHostRGBDHalf4Buffer>();
            m_buffer_rgbd->Width = width;
            m_buffer_rgbd->Height = height;
            m_buffer_rgbd->Buffer = std::shared_ptr<PixelRGBDHalf4[]>(new PixelRGBDHalf4[count]);
            bufferInOut = m_buffer_rgbd;
            break;
        case MetalPipelineType::RADAR:
            m_buffer_radar = chrono_types::make_shared<SensorHostRadarBuffer>();
            m_buffer_radar->Width = width;
            m_buffer_radar->Height = height;
            m_buffer_radar->Buffer = std::shared_ptr<RadarReturn[]>(new RadarReturn[count]);
            m_buffer_radar->Beam_return_count = (unsigned int)count;
            bufferInOut = m_buffer_radar;
            break;
        case MetalPipelineType::CAMERA:
        default:
            m_buffer_rgba8 = chrono_types::make_shared<SensorHostRGBA8Buffer>();
            m_buffer_rgba8->Width = width;
            m_buffer_rgba8->Height = height;
            m_buffer_rgba8->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[count]);
            bufferInOut = m_buffer_rgba8;
            break;
    }
}

void ChFilterMetalRTRender::Apply() {
    auto sensor = m_sensor.lock();
    if (!sensor || !m_scene)
        return;
    const unsigned int width = sensor->GetWidth();
    const unsigned int height = sensor->GetHeight();
    if (width == 0 || height == 0)
        return;
    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

    // pipeline type -> render mode
    int mode = 0;
    switch (sensor->GetPipelineType()) {
        case MetalPipelineType::DEPTH_CAMERA:
            mode = 1;
            break;
        case MetalPipelineType::NORMAL_CAMERA:
            mode = 2;
            break;
        case MetalPipelineType::SEGMENTATION:
            mode = 3;
            break;
        case MetalPipelineType::LIDAR_SINGLE:
        case MetalPipelineType::LIDAR_MULTI:
            mode = 4;
            break;
        case MetalPipelineType::RADAR:
            mode = 5;
            break;
        case MetalPipelineType::PHYS_CAMERA:
            mode = 6;
            break;
        default:
            mode = 0;
            break;
    }

    // Lazily create the Metal renderer against the engine's device.
    if (!m_renderer && m_device && m_device->IsValid())
        m_renderer = chrono_types::make_shared<ChMetalRTRenderer>(m_device->GetMTLDevice(), m_device->GetMTLQueue());

    if (m_renderer && m_renderer->Valid()) {
        const MetalRenderScene& rs = m_scene->GetRenderScene();
        if (!m_renderer_built) {
            m_renderer->Build(rs);
            if (!m_scene->GetBackground().env_tex.empty())  // HDR environment map for sky + reflections
                m_renderer->SetEnvMap(m_scene->GetBackground().env_tex);
            m_renderer_built = true;
        } else {
            m_renderer->UpdateDynamic(rs);
        }

        // Camera world pose (matches ChOptixEngine / ChFilterVulkanRTRender).
        ChFrame<double> cf = sensor->GetOffsetPose();
        if (auto body = sensor->GetParent())
            cf = body->GetVisualModelFrame() * sensor->GetOffsetPose();
        const ChVector3d o = cf.GetPos();
        const ChVector3d fwd = cf.TransformDirectionLocalToParent(ChVector3d(1, 0, 0));
        const ChVector3d rgt = cf.TransformDirectionLocalToParent(ChVector3d(0, -1, 0));
        const ChVector3d up = cf.TransformDirectionLocalToParent(ChVector3d(0, 0, 1));

        // ---- camera-like sensor parameters ----
        // The five rendered camera front-ends do NOT share a common base: ChCameraSensor,
        // ChPhysCameraSensor, ChDepthCamera, ChNormalCamera and ChSegmentationCamera each derive
        // straight from ChMetalSensor. So each type has to be probed on its own -- probing only
        // ChCameraSensor silently left depth/normal/segmentation on the 1.408 rad (80.67 deg)
        // fallback below and stuck them on PINHOLE, which de-registered them from the RGB view.
        float hfov = 1.408f;  // fallback only; every camera type overrides it
        int ss = 1;           // supersampling
        int lens_model = 0;   // 0 pinhole, 1 FOV, 2 radial
        ChVector3f distortion(0.f, 0.f, 0.f);
        float max_depth = 0.f;  // depth camera far clip; 0 = unlimited

        auto phys = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor);
        auto cam_rgb = std::dynamic_pointer_cast<ChCameraSensor>(sensor);
        auto cam_depth = std::dynamic_pointer_cast<ChDepthCamera>(sensor);
        auto cam_normal = std::dynamic_pointer_cast<ChNormalCamera>(sensor);
        auto cam_seg = std::dynamic_pointer_cast<ChSegmentationCamera>(sensor);

        if (cam_rgb) {
            hfov = cam_rgb->GetHFOV();
            ss = (int)cam_rgb->GetSampleFactor();
            lens_model = (int)cam_rgb->GetLensModelType();
            distortion = cam_rgb->GetCameraDistortionCoefficients();
        } else if (phys) {
            hfov = phys->GetHFOV();  // derived from sensor_width / focal_length
            ss = (int)phys->GetSampleFactor();
            lens_model = (int)phys->GetLensModelType();
            distortion = phys->GetCameraDistortionCoefficients();
        } else if (cam_depth) {
            hfov = cam_depth->GetHFOV();
            lens_model = (int)cam_depth->GetLensModelType();
            max_depth = cam_depth->GetMaxDepth();
        } else if (cam_normal) {
            hfov = cam_normal->GetHFOV();
            lens_model = (int)cam_normal->GetLensModelType();
        } else if (cam_seg) {
            hfov = cam_seg->GetHFOV();
            lens_model = (int)cam_seg->GetLensModelType();
        }
        const float aspect = (float)width / (float)height;

        MetalCameraParams cam{};
        cam.origin[0] = (float)o.x();
        cam.origin[1] = (float)o.y();
        cam.origin[2] = (float)o.z();
        cam.forward[0] = (float)fwd.x();
        cam.forward[1] = (float)fwd.y();
        cam.forward[2] = (float)fwd.z();
        cam.right[0] = (float)rgt.x();
        cam.right[1] = (float)rgt.y();
        cam.right[2] = (float)rgt.z();
        cam.up[0] = (float)up.x();
        cam.up[1] = (float)up.y();
        cam.up[2] = (float)up.z();
        ChVector3f amb = m_scene->GetAmbientLight();
        cam.ambient[0] = amb.x();
        cam.ambient[1] = amb.y();
        cam.ambient[2] = amb.z();
        // Background + fog + GI, mirroring the OptiX scene. Map the shared BackgroundMode enum
        // onto the shader's encoding (0 = env map / procedural, 1 = gradient, 2 = solid).
        const Background bg = m_scene->GetBackground();
        switch (bg.mode) {
            case BackgroundMode::ENVIRONMENT_MAP:
                cam.bgMode = 0;
                break;
            case BackgroundMode::GRADIENT:
                cam.bgMode = 1;
                break;
            default:
                cam.bgMode = 2;
                break;  // SOLID_COLOR
        }
        if (bg.mode == BackgroundMode::ENVIRONMENT_MAP && bg.env_tex.empty())
            cam.bgMode = 2;  // env map requested but none supplied -> fall back to solid
        ChVector3f bz = bg.color_zenith, bh = bg.color_horizon, fc = m_scene->GetFogColor();
        cam.bgZenith[0] = bz.x();
        cam.bgZenith[1] = bz.y();
        cam.bgZenith[2] = bz.z();
        cam.bgHorizon[0] = bh.x();
        cam.bgHorizon[1] = bh.y();
        cam.bgHorizon[2] = bh.z();
        cam.fogColor[0] = fc.x();
        cam.fogColor[1] = fc.y();
        cam.fogColor[2] = fc.z();
        cam.fogScatter = 0.f;
        cam.useGi = 0;
        cam.integratorPath = 0;
        // ChSensorManager::SetRayRecursions reaches us as m_ray_recursions but was previously unread, so the
        // setting did nothing on Metal. OptiX spends two levels of its budget before the first surface.
        cam.hitLimit = m_ray_recursions > 2 ? m_ray_recursions - 2 : 1;
        cam.envIntensity = m_scene->GetEnvIntensity();  // env-map radiance scale (OptiX AddEnvironmentLight)
        if (auto cc = std::dynamic_pointer_cast<ChCameraSensor>(sensor)) {
            if (cc->GetUseFog())
                cam.fogScatter = m_scene->GetFogScattering();
            cam.useGi = cc->GetUseGI() ? 1 : 0;
            // OptiX selects the shader from the integrator, not from use_gi (camera_shader.cuh), so a
            // LEGACY camera with GI enabled stays in the legacy shader with GI replacing ambient.
            cam.integratorPath = (cc->GetIntegrator() == Integrator::PATH) ? 1 : 0;
            cam.useDenoiser = cc->GetUseDenoiser();
            cam.gamma = cc->GetGamma();  // OptiX-configurable output gamma
        }
        if (phys) {
            // The physics-based camera models fog/GI/denoiser/gamma itself; exposure, vignetting, aperture and
            // sensor noise are handled by the downstream ChFilterPhysCamera* stages, so the scene-level knobs
            // stay neutral here (mode 6 ignores them in the shader as well).
            if (phys->GetUseFog())
                cam.fogScatter = m_scene->GetFogScattering();
            cam.useGi = phys->GetUseGI() ? 1 : 0;
            cam.integratorPath = (phys->GetIntegrator() == Integrator::PATH) ? 1 : 0;
            cam.useDenoiser = phys->GetUseDenoiser();
            cam.gamma = phys->GetGamma();
        }
        cam.exposure = m_scene->GetExposure();
        cam.vignette = m_scene->GetVignette();
        cam.noiseSigma = m_scene->GetSensorNoise();
        cam.apertureR = m_scene->GetApertureRadius();
        cam.focalDist = m_scene->GetFocalDist();
        cam.tanHalfV = std::tan(0.5f * hfov) / aspect;
        cam.aa = (mode == 0 || mode == 6) ? (ss < 1 ? 1 : (ss > 4 ? 4 : ss)) : 1;  // AA only for color (can't average ids/depth)
        cam.mode = mode;
        cam.rngSeedLo = (unsigned int)(m_rng_seed & 0xffffffffull);
        cam.rngSeedHi = (unsigned int)(m_rng_seed >> 32);
        cam.lensModel = lens_model;  // pinhole / FOV / radial
        cam.dk1 = distortion.x();
        cam.dk2 = distortion.y();
        cam.dk3 = distortion.z();
        if (mode != 4 && mode != 5)
            cam.lidarHFov = hfov;  // the FOV (fisheye) lens model reads the horizontal FOV from here
        if (mode == 1)
            cam.maxDist = max_depth;  // ChDepthCamera far clip (OptiX/Vulkan honour GetMaxDepth)
        if (mode == 4) {
            if (auto ld = std::dynamic_pointer_cast<ChLidarSensor>(sensor)) {
                cam.lidarHFov = ld->GetHFOV();
                cam.lidarVMin = ld->GetMinVertAngle();
                cam.lidarVMax = ld->GetMaxVertAngle();
                cam.maxDist = ld->GetMaxDistance();
                cam.lidarSampleRadius = (int)ld->GetSampleRadius();
                // The divergence is the FULL beam width; lidar_raygen.cu spreads its samples across it
                // bin-centred. Scaling it by the sample count here made the footprint 2*radius too wide.
                cam.lidarHDiv = ld->GetHorizDivAngle();
                cam.lidarVDiv = ld->GetVertDivAngle();
                cam.lidarBeamShape = (int)ld->GetBeamShape();
                cam.lidarReturnMode = (int)ld->GetReturnMode();
                cam.clipNear = ld->GetClipNear();  // sensor sits inside its own housing on some vehicles
            }
        }
        if (mode == 5) {
            if (auto rd = std::dynamic_pointer_cast<ChRadarSensor>(sensor)) {
                cam.lidarHFov = rd->GetHFOV();
                cam.lidarVMin = -0.5f * rd->GetVFOV();
                cam.lidarVMax = 0.5f * rd->GetVFOV();
                cam.maxDist = rd->GetMaxDistance();
                cam.clipNear = rd->GetClipNear();
            }
        }

        // Lights from the scene; if the user set none, synthesize a key + directional fill.
        // atten_scale follows ChOptixScene/ChOptixLightStructs exactly: max_range > 0 gives the
        // "1% of peak at max_range" scale, max_range <= 0 (the -1 sentinel) gives plain
        // inverse-square. const_color suppresses the distance falloff entirely, so it has to
        // reach the GPU -- without it a max_range=500 light is amplified ~100x at close range.
        auto atten_scale = [](float range) { return range > 0.f ? 0.01f * range * range : 1.f; };
        std::vector<MetalLightGPU> lights;
        const auto& sl = m_scene->GetLights();
        if (!sl.empty()) {
            for (const auto& L : sl)
                lights.push_back({{(float)L.pos.x(), (float)L.pos.y(), (float)L.pos.z()},
                                  L.range,
                                  {L.color.R, L.color.G, L.color.B},
                                  (float)L.type,
                                  {(float)L.dir.x(), (float)L.dir.y(), (float)L.dir.z()},
                                  L.cosOuter,
                                  L.cosInner,
                                  L.p0,
                                  atten_scale(L.range),
                                  L.const_color ? 1.f : 0.f});
        } else {
            ChVector3d key = o + fwd * 4.0 + rgt * 4.0 + ChVector3d(0, 0, 12.0);
            lights.push_back({{(float)key.x(), (float)key.y(), (float)key.z()}, 0.f, {1.05f, 1.02f, 0.98f}, 0.f, {0.f, 0.f, 0.f}, 0.f, 0.f, 0.f, 1.f, /*constColor=*/1.f});
            lights.push_back({{0.35f, 0.25f, -1.0f}, 0.f, {0.22f, 0.24f, 0.30f}, 1.f, {0.f, 0.f, 0.f}, 0.f, 0.f, 0.f, 1.f, /*constColor=*/1.f});
        }

        std::vector<float> scratch(count * 4);
        m_renderer->Render(cam, lights.data(), (int)lights.size(), (int)width, (int)height, scratch.data());

        // convert the RGBA32F output into the sensor's host buffer
        if (mode == 0 && m_buffer_rgba8 && m_buffer_rgba8->Buffer) {
            PixelRGBA8* px = m_buffer_rgba8->Buffer.get();
            for (size_t i = 0; i < count; ++i) {
                px[i].R = (uint8_t)(clampf(scratch[i * 4 + 0]) * 255.f);
                px[i].G = (uint8_t)(clampf(scratch[i * 4 + 1]) * 255.f);
                px[i].B = (uint8_t)(clampf(scratch[i * 4 + 2]) * 255.f);
                px[i].A = 255;
            }
        } else if (mode == 6 && m_buffer_rgbd && m_buffer_rgbd->Buffer) {
            // linear radiance (gamma-corrected in the shader, exactly like phys_cam_raygen.cu) + distance
            PixelRGBDHalf4* px = m_buffer_rgbd->Buffer.get();
            for (size_t i = 0; i < count; ++i) {
                px[i].R = scratch[i * 4 + 0];
                px[i].G = scratch[i * 4 + 1];
                px[i].B = scratch[i * 4 + 2];
                px[i].D = scratch[i * 4 + 3];
            }
        } else if (mode == 1 && m_buffer_depth && m_buffer_depth->Buffer) {
            PixelDepth* px = m_buffer_depth->Buffer.get();
            for (size_t i = 0; i < count; ++i)
                px[i].depth = scratch[i * 4 + 0];
        } else if (mode == 2 && m_buffer_normal && m_buffer_normal->Buffer) {
            PixelNormal* px = m_buffer_normal->Buffer.get();
            for (size_t i = 0; i < count; ++i) {
                px[i].normal_x = scratch[i * 4 + 0];
                px[i].normal_y = scratch[i * 4 + 1];
                px[i].normal_z = scratch[i * 4 + 2];
            }
        } else if (mode == 3 && m_buffer_semantic && m_buffer_semantic->Buffer) {
            PixelSemantic* px = m_buffer_semantic->Buffer.get();
            for (size_t i = 0; i < count; ++i) {
                px[i].class_id = (unsigned short)(scratch[i * 4 + 0] + 0.5f);
                px[i].instance_id = (unsigned short)(scratch[i * 4 + 1] + 0.5f);
            }
        } else if (mode == 4 && m_buffer_di && m_buffer_di->Buffer) {
            PixelDI* px = m_buffer_di->Buffer.get();
            if (m_buffer_di->Dual_return) {  // plane 0 = first return, plane 1 = strongest return
                for (size_t i = 0; i < count; ++i) {
                    px[i].range = scratch[i * 4 + 0];
                    px[i].intensity = scratch[i * 4 + 1];
                    px[count + i].range = scratch[i * 4 + 2];
                    px[count + i].intensity = scratch[i * 4 + 3];
                }
            } else {
                for (size_t i = 0; i < count; ++i) {
                    px[i].range = scratch[i * 4 + 0];
                    px[i].intensity = scratch[i * 4 + 1];
                }
            }
        } else if (mode == 5 && m_buffer_radar && m_buffer_radar->Buffer) {
            RadarReturn* px = m_buffer_radar->Buffer.get();
            const auto& inst = rs.instances;
            const float hfov = cam.lidarHFov, vmin = cam.lidarVMin, vmax = cam.lidarVMax;
            // Doppler is relative to the sensor's own motion: subtract the mount-body velocity (matches OptiX).
            float vsx = 0.f, vsy = 0.f, vsz = 0.f;
            if (auto pb = sensor->GetParent()) {
                auto vs = pb->GetPosDt();
                vsx = (float)vs.x();
                vsy = (float)vs.y();
                vsz = (float)vs.z();
            }
            // The REPORTED angles, which upstream deliberately are not the angles the beam was
            // traced along: OptiX writes (i/W)*hFOV - hFOV/2 here (radar_raygen.cu:83-84) while
            // tracing the pixel-centre direction half a step away. Reproduced as-is, because it
            // is what every existing reader of Chrono radar data receives. See the note in the
            // shader's mode 5 for the full comparison.
            for (unsigned int j = 0; j < height; ++j) {
                float el = (float)j / (float)height * (vmax - vmin) + vmin;
                for (unsigned int i = 0; i < width; ++i) {
                    size_t k = (size_t)j * width + i;
                    RadarReturn r{};
                    r.range = scratch[k * 4 + 0];
                    r.amplitude = scratch[k * 4 + 1];
                    r.azimuth = (float)i / (float)width * hfov - hfov * 0.5f;
                    r.elevation = el;
                    int objIdx = (int)(scratch[k * 4 + 2] + 0.5f);
                    float hit = scratch[k * 4 + 3];
                    r.objectId = (hit > 0.5f) ? (float)objIdx : -1.f;
                    if (hit > 0.5f && objIdx >= 0 && objIdx < (int)inst.size()) {
                        r.doppler_velocity[0] = inst[objIdx].vel[0] - vsx;
                        r.doppler_velocity[1] = inst[objIdx].vel[1] - vsy;
                        r.doppler_velocity[2] = inst[objIdx].vel[2] - vsz;
                    }
                    px[k] = r;
                }
            }
        }
    } else if (mode == 0 && m_buffer_rgba8 && m_buffer_rgba8->Buffer) {
        PixelRGBA8* px = m_buffer_rgba8->Buffer.get();
        for (size_t i = 0; i < count; ++i)
            px[i] = {60, 60, 66, 255};
    }

    auto stamp = [&](std::shared_ptr<SensorBuffer> b) {
        if (b) {
            b->TimeStamp = m_time_stamp;
            b->LaunchedCount = sensor->GetNumLaunches();
        }
    };
    stamp(m_buffer_rgba8);
    stamp(m_buffer_depth);
    stamp(m_buffer_normal);
    stamp(m_buffer_semantic);
    stamp(m_buffer_di);
    stamp(m_buffer_radar);
    stamp(m_buffer_rgbd);
}

}  // namespace sensor
}  // namespace chrono
