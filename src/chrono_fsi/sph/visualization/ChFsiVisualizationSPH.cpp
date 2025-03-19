// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_fsi/sph/visualization/ChFsiVisualizationSPH.h"
#include "chrono_fsi/sph/utils/UtilsTypeConvert.cuh"

namespace chrono {
namespace fsi {
namespace sph {

ChFsiVisualizationSPH::ChFsiVisualizationSPH(ChFsiSystemSPH* sysFSI)
    : m_sysFSI(sysFSI),
      m_sysSPH(&sysFSI->GetFluidSystemSPH()),
      m_user_system(nullptr),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 0.60f, 0.30f)),
      m_flex_bce_color(ChColor(0.40f, 0.10f, 0.65f)),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC();
}

ChFsiVisualizationSPH::ChFsiVisualizationSPH(ChFsiFluidSystemSPH* sysSPH)
    : m_sysFSI(nullptr),
      m_sysSPH(sysSPH),
      m_user_system(nullptr),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 0.60f, 0.30f)),
      m_flex_bce_color(ChColor(0.40f, 0.10f, 0.65f)),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC();
}

ChFsiVisualizationSPH::~ChFsiVisualizationSPH() {
    delete m_sysMBS;
}

void ChFsiVisualizationSPH::SetVerbose(bool verbose) {
    GetVisualSystem()->SetVerbose(verbose);
}

void ChFsiVisualizationSPH::SetSize(int width, int height) {}

void ChFsiVisualizationSPH::SetTitle(const std::string& title) {}

void ChFsiVisualizationSPH::AddCamera(const ChVector3d& pos, const ChVector3d& target) {}

void ChFsiVisualizationSPH::UpdateCamera(const ChVector3d& pos, const ChVector3d& target) {}

void ChFsiVisualizationSPH::SetCameraVertical(CameraVerticalDir up) {}

void ChFsiVisualizationSPH::SetCameraMoveScale(float scale) {}

void ChFsiVisualizationSPH::SetLightIntensity(double intensity) {}

void ChFsiVisualizationSPH::SetLightDirection(double azimuth, double elevation) {}

void ChFsiVisualizationSPH::SetParticleRenderMode(RenderMode mode) {}

void ChFsiVisualizationSPH::SetRenderMode(RenderMode mode) {}

void ChFsiVisualizationSPH::EnableInfoOverlay(bool val) {}

void ChFsiVisualizationSPH::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetFixed(true);
    m_sysMBS->AddBody(body);
}

void ChFsiVisualizationSPH::Initialize() {}

// -----------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin,
                                                         double hmax,
                                                         const ChVector3d& up)
    : m_monochrome(false), m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

ParticleHeightColorCallback::ParticleHeightColorCallback(const ChColor& base_color,
                                                         double hmin,
                                                         double hmax,
                                                         const ChVector3d& up)
    : m_monochrome(true), m_base_color(base_color), m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

ChColor ParticleHeightColorCallback::get(unsigned int n) const {
    double h = dot(pos[n], m_up);  // particle height
    if (m_monochrome) {
        float factor = (float)((h - m_hmin) / (m_hmax - m_hmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(h, m_hmin, m_hmax);
}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(double vmin,
                                                             double vmax,
                                                             Component component)
    : m_monochrome(false), m_vmin(vmin), m_vmax(vmax), m_component(component) {}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(const ChColor& base_color,
                                                             double vmin,
                                                             double vmax,
                                                             Component component)
    : m_monochrome(true), m_base_color(base_color), m_vmin(vmin), m_vmax(vmax), m_component(component) {}

ChColor ParticleVelocityColorCallback::get(unsigned int n) const {
    double v = 0;
    switch (m_component) {
        case Component::NORM:
            v = length(vel[n]);
            break;
        case Component::X:
            v = std::abs(vel[n].x);
            break;
        case Component::Y:
            v = std::abs(vel[n].y);
            break;
        case Component::Z:
            v = std::abs(vel[n].z);
            break;
    }

    if (m_monochrome) {
        float factor = (float)((v - m_vmin) / (m_vmax - m_vmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(v, m_vmin, m_vmax);
}

ParticleDensityColorCallback::ParticleDensityColorCallback(double dmin, double dmax)
    : m_monochrome(false), m_dmin(dmin), m_dmax(dmax) {}

ParticleDensityColorCallback::ParticleDensityColorCallback(const ChColor& base_color, double dmin, double dmax)
    : m_monochrome(true), m_base_color(base_color), m_dmin(dmin), m_dmax(dmax) {}

ChColor ParticleDensityColorCallback::get(unsigned int n) const {
    double d = prop[n].x;

    if (m_monochrome) {
        float factor = (float)((d - m_dmin) / (m_dmax - m_dmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(d, m_dmin, m_dmax);
}

ParticlePressureColorCallback::ParticlePressureColorCallback(double pmin, double pmax)
    : m_monochrome(false), m_bichrome(false), m_pmin(pmin), m_pmax(pmax) {}

ParticlePressureColorCallback::ParticlePressureColorCallback(const ChColor& base_color, double pmin, double pmax)
    : m_monochrome(true), m_bichrome(false), m_base_color(base_color), m_pmin(pmin), m_pmax(pmax) {}

ParticlePressureColorCallback::ParticlePressureColorCallback(const ChColor& base_color_neg,
                                                             const ChColor& base_color_pos,
                                                             double pmin,
                                                             double pmax)
    : m_monochrome(false),
      m_bichrome(true),
      m_base_color_neg(base_color_neg),
      m_base_color_pos(base_color_pos),
      m_pmin(pmin),
      m_pmax(pmax) {
    assert(m_pmin < 0);
}

ChColor ParticlePressureColorCallback::get(unsigned int n) const {
    double p = prop[n].y;

    if (m_monochrome) {
        float factor = (float)((p - m_pmin) / (m_pmax - m_pmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else if (m_bichrome) {
        if (p < 0) {
            float factor = (float)(p / m_pmin);  // color scaling factor (0,1)
            return ChColor(factor * m_base_color_neg.R, factor * m_base_color_neg.G, factor * m_base_color_neg.B);
        } else {
            float factor = (float)(+p / m_pmax);  // color scaling factor (0,1)
            return ChColor(factor * m_base_color_pos.R, factor * m_base_color_pos.G, factor * m_base_color_pos.B);
        }
    } else
        return ChColor::ComputeFalseColor(p, m_pmin, m_pmax);
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
