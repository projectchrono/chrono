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
// Authors: Huzaifa Mustafa Unjhawala
// =============================================================================
//
// Verify CRM shear stress and displacement in a confined elastic layer.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#ifdef CHRONO_VSG
    #include "chrono/assets/ChVisualShapeBox.h"
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

namespace {

constexpr double height = 0.04;
constexpr double width = 0.04;
constexpr double displacement = 0.00024;
constexpr double pressure = 5000;
constexpr double young = 1e6;
constexpr double poisson = 0.3;
constexpr double bulk_mu = 0.7;

double HorizontalDisplacement(double x, double x0) {
    double u = x - x0;
    return u - width * std::round(u / width);
}

struct Observation {
    double shear = 0;
    double normal = 0;
    double bottom = 0;
    double bulk = 0;
    double yield_ratio = 0;
};

Observation Observe(const ChFsiFluidSystemSPH& soil, const std::vector<ChVector3d>& initial) {
    auto positions = soil.GetParticlePositions(MarkerGroup::FLUID);
    auto velocities = soil.GetParticleVelocities(MarkerGroup::FLUID);
    auto diagonal = soil.GetParticleStressDiagonal(MarkerGroup::FLUID);
    auto off_diagonal = soil.GetParticleStressOffDiagonal(MarkerGroup::FLUID);
    Observation result;
    double n = 0, sum_z = 0, sum_u = 0, sum_zz = 0, sum_zu = 0;
    for (size_t i = 0; i < initial.size(); ++i) {
        if (!std::isfinite(positions[i].Length2()) || !std::isfinite(velocities[i].Length2()) || !std::isfinite(diagonal[i].Length2()) || !std::isfinite(off_diagonal[i].Length2()))
            throw std::runtime_error("Nonfinite CRM state");

        double z = initial[i].z();
        if (z <= 0.25 * height || z >= 0.75 * height)
            continue;
        double u = HorizontalDisplacement(positions[i].x(), initial[i].x());
        ++n;
        sum_u += u;
        sum_z += z;
        sum_zu += z * u;
        sum_zz += z * z;
        result.shear += off_diagonal[i].y();
        result.normal -= diagonal[i].z();
        double p = -(diagonal[i].x() + diagonal[i].y() + diagonal[i].z()) / 3;
        if (p <= 0)
            throw std::runtime_error("Lost confinement in the central soil layer");
        auto deviator = diagonal[i] + ChVector3d(p);
        result.yield_ratio += std::sqrt(0.5 * deviator.Length2() + off_diagonal[i].Length2()) / p;
    }
    // Fit the central displacement profile u(z) = bottom + (bulk / height) * z.
    double slope = (n * sum_zu - sum_z * sum_u) / (n * sum_zz - sum_z * sum_z);
    result.bottom = (sum_u - slope * sum_z) / n;
    result.bulk = slope * height;
    result.shear /= n;
    result.normal /= n;
    result.yield_ratio /= n;
    return result;
}

#ifdef CHRONO_VSG
class DisplacementColor : public ChSphVisualizationVSG::ParticleColorCallback {
  public:
    explicit DisplacementColor(const std::vector<ChVector3d>& initial) : m_initial(initial) {}
    std::string GetTile() const override { return "Horizontal displacement u/U"; }
    ChVector2d GetDataRange() const override { return {0, 1}; }
    ChColor GetColor(unsigned int n) const override { return m_vsys->GetColormap().Get(HorizontalDisplacement(pos[n].x, m_initial[n].x()) / displacement, 0, 1); }

  private:
    const std::vector<ChVector3d>& m_initial;
};
#endif

bool VerifyShear(int layers, [[maybe_unused]] bool render) {
    // Scale the time step with the particle spacing (4e-5 at the default 12 layers) to keep the Courant number fixed.
    const double step = 4e-5 * 12 / layers;
    constexpr double duration = 0.18;
    const int steps = static_cast<int>(std::round(duration / step));
    double spacing = height / layers;
    int lateral = static_cast<int>(std::round(width / spacing));
    if (std::abs(lateral * spacing - width) > 1e-6 * width)
        throw std::invalid_argument("The layer width must be a whole number of particle spacings");
    ChSystemSMC solid;
    solid.SetGravitationalAcceleration(VNULL);
    ChFsiFluidSystemSPH soil;
    ChFsiSystemSPH system(&solid, &soil);
    system.SetVerbose(false);
    system.SetCouplingScheme(ChFsiSystem::CouplingScheme::SEQUENTIAL);
    soil.SetGravitationalAcceleration(VNULL);
    ChFsiFluidSystemSPH::SoilProperties material;
    material.density = 1700;
    material.Young_modulus = young;
    material.Poisson_ratio = poisson;
    material.mu_fric_s = bulk_mu;
    material.mu_fric_2 = bulk_mu;
    material.cohesion_coeff = 0;
    soil.SetCrmSPH(material);
    ChFsiFluidSystemSPH::SPHParameters sph;
    sph.initial_spacing = spacing;
    sph.kernel_type = KernelType::WENDLAND;
    sph.d0_multiplier = 1.5;
    sph.integration_scheme = IntegrationScheme::SYMPLECTIC;
    sph.boundary_method = BoundaryMethod::ADAMI;
    sph.shifting_method = ShiftingMethod::NONE;
    sph.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph.artificial_viscosity = 0.2;
    sph.use_variable_time_step = false;
    sph.num_bce_layers = 3;
    sph.num_proximity_search_steps = 1;
    soil.SetSPHParameters(sph);
    std::vector<ChVector3d> initial, bottom, top;
    for (int x = 0; x < lateral; ++x) {
        for (int y = 0; y < lateral; ++y) {
            for (int z = 0; z < layers; ++z) {
                ChVector3d position((x + 0.5) * spacing, (y + 0.5) * spacing, (z + 0.5) * spacing);
                initial.push_back(position);
                soil.AddSPHParticle(position, VNULL, ChVector3d(-pressure), VNULL);
            }
            for (int z = 0; z < sph.num_bce_layers; ++z) {
                bottom.emplace_back((x + 0.5) * spacing, (y + 0.5) * spacing, -(z + 0.5) * spacing);
                top.emplace_back((x + 0.5) * spacing, (y + 0.5) * spacing, height + (z + 0.5) * spacing);
            }
        }
    }
    system.AddFsiBoundary(bottom, ChFrame<>());
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetFixed(true);
    solid.AddBody(plate);
    system.AddRigidBody(plate, top, ChFrame<>(), false);
    soil.SetComputationalDomain(ChAABB(ChVector3d(0, 0, -4 * spacing), ChVector3d(width, width, height + 4 * spacing)), {BCType::PERIODIC, BCType::PERIODIC, BCType::NONE});
    system.SetStepSizeCFD(step);
    system.SetStepsizeMBD(step);
    system.Initialize();

#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> visual;
    if (render) {
        auto plugin = chrono_types::make_shared<ChSphVisualizationVSG>(&system);
        plugin->EnableRigidBodyMarkers(false);
        plugin->SetSPHColorCallback(chrono_types::make_shared<DisplacementColor>(initial), ChColormap::Type::VIRIDIS);
        auto shape = chrono_types::make_shared<ChVisualShapeBox>(width, width, spacing);
        shape->SetColor(ChColor(0.38f, 0.43f, 0.50f));
        plate->AddVisualShape(shape, ChFrame<>(ChVector3d(width / 2, width / 2, height + spacing / 2)));
        auto base = chrono_types::make_shared<ChBody>();
        base->AddVisualShape(shape, ChFrame<>(ChVector3d(width / 2, width / 2, -spacing / 2)));
        plugin->AddProxyBody(base);
        visual = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visual->AttachPlugin(plugin);
        visual->AttachSystem(&solid);
        visual->SetWindowTitle("CRM elastic shear");
        visual->SetCameraVertical(CameraVerticalDir::Z);
        visual->AddCamera(ChVector3d(2.5 * width, -3 * width, 2 * height), ChVector3d(width / 2, width / 2, height / 2));
        visual->Initialize();
    }
#endif

    // The plate stops at 7/9 of the run; the creep of the base is measured over the last 1/9.
    const int hold_check = 8 * steps / 9;
    Observation final;
    double hold_bottom = 0;
    bool hold_recorded = false;
    for (int n = 0; n < steps; ++n) {
        // Ramp the plate velocity up, maintain it, ramp down, then hold at U.
        double phase = std::clamp(9 * (n * step) / duration - 1, 0.0, 6.0);
        double travel = displacement * (phase < 1 ? 0.5 * phase * phase : (phase < 5 ? phase - 0.5 : 5 - 0.5 * (6 - phase) * (6 - phase))) / 5;
        double speed = displacement / (5 * duration / 9) * (phase < 1 ? phase : (phase < 5 ? 1 : 6 - phase));
        plate->SetPos(ChVector3d(travel, 0, 0));
        plate->SetPosDt(ChVector3d(speed, 0, 0));
        system.SynchronizeDevice();
        system.DoStepDynamics(step);
        system.SynchronizeDevice();
        if (n % 100 == 0 || n + 1 == steps) {
            final = Observe(soil, initial);
            if (!hold_recorded && n >= hold_check) {
                hold_bottom = final.bottom;
                hold_recorded = true;
            }
#ifdef CHRONO_VSG
            if (visual) {
                if (!visual->Run())
                    throw std::runtime_error("Visualization closed before verification completed");
                visual->Render();
            }
#endif
        }
    }

    // Elastic reference stresses. CRM rotates the stress with the material (Jaumann rate), so the shear strain U/H also
    // adds G (1 - cos(U/H)) to the vertical compression.
    double shear_modulus = young / (2 * (1 + poisson));
    double strain = displacement / height;
    double shear_reference = shear_modulus * strain;
    double normal_reference = pressure + shear_modulus * (1 - std::cos(strain));
    std::cout << "layers=" << layers << " shear=" << final.shear << " reference=" << shear_reference << '\n';

    // Each check passes if the magnitude of its value is below its limit. The q/p limit keeps the layer inside the yield
    // surface (q/p = mu), where the elastic references apply.
    struct Check {
        const char* name;
        double value;
        double limit;
    };
    const Check checks[] = {
        {"relative shear stress error", final.shear / shear_reference - 1, 0.02},
        {"relative normal stress error", final.normal / normal_reference - 1, 0.0005},
        {"displacement closure / U", (final.bottom + final.bulk - displacement) / displacement, 0.05},
        {"base slip / U", final.bottom / displacement, 0.05},
        {"mean q/p", final.yield_ratio, 0.9 * bulk_mu},
        {"base creep during the hold / U", (final.bottom - hold_bottom) / displacement, 0.001},
    };
    bool passed = true;
    for (const auto& check : checks) {
        bool ok = std::abs(check.value) < check.limit;
        std::cout << (ok ? "  pass  " : "  FAIL  ") << check.name << " = " << check.value << " (limit " << check.limit << ")\n";
        passed = passed && ok;
    }
    return passed;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        ChCLI cli(argv[0], "Verify confined elastic CRM shear");
        cli.AddOption<int>("Verification", "layers", "Particles through the layer height (at least 10)", "12");
        cli.AddOption<bool>("Visualization", "vis", "Show the test with VSG");
        if (!cli.Parse(argc, argv, true, false))
            return 1;
        if (cli.CheckHelp()) {
            cli.Help();
            return 0;
        }
        int layers = cli.GetAsType<int>("layers");
        if (layers < 10)
            throw std::invalid_argument("--layers must be at least 10 (with fewer, the periodic domain is too narrow for the neighbor search)");
        bool render = cli.GetAsType<bool>("vis");
#ifndef CHRONO_VSG
        if (render)
            throw std::invalid_argument("--vis requires Chrono::VSG");
#endif
        return VerifyShear(layers, render) ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
