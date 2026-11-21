// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by a BSD-style license in the LICENSE file.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

struct Observation {
    double shear = 0;
    double pressure = 0;
    double bottom_displacement = 0;
    double bulk_displacement = 0;
    double mean_pressure = 0;
    double yield_ratio = 0;
};

Observation Observe(ChFsiFluidSystemSPH& soil, const std::vector<ChVector3d>& initial, double height, double width) {
    auto positions = soil.GetParticlePositions();
    auto velocities = soil.GetParticleVelocities();
    auto diagonal = soil.GetParticleStressDiagonal();
    auto off_diagonal = soil.GetParticleStressOffDiagonal();
    Observation result;
    double n = 0, sum_z = 0, sum_u = 0, sum_zz = 0, sum_zu = 0;
    for (size_t i = 0; i < initial.size(); ++i) {
        double z = initial[i].z();
        double u = positions[i].x() - initial[i].x();
        u -= width * std::round(u / width);
        if (!std::isfinite(u) || !std::isfinite(diagonal[i].z()) || !std::isfinite(velocities[i].Length()))
            throw std::runtime_error("Nonfinite soil state");
        if (z > 0.25 * height && z < 0.75 * height) {
            n += 1;
            sum_z += z;
            sum_u += u;
            sum_zz += z * z;
            sum_zu += z * u;
            result.shear += off_diagonal[i].y();
            result.pressure -= diagonal[i].z();
            double p = -(diagonal[i].x() + diagonal[i].y() + diagonal[i].z()) / 3;
            ChVector3d deviator = diagonal[i] + ChVector3d(p);
            double q = std::sqrt(0.5 * deviator.Length2() + off_diagonal[i].Length2());
            result.mean_pressure += p;
            result.yield_ratio += q / p;
        }
    }
    double slope = (n * sum_zu - sum_z * sum_u) / (n * sum_zz - sum_z * sum_z);
    result.bottom_displacement = (sum_u - slope * sum_z) / n;
    result.bulk_displacement = slope * height;
    result.shear /= n;
    result.pressure /= n;
    result.mean_pressure /= n;
    result.yield_ratio /= n;
    return result;
}

bool CheckElastic(const Observation& state, double stiffness, double displacement, double pressure, double hold_creep) {
    double traction_error = std::abs(state.shear / (stiffness * displacement) - 1);
    double normal_error = std::abs(state.pressure / pressure - 1);
    double closure = std::abs(state.bottom_displacement + state.bulk_displacement - displacement) / displacement;
    bool valid = traction_error < 0.08 && normal_error < 0.05 && closure < 0.1 && std::abs(state.bottom_displacement) < 0.1 * displacement && state.yield_ratio < 0.9 * 0.7 &&
                 hold_creep < 0.01;
    std::cout << std::setprecision(17) << "ELASTIC traction_error=" << traction_error << " normal_error=" << normal_error << " closure=" << closure << " hold_creep=" << hold_creep
              << " valid=" << valid << '\n';
    return valid;
}

#ifdef CHRONO_VSG
class DisplacementColor : public ChSphVisualizationVSG::ParticleColorCallback {
  public:
    explicit DisplacementColor(const std::vector<ChVector3d>& initial) : m_initial(initial) {}
    std::string GetTile() const override { return "Horizontal displacement (mm)"; }
    ChVector2d GetDataRange() const override { return {0, 0.24}; }
    ChColor GetColor(unsigned int n) const override {
        double u = pos[n].x - m_initial[n].x();
        u -= 0.04 * std::round(u / 0.04);
        return m_vsys->GetColormap().Get(u, 0, 0.00024);
    }

  private:
    const std::vector<ChVector3d>& m_initial;
};
#endif

int main(int argc, char** argv) {
    try {
        ChCLI cli(argv[0], "Confined elastic CRM shear: tau = G U/H");
        cli.AddOption<bool>("Visualization", "no_vis", "Disable live VSG visualization");
        cli.AddOption<bool>("Visualization", "snapshots", "Save live VSG screenshots");
        cli.AddOption<bool>("Validation", "validate", "Check the full loading and hold response");
        cli.AddOption<bool>("Output", "frames", "Export particle frames for offline rendering");
        cli.AddOption<std::string>("Output", "output", "Output directory", GetChronoOutputPath() + "FSI_Planar_Shear");
        if (!cli.Parse(argc, argv))
            return cli.CheckHelp() ? 0 : 1;
        bool validate = cli.GetAsType<bool>("validate");
        bool render = !cli.GetAsType<bool>("no_vis");
        bool snapshots = cli.GetAsType<bool>("snapshots");
        bool export_frames = cli.GetAsType<bool>("frames");
#ifndef CHRONO_VSG
        render = false;
#endif
        if (snapshots && !render)
            throw std::invalid_argument("Snapshots require live VSG visualization");
        std::filesystem::path output(cli.GetAsType<std::string>("output"));
        std::filesystem::create_directories(output);
        constexpr double step = 2e-5;
        constexpr double duration = 0.36;
        constexpr double height = 0.04;
        constexpr double young = 1e6;
        constexpr double poisson = 0.3;
        constexpr double pressure = 5000;
        constexpr double bulk_mu = 0.7;
        constexpr double displacement = 0.00024;
        const double shear_modulus = young / (2 * (1 + poisson));
        const double spacing = height / 12;
        constexpr int lateral = 12;
        const double width = lateral * spacing;
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
                for (int z = 0; z < 12; ++z) {
                    ChVector3d position((x + 0.5) * spacing, (y + 0.5) * spacing, (z + 0.5) * spacing);
                    initial.push_back(position);
                    soil.AddSPHParticle(position, VNULL, ChVector3d(-pressure), VNULL);
                }
                for (int z = 0; z < 3; ++z) {
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

        std::shared_ptr<ChVisualSystem> visual;
#ifdef CHRONO_VSG
        if (render) {
            auto sph_visual = chrono_types::make_shared<ChSphVisualizationVSG>(&system);
            sph_visual->EnableBoundaryMarkers(false);
            sph_visual->EnableRigidBodyMarkers(false);
            sph_visual->SetSPHColorCallback(chrono_types::make_shared<DisplacementColor>(initial), ChColormap::Type::VIRIDIS);
            auto shape = chrono_types::make_shared<ChVisualShapeBox>(width + 0.006, width + 0.006, 0.0015);
            shape->SetColor(ChColor(0.38f, 0.43f, 0.50f));
            plate->AddVisualShape(shape, ChFrame<>(ChVector3d(width / 2, width / 2, height + 0.00075)));
            auto base = chrono_types::make_shared<ChBody>();
            base->AddVisualShape(shape, ChFrame<>(ChVector3d(width / 2, width / 2, -0.00075)));
            sph_visual->AddProxyBody(base);
            if (snapshots) {
                std::filesystem::create_directories(output / "snapshots");
                sph_visual->SetImageOutputDirectory((output / "snapshots").string());
                sph_visual->SetImageOutput(true);
            }
            auto vsg_visual = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
            vsg_visual->AttachPlugin(sph_visual);
            vsg_visual->AttachSystem(&solid);
            vsg_visual->SetWindowTitle("CRM elastic shear - true displacement, no magnification");
            vsg_visual->SetWindowSize(1280, 800);
            vsg_visual->SetCameraVertical(CameraVerticalDir::Z);
            vsg_visual->AddCamera(ChVector3d(0.105, -0.13, 0.09), ChVector3d(0.02, 0.02, 0.02));
            vsg_visual->SetLightIntensity(0.9f);
            vsg_visual->Initialize();
            visual = vsg_visual;
        }
#endif
        std::ofstream history(output / "capture.csv");
        history.exceptions(std::ios::failbit | std::ios::badbit);
        history << std::setprecision(17) << "time,top_displacement,shear,normal_stress,bottom_extrapolated_displacement,bulk_displacement,mean_pressure,yield_ratio\n";
        std::ofstream frames;
        if (export_frames) {
            frames.open(output / "capture.frames.csv");
            frames.exceptions(std::ios::failbit | std::ios::badbit);
            frames << std::setprecision(17) << "time,top_displacement,id,initial_x,x,y,z,displacement_x,shear_xz\n";
        }
        auto write_frame = [&](double time, double travel) {
            auto positions = soil.GetParticlePositions();
            auto stress = soil.GetParticleStressOffDiagonal();
            for (size_t i = 0; i < initial.size(); ++i) {
                double u = positions[i].x() - initial[i].x();
                u -= width * std::round(u / width);
                frames << time << ',' << travel << ',' << i << ',' << initial[i].x() << ',' << positions[i].x() << ',' << positions[i].y() << ',' << positions[i].z() << ',' << u
                       << ',' << stress[i].y() << '\n';
            }
        };
        if (export_frames)
            write_frame(0, 0);
        if (visual)
            visual->Render();
        const int steps = int(std::ceil(duration / step));
        Observation observation;
        double hold_start_displacement = 0;
        int completed_steps = 0;
        double final_travel = 0;
        for (int n = 0; n < steps; ++n) {
            double t = n * step;
            double phase = std::clamp(9 * t / duration - 1, 0.0, 6.0);
            double speed = displacement / (5 * duration / 9);
            double u = displacement * (phase < 1 ? 0.5 * phase * phase : (phase < 5 ? phase - 0.5 : 5 - 0.5 * (6 - phase) * (6 - phase))) / 5;
            speed *= phase < 1 ? phase : (phase < 5 ? 1 : 6 - phase);
            final_travel = u;
            plate->SetPos(ChVector3d(u, 0, 0));
            plate->SetPosDt(ChVector3d(speed, 0, 0));
            system.SynchronizeDevice();
            system.DoStepDynamics(step);
            system.SynchronizeDevice();
            if (n % 100 == 0 || n + 1 == steps) {
                observation = Observe(soil, initial, height, width);
                if (t <= 8 * duration / 9)
                    hold_start_displacement = observation.bottom_displacement;
                history << t + step << ',' << u << ',' << observation.shear << ',' << observation.pressure << ',' << observation.bottom_displacement << ','
                        << observation.bulk_displacement << ',' << observation.mean_pressure << ',' << observation.yield_ratio << '\n';
            }
            completed_steps = n + 1;
            if ((n + 1) % 100 == 0 || n + 1 == steps) {
                if (export_frames)
                    write_frame(t + step, u);
                if (visual) {
                    if (!visual->Run())
                        break;
                    visual->Render();
                }
            }
        }
        std::ofstream state(output / "capture.state.csv");
        state.exceptions(std::ios::failbit | std::ios::badbit);
        state << std::setprecision(17);
        auto position = soil.GetParticlePositions();
        auto velocity = soil.GetParticleVelocities();
        auto diagonal = soil.GetParticleStressDiagonal();
        auto off = soil.GetParticleStressOffDiagonal();
        for (size_t i = 0; i < initial.size(); ++i)
            state << position[i].x() << ',' << position[i].y() << ',' << position[i].z() << ',' << velocity[i].x() << ',' << velocity[i].y() << ',' << velocity[i].z() << ','
                  << diagonal[i].x() << ',' << diagonal[i].y() << ',' << diagonal[i].z() << ',' << off[i].x() << ',' << off[i].y() << ',' << off[i].z() << '\n';
        state.close();
        history.close();
        if (export_frames)
            frames.close();
        std::cout << std::setprecision(17) << "Final shear: " << observation.shear << " Pa; elastic reference: " << shear_modulus * final_travel / height << " Pa\n";
        if (validate) {
            if (completed_steps != steps)
                throw std::runtime_error("Validation requires the complete loading and hold interval");
            double hold_creep = std::abs(observation.bottom_displacement - hold_start_displacement) / displacement;
            return CheckElastic(observation, shear_modulus / height, displacement, pressure, hold_creep) ? 0 : 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
