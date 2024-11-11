// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Luning Bakke, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Final simulation time
double t_end = 20.0;

// Position and dimensions of WEC device
ChVector3d wec_pos(-2.9875, 0, -0.1);
ChVector3d wec_size(0.225, 0.975, 1.2);
double wec_density = 500;

// Container dimensions
ChVector3d csize(12, 1.25, 1.8);

// Beach start location
double x_start = 8;

// Fluid depth
double depth = 1.3;

// Output frequency
bool output = true;
double output_fps = 10;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 200;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// Size of initial volume of SPH fluid
// wec location
// ChVector3d wec_pos(-2.9875, -0.0125, -0.1);
//// Size of the baffles
// ChVector3d wec_size(0.225, 0.975, 1.2);
// ChVector3d fsize(12, 1.25, 1.3);  // fluid slightly higher than the flap.
//// Container dimensions
// ChVector3d csize(12, 1.25, 1.8);

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override {
        auto p = pos[n];
        return p.y >= 0;
    }
};

// -----------------------------------------------------------------------------

class WaveFunction : public ChFunction {
  public:
    WaveFunction() : delay(0), a2(0), omega(0) {}
    WaveFunction(double delay, double amplitude, double frequency)
        : delay(delay), a2(amplitude / 2), omega(CH_2PI * frequency) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        if (t <= delay)
            return 0;
        double tt = t - delay;
        return a2 * (1 - std::cos(omega * tt));
    }

  private:
    double delay;
    double a2;
    double omega;
};

class WaveFunctionDecay : public ChFunction {
  public:
    // stroke s0, period T, with an exponential decay
    WaveFunctionDecay() : s0(0.1), T(1) {}
    WaveFunctionDecay(double s0, double period) : s0(s0 / 2), T(period) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        return 0.5 * s0 * (1 - std::exp(-t * 2 / T)) * std::sin(2. * CH_PI / T * t);
    }

  private:
    double s0;  // stroke
    double T;   // period
};

// -----------------------------------------------------------------------------

// Wave tank profile for a beach represented as a 4th order Bezier curve.
class WaveTankBezierBeach : public ChFsiProblemCartesian::WaveTankProfile {
  public:
    WaveTankBezierBeach(double x_start) : x_start(x_start), last_t(1e-2) {
        const double in2m = 0.0254;
        P0 = in2m * ChVector2d(0, 0);
        P1 = in2m * ChVector2d(0, 28.77);
        P2 = in2m * ChVector2d(62.04, 50.83);
        P3 = in2m * ChVector2d(90.28, 59.26);
        P4 = in2m * ChVector2d(197.63, 61.19);

        Q0 = 4.0 * (P1 - P0);
        Q1 = 4.0 * (P2 - P1);
        Q2 = 4.0 * (P3 - P2);
        Q3 = 4.0 * (P4 - P3);
    }

    virtual double operator()(double x) {
        if (x <= x_start)
            return 0;

        double xx = x - x_start;
        if (xx >= P4.x())
            return P4.y();

        // Find t such that P(t).x = xx (Newton)
        int N = 10;
        double tol = 1e-5;
        double t = last_t;
        for (int i = 0; i < N; i++) {
            double f = eval(t, 0) - xx;
            if (std::abs(f) < tol)
                break;
            double fp = eval_der(t, 0);
            assert(std::abs(fp) > tol);
            t -= f / fp;
        }
        last_t = t;

        // Return h = P(t).y
        return eval(t, 1);
    }

  private:
    // Evaluate Bezier curve at given parameter 0 <= t <= 1 and return specified coordinate.
    double eval(double t, int i) {
        double omt = 1 - t;
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t2 * t2;
        double omt2 = omt * omt;
        double omt3 = omt2 * omt;
        double omt4 = omt2 * omt2;
        return omt4 * P0[i] + 4 * t * omt3 * P1[i] + 6 * t2 * omt2 * P2[i] + 4 * t3 * omt * P3[i] + t4 * P4[i];
    }

    // Evaluate Bezier curve derivative at given parameter 0 <= t <= 1 and return specified coordinate.
    double eval_der(double t, int i) {
        double omt = 1 - t;
        double t2 = t * t;
        double t3 = t * t2;
        double omt2 = omt * omt;
        double omt3 = omt2 * omt;
        return omt3 * Q0[i] + 3 * t * omt2 * Q1[i] + 3 * t2 * omt * Q2[i] + t3 * Q3[i];
    }

    ChVector2d P0;
    ChVector2d P1;
    ChVector2d P2;
    ChVector2d P3;
    ChVector2d P4;

    ChVector2d Q0;
    ChVector2d Q1;
    ChVector2d Q2;
    ChVector2d Q3;

    double x_start;
    double last_t;
};

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkLockRevolute> CreateFlap(ChFsiProblemSPH& fsi) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Common contact material and geometry
    ChContactMaterialData cmat;
    cmat.Y = 1e8f;
    cmat.mu = 0.2f;
    cmat.cr = 0.05f;

    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(cmat);
    geometry.coll_boxes.push_back(
        utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0.5 * wec_size.z()), QUNIT, wec_size, 0));

    auto flap = chrono_types::make_shared<ChBody>();
    flap->SetPos(wec_pos);
    flap->SetRot(QUNIT);
    flap->SetFixed(false);

    double wec_volume = wec_size.x() * wec_size.y() * wec_size.z();

    double wec_mass = wec_volume * wec_density;
    flap->SetMass(wec_mass);
    flap->SetInertiaXX(ChVector3d(0.5 * wec_mass * wec_size.y() * wec_size.y(),
                                  0.5 * wec_mass * wec_size.z() * wec_size.z(),
                                  0.5 * wec_mass * wec_size.y() * wec_size.y()));

    sysMBS.AddBody(flap);
    if (show_rigid)
        geometry.CreateVisualizationAssets(flap, VisualizationType::COLLISION);
    fsi.AddRigidBody(flap, geometry, true, true);

    // add ground
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetMass(2 * wec_mass);
    ground->SetInertiaXX(flap->GetInertiaXX());
    sysMBS.AddBody(ground);

    // Add revolute joint
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(ground, flap, ChFrame<>(wec_pos, Q_ROTATE_Z_TO_Y));
    sysMBS.AddLink(revolute);

    return revolute;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.05;
    double step_size = 5e-5;
    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();
    ChFluidSystemSPH& sysSPH = fsi.GetFluidSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set CFD fluid properties
    ChFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    sysSPH.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 8;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 0.0;
    // sph_params.density_reinit_steps = 800;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    sph_params.artificial_viscosity = 0.02;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sysSPH.SetSPHParameters(sph_params);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);
    sysSPH.SetNumBCELayers(5);

    // Create WEC device
    auto revolute = CreateFlap(fsi);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(sysSPH, depth));

    // Create a wave tank
    double stroke = 0.1;
    double period = 2;
    auto fun = chrono_types::make_shared<WaveFunctionDecay>(stroke, period);

    auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                           //
                                      ChVector3d(0, 0, 0), csize, depth,                                //
                                      fun,                                                              //
                                      chrono_types::make_shared<WaveTankBezierBeach>(x_start), false);  //

    // Initialize the FSI system
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flap/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    out_dir = out_dir + sysSPH.GetSphMethodTypeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        cerr << "Error creating directory " << out_dir + "/particles" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        cerr << "Error creating directory " << out_dir + "/fsi" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        cerr << "Error creating directory " << out_dir + "/vtk" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/wave_fun.gpl");
    gplot.SetGrid();
    std::string speed_title = "Wave function";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(*fun, 0, 5, 0.02, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

////fsi.SaveInitialMarkers(out_dir);

// Create a run-time visualizer
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(
            ChColor(1, 0, 0), ChColor(0.14f, 0.44f, 0.7f), -1000, 12000);

        visFSI->SetTitle("Chrono::FSI Flap");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(wec_pos + ChVector3d(0, -9 * csize.y(), 0), wec_pos);
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->SetLightIntensity(0.9);
        visFSI->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBS = 0;
    double timer_FSI = 0;
    double timer_step = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBS += sysFSI.GetTimerMBS();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (verbose && sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBS:  " << timer_MBS << endl;
            cout << "   timer FSI:  " << timer_FSI << endl;
            cout << "   timer step: " << timer_step << endl;
        }

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;
}
