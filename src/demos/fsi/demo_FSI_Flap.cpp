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
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Final simulation time
double t_end = 10.0;
double initial_spacing = 0.01;

// Position and dimensions of WEC device
// ChVector3d wec_pos(-2.9875, 0, -0.1);
ChVector3d wec_pos(-1.5, 0, -0.1);

ChVector3d wec_size(0.076, 0.4, 0.56);
double wec_density = 500;

// Container dimensions
ChVector3d csize(5.0, 0.5, 0.7);  // original size????

// Beach start location
double x_start = csize.x() / 2;

double pto_damping = 0;

// Fluid depth
// double depth = 1.3;

// this is for testing wec deivce
double depth = 0.4;

// Output frequency
bool output = true;
double output_fps = 10;

// write info frequency
double csv_fps = 200;

// Enable/disable run-time visualization
bool render = false;
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

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y >= 0; }
};
#endif

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
    WaveFunctionDecay() : a2(0.1), omega(1) {}
    WaveFunctionDecay(double s0, double frequency) : a2(s0 / 2), omega(CH_2PI * frequency) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        double T = CH_2PI / omega;
        return a2 * (1 - std::exp(-t / T)) * (1 - std::cos(2. * CH_PI / T * t));
    }

  private:
    double a2;     // stroke
    double omega;  // period
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

std::shared_ptr<ChLinkLockRevolute> CreateFlap(ChFsiProblemSPH& fsi, double mini_window_angle) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Common contact material and geometry
    ChContactMaterialData cmat;
    cmat.Y = 1e8f;
    cmat.mu = 0.2f;
    cmat.cr = 0.05f;

    // TODO: use multiple chbodies

    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(cmat);
    // geometry.coll_boxes.push_back(
    //     utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0.5 * wec_size.z()), QUNIT, wec_size, 0));

    double door_thickness = 0.076;
    double door_height = 0.56;
    double door_width = 0.4;
    double window_width = 0.35;
    double mini_window_height = 0.076;
    int num_windows = 4;

    double window_height = 0.076 * num_windows;
    double top_panel_height = 0.0575;
    double bottom_panel_height = door_height - window_height - top_panel_height;
    double bottom_panel_thickness = door_thickness / 2;

    // TODO (Luning): i think this is messing me up, causing instability
    // let me inflate this to door thickness and see what happens
    double mini_window_rb = 0.01;
    ////double mini_window_rb = door_thickness/2;
    ////double mini_window_ra = mini_window_height / 2;
    ////double mini_window_angle = 0;   // when angle is 0, all windows are closed!
    ////double mini_window_angle = 45 / 180. * CH_PI;

    // location of front box -y
    ChVector3d front_box_pos(0, -(door_width + window_width) / 4, door_height / 2);
    ChVector3d back_box_pos(0, (door_width + window_width) / 4, door_height / 2);
    ChVector3d thin_plate_size(door_thickness, (door_width - window_width) / 2, door_height);
    ChVector3d top_panel_pos(0, 0, door_height - top_panel_height / 2);
    ChVector3d top_panel_size(door_thickness, window_width - initial_spacing * 2, top_panel_height);
    ChVector3d bottom_panel_pos(0, 0, bottom_panel_height / 2);
    ChVector3d bottom_panel_size(bottom_panel_thickness, window_width - initial_spacing * 2, bottom_panel_height);
    ChVector3d mini_window_size;
    // mini window size all the same! 0 angle is when the windows are closed
    if (mini_window_angle > 0 && mini_window_angle < CH_PI_2)
        mini_window_size = ChVector3d(mini_window_rb * 2, window_width - 2 * initial_spacing,
                                      mini_window_height - 4 * initial_spacing);
    else
        mini_window_size =
            ChVector3d(mini_window_rb * 2, window_width - 2 * initial_spacing, mini_window_height - initial_spacing);

    ChVector3d mini_window_pos(0, 0, bottom_panel_height + mini_window_height / 2);

    std::cout << "mini_window_size: " << mini_window_size << std::endl;
    std::cout << "mini_window_pos: " << mini_window_pos << std::endl;

    //  ***********************
    //  **  mini_window_0    **
    //  ***********************

    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(front_box_pos, QUNIT, thin_plate_size, 0));
    std::cout << "Add front box at location " << front_box_pos << ", size of : " << thin_plate_size
              << " and initial spacing of: " << initial_spacing << std::endl;
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(back_box_pos, QUNIT, thin_plate_size, 0));
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(top_panel_pos, QUNIT, top_panel_size, 0));
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(bottom_panel_pos, QUNIT, bottom_panel_size, 0));

    for (int i = 0; i < num_windows; i++) {
        geometry.coll_boxes.push_back(
            utils::ChBodyGeometry::BoxShape(mini_window_pos, QuatFromAngleY(mini_window_angle), mini_window_size, 0));
        mini_window_pos.z() += mini_window_height;
    }

    auto flap = chrono_types::make_shared<ChBody>();
    flap->SetPos(wec_pos);
    flap->SetRot(QUNIT);
    flap->SetFixed(false);

    /////////////////////////////////////////////////////////
    // is there a way to compute inertia automatically???? //
    //////////////////////////////////////////////////////////
    double wec_volume = wec_size.x() * wec_size.y() * wec_size.z();

    double wec_mass = wec_volume * wec_density;
    flap->SetMass(wec_mass);
    std::cout << "wec_mass: " << wec_mass << "kg " << std::endl;
    flap->SetInertiaXX(ChVector3d(0.5 * wec_mass * wec_size.y() * wec_size.y(),
                                  0.5 * wec_mass * wec_size.z() * wec_size.z(),
                                  0.5 * wec_mass * wec_size.y() * wec_size.y()));
    std::cout << "wec_inertia: " << std::endl << flap->GetInertiaXX() << std::endl;
    sysMBS.AddBody(flap);
    if (show_rigid)
        geometry.CreateVisualizationAssets(flap, VisualizationType::COLLISION);

    // TODO: do the class thing, so it initialize mass and inertia as well
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

    // add a torsional spring to the joint
    double C_ext = 6.57;  // from omae paper
    auto torsional_spring = chrono_types::make_shared<ChLinkRSDA>();
    torsional_spring->Initialize(ground, flap, false, ChFrame<>(wec_pos, Q_ROTATE_Z_TO_Y),
                                 ChFrame<>(wec_pos, Q_ROTATE_Z_TO_Y));
    torsional_spring->SetSpringCoefficient(C_ext);
    torsional_spring->SetDampingCoefficient(pto_damping);
    sysMBS.AddLink(torsional_spring);

    return revolute;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double step_size = 2.5e-5;  // used to be 5e-5!
    bool verbose = true;

    if (argc != 4) {
        std::cout << "Usage: demo_FSI_Flap <flap angle DEG, 0 closed, 90 fully open> <PTO damping. 0.5-5> <wave T>"
                  << std::endl;
        return 0;
    }

    double mini_window_angle = atof(argv[1]) / 180. * CH_PI;
    pto_damping = atof(argv[2]);
    double period = atof(argv[3]);  // range from omae paper: 0.8 to 2.0

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.num_bce_layers = 5;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 4;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    sph_params.artificial_viscosity = 0.02;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;

    sph_params.num_bce_layers = 5;
    fsi.SetSPHParameters(sph_params);
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Create WEC device
    std::shared_ptr<ChLinkLockRevolute> revolute = CreateFlap(fsi, mini_window_angle);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(depth));

    // Create a wave tank
    // double stroke = 0.1;  //
    // double stroke = 0.1;
    double stroke = 0.06;
    double frequency = 1 / period;
    auto fun = chrono_types::make_shared<WaveFunctionDecay>(stroke, frequency);

    // auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                           //
    //                                   ChVector3d(0, 0, 0), csize, depth,                                //
    //                                   fun,                                                              //
    //                                   chrono_types::make_shared<WaveTankBezierBeach>(x_start), false);  //

    auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                                  //
                                      ChVector3d(0, 0, 0), csize, depth,                                       //
                                      fun, chrono_types::make_shared<WaveTankRampBeach>(x_start, 0.2), true);  //

    // Initialize the FSI system
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir =
        GetChronoOutputPath() + "FSI_Flap_pto_" + argv[2] + "_window_" + argv[1] + "_DEG" + "_waveT_" + argv[3];
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    out_dir = out_dir + fsi.GetSphMethodTypeString();
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
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(
            ChColor(1, 0, 0), ChColor(0.14f, 0.44f, 0.7f), -1000, 12000);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("WEC Device");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(wec_pos + ChVector3d(0, -9 * csize.y(), 0), wec_pos);
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int csv_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBD = 0;
    double timer_FSI = 0;
    double timer_step = 0;

    ChTimer timer;
    timer.start();
    ChVector3 reaction_force;
    double flap_angular_velo;  // pitch velo
    double pto_power;

    // create a csv file to store the reaction force
    // create a csv file to store the reaction force
    std::ofstream ofile;
    ofile.open(out_dir + "/info.csv");
    ofile << "time,Fx,Fy,Fz,angle,angle_dt,pto_power" << std::endl;

    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            printf("write file: %s\n", (out_dir + "/fsi").c_str());
            out_frame++;
        }

        if (output && time >= csv_frame / csv_fps) {
            // get the reaction force
            reaction_force = revolute->GetReaction2().force;
            flap_angular_velo = revolute->GetRelativeAngVel().z();
            pto_power = pto_damping * flap_angular_velo * flap_angular_velo;
            ofile << time << ", " << reaction_force.x() << ", " << reaction_force.y() << ", " << reaction_force.z()
                  << ", " << revolute->GetRelAngle() << "," << flap_angular_velo << ", " << pto_power << std::endl;
            csv_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBD += sysFSI.GetTimerMBD();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (verbose && sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBD:  " << timer_MBD << endl;
            cout << "   timer FSI:  " << timer_FSI << endl;
            cout << "   timer step: " << timer_step << endl;
        }

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    ofile.close();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;
}
