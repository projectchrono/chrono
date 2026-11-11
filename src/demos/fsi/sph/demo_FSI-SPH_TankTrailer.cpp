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
// Author: Radu Serban
// =============================================================================
//
// Sloshing in a partially filled cylindrical tank (axis along X), modeled with
// Chrono::FSI-SPH. Two modes are available:
// - vehicle:    the tank is mounted on a UTV trailer towed by a Polaris UTV, which
//               accelerates, performs a double lane change, and brakes hard;
// - prescribed: the tank follows a prescribed motion (acceleration, lane change,
//               and hard braking).
//
// Optionally, the demo saves data for rendering in Blender (see the --blender option).
//
// The SPH problem is solved in a reference frame that translates with the tank,
// so that the computational domain stays small and fixed no matter how far the
// vehicle travels. The tank body (A) is part of the multibody system. A mirror
// body (B), which carries the BCE markers, is kept at a fixed location with the
// orientation and angular velocity of the tank, and no translational motion. In
// this non-inertial frame, the fluid is subject to an effective gravity g - a,
// with a the acceleration of the tank. The fluid force and torque on the mirror
// body are applied to the tank body.
//
// =============================================================================

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "chrono_fsi/sph/ChFsiSplashsurfSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Tank interior dimensions and fill height (measured from the tank bottom)
double tank_radius = 0.33;
double tank_length = 1.40;
double fill_height = 0.33;

// Tank shell (visualization and inertia only)
double shell_thickness = 0.01;
double shell_mass = 40;

// Tank location on the trailer (relative to the trailer chassis reference frame, at the axle center).
// The trailer bed top is at z = 0.232; the tank rests on 5 cm saddles.
ChVector3d tank_loc_trailer(0.15, 0, 0.232 + 0.05 + 0.34);

// Initial height of the tank axis above ground (prescribed mode)
double tank_height = 0.95;

// Location of the mirror tank body, which carries the BCE markers (the SPH problem is solved around this point)
ChVector3d mirror_loc(0, 0, 0);

// -----------------------------------------------------------------------------

// Acceleration pulse a(t) = A sin^2(pi (t - t0) / T) on [t0, t0 + T], zero elsewhere.
struct Pulse {
    double t0;
    double T;
    double A;
};

// Motion along one direction obtained by superposing acceleration pulses (zero initial position and velocity).
class PulseMotion : public ChFunction {
  public:
    PulseMotion(const std::vector<Pulse>& pulses) : m_pulses(pulses) {}
    virtual PulseMotion* Clone() const override { return new PulseMotion(*this); }

    virtual double GetVal(double t) const override {
        double x = 0;
        for (const auto& p : m_pulses) {
            double s = t - p.t0;
            if (s <= 0)
                continue;
            if (s < p.T)
                x += p.A * (s * s / 4 + p.T * p.T / (8 * CH_PI * CH_PI) * (std::cos(CH_2PI * s / p.T) - 1));
            else
                x += p.A * (p.T * p.T / 4 + p.T / 2 * (s - p.T));
        }
        return x;
    }

    virtual double GetDer(double t) const override {
        double v = 0;
        for (const auto& p : m_pulses) {
            double s = t - p.t0;
            if (s <= 0)
                continue;
            if (s < p.T)
                v += p.A * (s / 2 - p.T / (4 * CH_PI) * std::sin(CH_2PI * s / p.T));
            else
                v += p.A * p.T / 2;
        }
        return v;
    }

    virtual double GetDer2(double t) const override {
        double a = 0;
        for (const auto& p : m_pulses) {
            double s = t - p.t0;
            if (s > 0 && s < p.T)
                a += p.A * std::pow(std::sin(CH_PI * s / p.T), 2);
        }
        return a;
    }

  private:
    std::vector<Pulse> m_pulses;
};

// -----------------------------------------------------------------------------

// FSI problem with SPH particles filling a horizontal cylindrical tank.
class TankProblem : public ChFsiProblemCartesian {
  public:
    TankProblem(double spacing, ChSystem* sys) : ChFsiProblemCartesian(spacing, sys) {}

    /// Create SPH particles in a cylinder with axis along X, centered at 'center', up to the given fill height.
    /// Particles are kept at least one spacing away from the tank walls (where the first BCE layer is placed).
    void FillTank(const ChVector3d& center, double radius, double length, double height) {
        int nr = (int)std::ceil(radius / m_spacing);
        int nl = (int)std::ceil(length / (2 * m_spacing));
        double r2_max = (radius - m_spacing) * (radius - m_spacing);
        double x_max = length / 2 - m_spacing + 1e-6;
        double z_max = height - radius + 1e-6;
        for (int ix = -nl; ix <= nl; ix++) {
            double x = ix * m_spacing;
            if (std::abs(x) > x_max)
                continue;
            for (int iy = -nr; iy <= nr; iy++) {
                double y = iy * m_spacing;
                for (int iz = -nr; iz <= nr; iz++) {
                    double z = iz * m_spacing;
                    if (z <= z_max && y * y + z * z <= r2_max)
                        m_sph.insert(ChVector3i(ix, iy, iz));
                }
            }
        }
        m_offset_sph = center;

        if (m_verbose)
            cout << "Fill tank: " << m_sph.size() << " SPH particles" << endl;
    }
};

// -----------------------------------------------------------------------------

// Coupling of the tank body (A) and its mirror (B), which carries the BCE markers.
// The mirror is kept at a fixed location, with the orientation, angular velocity, and angular acceleration of the tank
// and no translational velocity or acceleration. The SPH problem is thus solved in a frame translating with the tank.
// Before each step, the fluid gravity is set to g - a (see UpdateFluidGravity), where a is the tank acceleration at the
// end of the previous multibody step. Do not filter this acceleration: the delay introduced by a filter makes the
// coupling between the tank and the fluid (whose acoustic modes are low because of the artificial sound speed) unstable.
// Advancing the multibody system (through this MBD callback):
// - the fluid force and torque on the mirror (from the last data exchange) are applied to the tank;
// - the multibody system is advanced;
// - the mirror state is updated from the new tank state (before the next data exchange).
class TankMirror : public ChFsiSystem::MBDCallback {
  public:
    TankMirror(ChSystem& sys, double step, std::shared_ptr<ChBody> tank, std::shared_ptr<ChBody> mirror, TankProblem& fsi)
        : m_sys(sys), m_step(step), m_tank(tank), m_mirror(mirror), m_fsi(fsi) {
        m_acc = m_tank->AddAccumulator();
        m_mirror->SetFixed(true);
        UpdateMirror();
    }

    virtual void Advance(double step, double threshold) override {
        m_tank->EmptyAccumulator(m_acc);
        m_tank->AccumulateForce(m_acc, m_fsi.GetFsiBodyForce(m_mirror), m_tank->GetPos(), false);
        m_tank->AccumulateTorque(m_acc, m_fsi.GetFsiBodyTorque(m_mirror), false);

        double t = 0;
        while (t < step) {
            double h = std::min(m_step, step - t);
            if (h <= threshold)
                break;
            m_sys.DoStepDynamics(h);
            t += h;
        }

        UpdateMirror();
    }

    void UpdateMirror() {
        m_mirror->SetPos(mirror_loc);
        m_mirror->SetRot(m_tank->GetRot());
        m_mirror->SetPosDt(VNULL);
        m_mirror->SetPosDt2(VNULL);
        m_mirror->SetAngVelParent(m_tank->GetAngVelParent());
        m_mirror->SetAngAccParent(m_tank->GetAngAccParent());
    }

    void UpdateFluidGravity(const ChVector3d& gravity) { m_fsi.GetFluidSystemSPH()->SetGravitationalAcceleration(gravity - m_tank->GetPosDt2()); }

  private:
    ChSystem& m_sys;
    double m_step;
    std::shared_ptr<ChBody> m_tank;
    std::shared_ptr<ChBody> m_mirror;
    TankProblem& m_fsi;
    unsigned int m_acc;
};

// -----------------------------------------------------------------------------

// Create a tank body (reference frame at the tank center, axis along X), with or without visualization shapes
// (tank shell and the two saddles it rests on).
std::shared_ptr<ChBody> CreateTankBody(ChSystem& sys, const std::string& name, bool visualize) {
    double shell_radius = tank_radius + shell_thickness;
    auto tank = chrono_types::make_shared<ChBody>();
    tank->SetName(name);
    tank->SetMass(shell_mass);
    double Ixx = shell_mass * shell_radius * shell_radius;
    double Iyy = shell_mass * (shell_radius * shell_radius / 2 + tank_length * tank_length / 12);
    tank->SetInertiaXX(ChVector3d(Ixx, Iyy, Iyy));
    sys.AddBody(tank);

    if (visualize) {
        auto shape = chrono_types::make_shared<ChVisualShapeCylinder>(shell_radius, tank_length + 2 * shell_thickness);
        shape->SetColor(ChColor(0.7f, 0.8f, 0.9f));
        shape->SetOpacity(0.2f);
        tank->AddVisualShape(shape, ChFramed(VNULL, QuatFromAngleY(CH_PI_2)));

        auto saddle = chrono_types::make_shared<ChVisualShapeBox>(0.08, 0.5, 0.05);
        saddle->SetColor(ChColor(0.3f, 0.3f, 0.3f));
        tank->AddVisualShape(saddle, ChFramed(ChVector3d(+0.45, 0, -shell_radius - 0.025), QUNIT));
        tank->AddVisualShape(saddle, ChFramed(ChVector3d(-0.45, 0, -shell_radius - 0.025), QUNIT));
    }

    return tank;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    ChCLI cli(argv[0], "Sloshing in a cylindrical tank on a towed trailer or with prescribed motion");
    cli.AddOption<std::string>("Simulation", "mode", "Tank motion (vehicle/prescribed)", "vehicle");
    cli.AddOption<double>("Simulation", "spacing", "Initial particle spacing", "0.02");
    cli.AddOption<double>("Simulation", "step_size", "Integration step size", "1e-4");
    cli.AddOption<double>("Simulation", "t_end", "Simulation end time (default: 13 for vehicle mode, 7 for prescribed mode)", "-1");
    cli.AddOption<double>("Vehicle", "speed", "Vehicle target speed [km/h]", "35");
    cli.AddOption<double>("Output", "output_fps", "Frequency of tank force and fluid COM output", "100");
    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "blender", "Save data for rendering in Blender");
    cli.AddOption<double>("Output", "blender_fps", "Frequency of Blender output", "50");
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<std::string>("Visualization", "view", "Run-time view in vehicle mode (tank/vehicle)", "tank");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency", "100");
    cli.AddOption<bool>("Visualization", "snapshots", "Save snapshot image files");
    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return 1;
    }
    bool vehicle_mode = cli.GetAsType<std::string>("mode") != "prescribed";
    double spacing = cli.GetAsType<double>("spacing");
    double step_size = cli.GetAsType<double>("step_size");
    double t_end = cli.GetAsType<double>("t_end");
    if (t_end < 0)
        t_end = vehicle_mode ? 13 : 7;
    double target_speed = cli.GetAsType<double>("speed") / 3.6;
    double output_fps = cli.GetAsType<double>("output_fps");
    bool verbose = !cli.GetAsType<bool>("quiet");
    bool blender = cli.GetAsType<bool>("blender");
    double blender_fps = cli.GetAsType<double>("blender_fps");
    bool render = !cli.GetAsType<bool>("no_vis");
    bool vehicle_view = cli.GetAsType<std::string>("view") == "vehicle";
    double render_fps = cli.GetAsType<double>("render_fps");
    bool snapshots = cli.GetAsType<bool>("snapshots");

    // Create the multibody system
    ChSystemSMC sysMBS;
    const ChVector3d gravity(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Tank body (A) and its mirror (B), which carries the BCE markers.
    // The mirror is not visualized: the SPH particles are displayed in the tank (see ChSphVisualizationVSG::SetMarkersFrame).
    auto tank = CreateTankBody(sysMBS, "tank", true);
    auto mirror = CreateTankBody(sysMBS, "tank_mirror", false);

    // Vehicle mode: Polaris UTV towing a UTV trailer, with the tank welded to the trailer chassis.
    // Prescribed mode: tank driven by an imposed motion.
    std::shared_ptr<WheeledVehicle> vehicle;
    std::shared_ptr<WheeledTrailer> trailer;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<ChPathFollowerDriver> driver;
    ChVector3d veh_init_loc(-40, -15, 0.0);
    double x_brake = 0;

    if (vehicle_mode) {
        vehicle = chrono_types::make_shared<WheeledVehicle>(&sysMBS, GetVehicleDataFile("Polaris/Polaris.json"));
        vehicle->Initialize(ChCoordsys<>(veh_init_loc, QUNIT));
        vehicle->GetChassis()->SetFixed(false);
        vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetWheelVisualizationType(VisualizationType::MESH);
        auto engine = ReadEngineJSON(GetVehicleDataFile("Polaris/Polaris_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("Polaris/Polaris_AutomaticTransmissionSimpleMap.json"));
        vehicle->InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("Polaris/Polaris_TMeasyTire.json"));
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }

        trailer = chrono_types::make_shared<WheeledTrailer>(&sysMBS, GetVehicleDataFile("utv_trailer/UTV_Trailer.json"));
        trailer->Initialize(vehicle->GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::MESH);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::MESH);
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("Polaris/Polaris_TMeasyTire.json"));
                trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }

        // Weld the tank to the trailer chassis
        auto trailer_body = trailer->GetChassis()->GetBody();
        ChFramed tank_frame = trailer_body->GetFrameRefToAbs() * ChFramed(tank_loc_trailer, QUNIT);
        tank->SetPos(tank_frame.GetPos());
        tank->SetRot(tank_frame.GetRot());
        auto weld = chrono_types::make_shared<ChLinkLockLock>();
        weld->Initialize(tank, trailer_body, ChFrame<>(tank_frame.GetPos(), tank_frame.GetRot()));
        sysMBS.AddLink(weld);

        // Rigid terrain
        terrain = chrono_types::make_shared<RigidTerrain>(&sysMBS);
        auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
        patch_mat->SetFriction(0.9f);
        auto patch = terrain->AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300, 60);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 300, 60);
        terrain->Initialize();

        // Path follower driver: initial run, double lane change, final run (hard braking)
        double run = 30;
        double ramp = 10;
        double width = 3;
        double length = 5;
        auto path = DoubleLaneChangePath(ChVector3d(veh_init_loc.x(), veh_init_loc.y(), 0), ramp, width, length, run, true);
        x_brake = veh_init_loc.x() + run + 2 * ramp + length + 10;  // vehicle reference point, about 3.5 m ahead of the tank
        driver = chrono_types::make_shared<ChPathFollowerDriver>(*vehicle, path, "DLC", target_speed);
        driver->GetSteeringController().SetLookAheadDistance(5.0);
        driver->GetSteeringController().SetGains(0.5, 0, 0);
        driver->GetSpeedController().SetGains(0.4, 0, 0);
        driver->Initialize();
    } else {
        tank->SetPos(ChVector3d(0, 0, tank_height));

        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sysMBS.AddBody(ground);

        // Prescribed tank motion: accelerate, change lanes, brake hard
        auto motion_x = chrono_types::make_shared<PulseMotion>(std::vector<Pulse>{{0.5, 1.5, 4.0}, {3.5, 1.0, -6.0}});
        auto motion_y = chrono_types::make_shared<PulseMotion>(std::vector<Pulse>{{1.0, 1.0, 3.0}, {2.0, 1.0, -3.0}});
        auto motion = chrono_types::make_shared<ChFunctionPositionXYZFunctions>();
        motion->SetFunctionX(motion_x);
        motion->SetFunctionY(motion_y);
        motion->SetFunctionZ(chrono_types::make_shared<ChFunctionConst>(0));

        auto link = chrono_types::make_shared<ChLinkMotionImposed>();
        link->Initialize(tank, ground, ChFramed(tank->GetPos(), QUNIT));
        link->SetPositionFunction(motion);
        sysMBS.AddLink(link);
    }

    // Create the FSI problem
    TankProblem fsi(spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(gravity);
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1e-3;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 4.0;
    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.0;
    sph_params.shifting_diffusion_AFSM = 2.9;
    sph_params.shifting_diffusion_AFST = 2.0;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.artificial_viscosity = 0.02;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.num_bce_layers = 5;
    sph_params.num_proximity_search_steps = 1;
    fsi.SetSPHParameters(sph_params);

    // Mirror body coupling (also sets the initial mirror state)
    auto tank_mirror = chrono_types::make_shared<TankMirror>(sysMBS, step_size, tank, mirror, fsi);
    fsi.GetFsiSystemSPH()->RegisterMBDCallback(tank_mirror);

    // Tank walls: BCE markers outside a Z-aligned cylinder (side and end caps), rotated to align with the tank axis.
    // With fewer than 5 BCE layers, a few particles are pushed through the rear end cap as the fluid surges back after braking.
    auto bce = sysSPH->CreatePointsCylinderExterior(tank_radius, tank_length, true);
    fsi.AddRigidBody(mirror, bce, ChFramed(VNULL, QuatFromAngleY(CH_PI_2)), false);

    // Fluid, with hydrostatic initial pressure
    double fluid_top = mirror_loc.z() - tank_radius + fill_height;
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(fluid_top));
    fsi.FillTank(mirror_loc, tank_radius, tank_length, fill_height);

    // Computational domain: cover the mirror tank and its BCE layers for any yaw angle and small roll and pitch angles
    double bce_ext = sph_params.num_bce_layers * spacing;
    double half_xy = std::sqrt(std::pow(tank_length / 2 + bce_ext, 2) + std::pow(tank_radius + bce_ext, 2)) + 0.1;
    double half_z = tank_radius + bce_ext + 0.25;
    fsi.SetComputationalDomain(ChAABB(mirror_loc - ChVector3d(half_xy, half_xy, half_z), mirror_loc + ChVector3d(half_xy, half_xy, half_z)));

    fsi.Initialize();

    if (verbose) {
        cout << "Tank BCE markers: " << fsi.GetNumBCE(mirror) << endl;
        cout << "Fluid mass:       " << fsi.GetNumSPHParticles() * std::pow(spacing, 3) * fluid_props.density << endl;
    }

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Tank_Trailer";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    std::ostringstream case_name;
    case_name << (vehicle_mode ? "vehicle" : "prescribed") << "_d" << spacing << "_h" << step_size;
    out_dir = out_dir + "/" + case_name.str();
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (snapshots && !CreateOutputDirectory(std::filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }

    // Blender output:
    // - scene (vehicle, trailer, tank, terrain) through the ChBlender exporter, in the 'blender' subdirectory;
    // - fluid particle positions in the frame translating with the tank (for surface reconstruction with splashsurf),
    //   in the 'particles' subdirectory;
    // - translation of that frame relative to the world frame (tank position minus mirror location) for each frame, in
    //   'fluid_offset.txt': the fluid surface reconstructed from the particles is placed in the world by this offset.
#ifndef CHRONO_POSTPROCESS
    if (blender) {
        cerr << "Blender output requires the Chrono::PostProcess module" << endl;
        return 1;
    }
#endif
    std::unique_ptr<ChFsiSplashsurfSPH> particle_writer;
    std::ofstream offset_file;
    if (blender) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir + "/blender")) || !CreateOutputDirectory(std::filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating Blender output directories in " << out_dir << endl;
            return 1;
        }
        particle_writer = chrono_types::make_unique<ChFsiSplashsurfSPH>(*sysSPH);
        offset_file.open(out_dir + "/fluid_offset.txt");
        offset_file << "# frame  time  offset_x  offset_y  offset_z" << endl;
    }
#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(&sysMBS);
    if (blender) {
        blender_exporter.SetBasePath(out_dir + "/blender");
        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
#endif

    // Run-time visualization.
    // The SPH particles, which live in the frame translating with the tank, are displayed in the tank by setting the
    // frame of the SPH visualization plugin before each rendering (see below).
    // - tank view: camera following the tank;
    // - vehicle view (vehicle mode only): vehicle chase camera.
    ChVector3d cam_offset(-2.5, -3.0, 1.2);
    std::shared_ptr<ChVisualSystem> vis;
    std::shared_ptr<ChVehicleVisualSystem> vis_veh;
#ifdef CHRONO_VSG
    std::shared_ptr<ChSphVisualizationVSG> visFSI;
    if (render) {
        visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysSPH.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0), ChColormap::Type::FAST);

        std::shared_ptr<vsg3d::ChVisualSystemVSG> visVSG;
        if (vehicle_mode && vehicle_view) {
            auto visVeh = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            visVeh->AttachVehicle(vehicle.get());
            visVeh->SetChaseCamera(ChVector3d(-2.5, 0, 0.5), 9.0, 3.0);
            visVSG = visVeh;
            vis_veh = visVeh;
        } else {
            visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
            visVSG->AttachSystem(&sysMBS);
            visVSG->AddCamera(tank->GetPos() + cam_offset, tank->GetPos());
        }
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Tank sloshing");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Output: tank position, acceleration, and speed; fluid force on the tank; fluid center of mass in the tank frame
    std::ofstream ofile(out_dir + "/results.txt");
    ofile << "# time  x  y  z  ax  ay  az  speed  Fx  Fy  Fz  com_x  com_y  com_z" << endl;

    double time = 0;
    int out_frame = 0;
    int render_frame = 0;
    int blender_frame = 0;
    bool braking = false;
    DriverInputs driver_inputs = {0, 0, 0};
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (time >= out_frame / output_fps) {
            auto pos = sysSPH->GetParticlePositions();
            ChVector3d com = VNULL;
            int num_out = 0;
            size_t num_fluid = sysSPH->GetNumFluidMarkers();
            for (size_t i = 0; i < num_fluid; i++) {
                auto q = mirror->TransformPointParentToLocal(pos[i]);
                com += q;
                if (std::abs(q.x()) > tank_length / 2 || q.y() * q.y() + q.z() * q.z() > tank_radius * tank_radius)
                    num_out++;
            }
            com /= (double)num_fluid;
            const auto& F = fsi.GetFsiBodyForce(mirror);
            const auto& p = tank->GetPos();
            const auto& a = tank->GetPosDt2();
            double speed = tank->GetPosDt().Length();
            ofile << time << " " << p.x() << " " << p.y() << " " << p.z() << " "   //
                  << a.x() << " " << a.y() << " " << a.z() << " " << speed << " "  //
                  << F.x() << " " << F.y() << " " << F.z() << " "                  //
                  << com.x() << " " << com.y() << " " << com.z() << endl;
            if (verbose)
                cout << "t = " << time << "  v = " << speed << "  F = " << F << "  com = " << com << "  outside = " << num_out << endl;
            out_frame++;
        }

#ifdef CHRONO_POSTPROCESS
        if (blender && time >= blender_frame / blender_fps) {
            blender_exporter.ExportData();
            std::ostringstream filename;
            filename << out_dir << "/particles/fluid_" << std::setw(5) << std::setfill('0') << blender_frame << ".json";
            particle_writer->WriteParticleFileJSON(filename.str());
            auto offset = tank->GetPos() - mirror_loc;
            offset_file << blender_frame << " " << time << " " << offset.x() << " " << offset.y() << " " << offset.z() << endl;
            blender_frame++;
        }
#endif

#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            visFSI->SetMarkersFrame(ChFramed(tank->GetPos() - mirror_loc, QUNIT));
            if (!(vehicle_mode && vehicle_view))
                vis->UpdateCamera(tank->GetPos() + cam_offset, tank->GetPos());
            vis->Render();
            if (snapshots) {
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".png";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }
#endif

        // Vehicle mode: synchronize vehicle subsystems.
        // Hold the brakes for the first second to let the system settle; brake hard after the double lane change.
        if (vehicle_mode) {
            driver_inputs = driver->GetInputs();
            if (time < 1.0) {
                driver_inputs.m_throttle = 0;
                driver_inputs.m_braking = 1;
            }
            if (!braking && vehicle->GetPos().x() > x_brake) {
                braking = true;
                if (verbose)
                    cout << "Start braking at t = " << time << endl;
            }
            if (braking) {
                driver_inputs.m_throttle = 0;
                driver_inputs.m_braking = 1;
            }
            driver->Synchronize(time);
            terrain->Synchronize(time);
            vehicle->Synchronize(time, driver_inputs, *terrain);
            trailer->Synchronize(time, driver_inputs, *terrain);
            if (vis_veh)
                vis_veh->Synchronize(time, driver_inputs);
        }

        // Fluid gravity in the frame translating with the tank
        tank_mirror->UpdateFluidGravity(gravity);

        // Advance the coupled FSI problem (the multibody system is advanced through the mirror MBD callback)
        fsi.DoStepDynamics(step_size);

        if (vehicle_mode) {
            driver->Advance(step_size);
            terrain->Advance(step_size);
            vehicle->Advance(step_size);
            trailer->Advance(step_size);
            if (vis_veh)
                vis_veh->Advance(step_size);
        }

        time += step_size;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds" << endl;
    cout << "RTF: " << timer() / t_end << endl;

    return 0;
}
