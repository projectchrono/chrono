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

#include <iomanip>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::tdpf;

// -----------------------------------------------------------------------------
// Simulation parameters

double t_end = 200;
double time_step = 1.0e-2;
bool enforce_realtime = true;

bool lock = false;
bool verbose = false;

bool render_waves = true;
double render_fps = 30;
bool snapshots = false;

bool debug_sys = false;

bool save_output = false;

////ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type solver_type = ChSolver::Type::APGD;
ChSolver::Type solver_type = ChSolver::Type::GMRES;

bool use_diag_precond = true;

// -----------------------------------------------------------------------------
// MBS configuration

// Number of pontoon segments
constexpr int NUM_BODIES = 6;

// Joint configuration mode
enum class JointMode {
    HINGED_6,  // 6 articulated bodies - all 5 joints are revolute (hinged)
    HINGED_3,  // 3 effective bodies - pairs locked together (joints 0,2,4 fixed; 1,3 revolute)
    RIGID_1    // 1 rigid body - all 5 joints are fixed/locked
};

JointMode joint_mode = JointMode::HINGED_6;

// Body properties
struct BodyConfig {
    std::string name;
    ChVector3d location;
    double mass;
    ChVector3d inertia_moments;
    std::string mesh_file;
};

// Joint configuration
struct JointConfig {
    std::string name;
    int body1_idx;
    int body2_idx;
    ChVector3d location;
    ChVector3d axis;
};

// RSDA (Rotational Spring-Damper-Actuator) configuration
struct RsdaConfig {
    std::string name;
    int body1_idx;
    int body2_idx;
    ChVector3d location;
    ChVector3d axis;
    double spring_coeff;
    double damping_coeff;
    double free_angle;
};

// Body configurations
const std::array<BodyConfig, NUM_BODIES> BODY_CONFIGS = {{
    {"body1", ChVector3d(14.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_1.obj"},

    {"body2", ChVector3d(43.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_2.obj"},

    {"body3", ChVector3d(72.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_3.obj"},

    {"body4", ChVector3d(101.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_4.obj"},

    {"body5", ChVector3d(130.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_5.obj"},

    {"body6", ChVector3d(159.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_6.obj"},
}};

// Joint configurations (from vlfp.model.yaml)
const std::array<JointConfig, NUM_BODIES - 1> JOINT_CONFIGS = {{
    {"hinge_1_2", 0, 1, ChVector3d(29.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_2_3", 1, 2, ChVector3d(58.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_3_4", 2, 3, ChVector3d(87.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_4_5", 3, 4, ChVector3d(116.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_5_6", 4, 5, ChVector3d(145.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
}};

// RSDA configurations (from vlfp.model.yaml)
const std::array<RsdaConfig, NUM_BODIES - 1> RSDA_CONFIGS = {{
    {"damper_1_2", 0, 1, ChVector3d(29.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_2_3", 1, 2, ChVector3d(58.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_3_4", 2, 3, ChVector3d(87.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_4_5", 3, 4, ChVector3d(116.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_5_6", 4, 5, ChVector3d(145.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
}};

// -----------------------------------------------------------------------------
// Output setup

// Measurement Point Configuration
struct MeasurementConfig {
    double total_length = 174.0;
    double centerline_y = 29.0;
    double deck_z = 1.6;
    std::vector<double> x_over_L;
};

struct MeasurementPoint {
    double x_over_L;          // x/L ratio (0 to 1)
    double global_x;          // Global X coordinate
    int body_idx;             // Which body this point belongs to
    ChVector3d local_offset;  // Offset from body CG in body-local frame
};

// Data recorded at each measurement point per timestep
struct MeasurementPointData {
    std::vector<double> heave;  // Vertical displacement (Z)
    std::vector<double> surge;  // Horizontal displacement (X)
    std::vector<double> pitch;  // Rotation about Y axis

    void reserve(size_t n) {
        heave.reserve(n);
        surge.reserve(n);
        pitch.reserve(n);
    }
};

struct OutputData {
    std::vector<double> time;
    std::array<std::vector<double>, NUM_BODIES> heave;
    std::array<std::vector<double>, NUM_BODIES> surge;
    std::array<std::vector<double>, NUM_BODIES> pitch;
    std::array<std::vector<double>, NUM_BODIES - 1> relative_pitch;  // Between adjacent bodies

    // Measurement points data
    std::vector<MeasurementPointData> mp_data;  // One per measurement point

    void reserve(size_t n) {
        time.reserve(n);
        for (int i = 0; i < NUM_BODIES; ++i) {
            heave[i].reserve(n);
            surge[i].reserve(n);
            pitch[i].reserve(n);
        }
        for (int i = 0; i < NUM_BODIES - 1; ++i) {
            relative_pitch[i].reserve(n);
        }
    }

    void reserveMeasurementPoints(size_t num_points, size_t num_steps) {
        mp_data.resize(num_points);
        for (auto& mp : mp_data) {
            mp.reserve(num_steps);
        }
    }
};

// -----------------------------------------------------------------------------

// Forward declarations
MeasurementPoint CreateMeasurementPoint(double x_over_L, const MeasurementConfig& config, JointMode mode);
void SaveOutput(const std::string& out_dir,
                const OutputData& output,
                const std::vector<MeasurementPoint>& measurement_points);

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    auto vlfp_hydrofile = GetChronoDataFile("fsi-tdpf/vlfp/vlfp_bemio.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    double wave_height = 4.0;
    double wave_period = 10;
    double wave_amplitude = wave_height / 2;

    // ----- Multibody system
    ChSystemSMC sysMBS;
    sysMBS.SetSolverType(solver_type);
    sysMBS.GetSolver()->AsIterative()->SetMaxIterations(300);
    sysMBS.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(use_diag_precond);

    // Create ground body (add to system only if lock=true)
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    if (lock)
        sysMBS.AddBody(ground);

    // Create pontoon bodies
    std::cout << "Creating " << NUM_BODIES << " pontoon bodies...\n";
    std::array<std::shared_ptr<ChBody>, NUM_BODIES> bodies;

    ChColormap cmap(ChColormap::Type::KINDLMANN);
    for (int i = 0; i < NUM_BODIES; ++i) {
        const auto& cfg = BODY_CONFIGS[i];

        auto body = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("fsi-tdpf/vlfp/" + cfg.mesh_file),
                                                              1,      // density (not used since we set mass manually)
                                                              false,  // do not compute mass from mesh
                                                              true,   // create visualization asset
                                                              false   // no collision
        );
        body->SetName(cfg.name);
        body->SetPos(cfg.location);
        body->SetMass(cfg.mass);
        body->SetInertiaXX(cfg.inertia_moments);
        body->SetFixed(false);

        body->GetVisualShape(0)->SetColor(cmap.Get((i+1.0)/NUM_BODIES));

        sysMBS.Add(body);
        bodies[i] = body;
    }

    // Create joints (revolute or fixed based on configuration)
    std::cout << "Creating " << (NUM_BODIES - 1) << " joints...\n";
    std::array<std::shared_ptr<ChLink>, NUM_BODIES - 1> joints;

    for (int i = 0; i < NUM_BODIES - 1; ++i) {
        const auto& cfg = JOINT_CONFIGS[i];

        bool is_locked = (joint_mode == JointMode::RIGID_1) ||               //
                         (joint_mode == JointMode::HINGED_3 && i % 2 == 0);  //

        if (verbose)
            std::cout << "  joint #" << i << " locked? " << (is_locked ? "true" : "false") << std::endl;

        auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
        joint->SetName(cfg.name);
        joint->Initialize(bodies[cfg.body1_idx], bodies[cfg.body2_idx], ChFramed(cfg.location, Q_ROTATE_Y_TO_Z));
        joint->Lock(is_locked);

        sysMBS.AddLink(joint);
        joints[i] = joint;
    }

    // Create RSDAs
    std::cout << "Creating " << (NUM_BODIES - 1) << " RSDA dampers...\n";
    std::array<std::shared_ptr<ChLinkRSDA>, NUM_BODIES - 1> rsdas;

    for (int i = 0; i < NUM_BODIES - 1; ++i) {
        const auto& cfg = RSDA_CONFIGS[i];

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName(cfg.name);
        rsda->Initialize(bodies[cfg.body1_idx], bodies[cfg.body2_idx], ChFramed(cfg.location, Q_ROTATE_Y_TO_Z));
        rsda->SetSpringCoefficient(cfg.spring_coeff);
        rsda->SetDampingCoefficient(cfg.damping_coeff);
        rsda->SetRestAngle(cfg.free_angle);

        sysMBS.AddLink(rsda);
        rsdas[i] = rsda;
    }

    if (lock) {
        auto weld = chrono_types::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, bodies[0], ChFramed());
        sysMBS.AddLink(weld);
    }

    // ----- TDPF fluid system
    ChFsiFluidSystemTDPF sysTDPF;
    sysTDPF.SetHydroFilename(vlfp_hydrofile);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.num_bodies_ = NUM_BODIES;
    reg_wave_params.regular_wave_amplitude_ = wave_amplitude;
    reg_wave_params.regular_wave_omega_ = CH_2PI / wave_period;
    sysTDPF.AddWaves(reg_wave_params);

    if (verbose) {
        std::cout << "\nWave parameters\n";
        std::cout << "  Wave height:    " << wave_height << " m\n";
        std::cout << "  Wave period:    " << wave_period << " s\n";
        std::cout << "  Wave amplitude: " << wave_amplitude << " m\n";
        std::cout << "  Wave omega:     " << CH_2PI / wave_period << " rad/s\n";
    }

    // ----- FSI system
    ChFsiSystemTDPF sysFSI(&sysMBS, &sysTDPF);
    sysFSI.SetGravitationalAcceleration(g_acc);
    sysFSI.SetVerbose(verbose);
    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    // Add FSI bodies
    for (auto& body : bodies)
        sysFSI.AddFsiBody(body, nullptr, false);

    sysFSI.Initialize();

    // ----- Run-time visualization
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    auto visFSI = chrono_types::make_shared<ChTdpfVisualizationVSG>(&sysFSI);
    visFSI->SetWaveMeshVisibility(render_waves);
    visFSI->SetWaveMeshParams({87.0, 29.0}, {300.0, 200.0});
    visFSI->SetWaveMeshColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWaveMeshColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_amplitude, +wave_amplitude});
    visFSI->SetWaveMeshUpdateFrequency(render_fps);

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sysMBS);
    visVSG->SetWindowTitle("FSI-TDPF VLFP regular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(-45.0, -45.0, 45.0), ChVector3d(80.0, 30.0, 0.0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->SetModelScale(10);
    visVSG->ToggleCOMFrameVisibility();
    visVSG->ToggleCOMSymbolVisibility();

    visVSG->Initialize();
    vis = visVSG;
#endif

    // ----- Create output directory
    std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_vlfp";
    std::string img_dir = out_dir + "/reg_waves_img";
    std::string dbg_dir = out_dir + "/reg_waves_dbg_" + sysMBS.GetSolver()->GetTypeAsString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cerr << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }
    if (debug_sys) {
        if (!filesystem::create_directory(filesystem::path(dbg_dir))) {
            std::cerr << "Error creating directory " << dbg_dir << std::endl;
            return 1;
        }
    }

    // ----- Set up measurement points configuration
    MeasurementConfig mp_config;
    for (int i = 1; i <= 20; ++i) {  // Use 20 points from 0.05 to 1.0
        mp_config.x_over_L.push_back(i * 0.05);
    }

    std::vector<MeasurementPoint> measurement_points;
    for (double x_L : mp_config.x_over_L) {
        measurement_points.push_back(CreateMeasurementPoint(x_L, mp_config, joint_mode));
    }

    std::cout << "\nMeasurement points configured:\n";
    std::cout << "  Total length: " << mp_config.total_length << " m\n";
    std::cout << "  Number of points: " << measurement_points.size() << "\n";
    for (size_t i = 0; i < measurement_points.size(); ++i) {
        const auto& mp = measurement_points[i];
        std::cout << "  x/L=" << std::fixed << std::setprecision(2) << mp.x_over_L << " -> x=" << std::setprecision(1)
                  << mp.global_x << "m"
                  << " (body " << (mp.body_idx + 1) << ", offset=" << std::setprecision(2) << mp.local_offset.x()
                  << "m)\n";
    }

    // ----- Prepare output data
    size_t estimated_steps = static_cast<size_t>(t_end / time_step) + 100;
    OutputData output;
    output.reserve(estimated_steps);
    output.reserveMeasurementPoints(measurement_points.size(), estimated_steps);

    // ----- Simulation loop
    cout << "Using solver: " << sysMBS.GetSolver()->GetTypeAsString() << endl;

    ChRealtimeStepTimer realtime_timer;
    double time = 0;
    int render_frame = 0;

    while (time <= t_end) {
#ifdef CHRONO_VSG
        if (time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(5) << std::setfill('0') << render_frame << ".png";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }
#endif

        sysFSI.DoStepDynamics(time_step);

        // Output
        output.time.push_back(time);
        for (int i = 0; i < NUM_BODIES; ++i) {
            output.heave[i].push_back(bodies[i]->GetPos().z());
            output.surge[i].push_back(bodies[i]->GetPos().x());
            output.pitch[i].push_back(bodies[i]->GetRot().GetCardanAnglesXYZ().y());
        }
        for (int i = 0; i < NUM_BODIES - 1; ++i) {
            double pitch1 = bodies[i]->GetRot().GetCardanAnglesXYZ().y();
            double pitch2 = bodies[i + 1]->GetRot().GetCardanAnglesXYZ().y();
            output.relative_pitch[i].push_back(pitch2 - pitch1);
        }
        for (size_t mp_idx = 0; mp_idx < measurement_points.size(); ++mp_idx) {
            const auto& mp = measurement_points[mp_idx];
            const auto& body = bodies[mp.body_idx];
            ChVector3d global_pos = body->TransformPointLocalToParent(mp.local_offset);
            double body_pitch = body->GetRot().GetCardanAnglesXYZ().y();
            output.mp_data[mp_idx].surge.push_back(global_pos.x());
            output.mp_data[mp_idx].heave.push_back(global_pos.z());
            output.mp_data[mp_idx].pitch.push_back(body_pitch);
        }

        time += time_step;
        if (enforce_realtime)
            realtime_timer.Spin(time_step);
    }

    // Save output data
    if (save_output) {
        std::cout << "\nSaving output data...\n";
        SaveOutput(out_dir, output, measurement_points);
    }

#ifdef CHRONO_POSTPROCESS
    // ----- Output plot
    postprocess::ChGnuPlot gplot(out_dir + "/reg_waves.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave (m)");
    gplot.SetTitle("VLFP regular waves");
    for (int i = 0; i < NUM_BODIES; i++)
        gplot.Plot(output.time, output.heave[i], bodies[i]->GetName(), " with lines lw 2");
#endif

    return 0;
}

// -----------------------------------------------------------------------------

// Determine which body a global X position belongs to and compute local offset
// Body CG positions are at: 14.5, 43.5, 72.5, 101.5, 130.5, 159.5 (29m spacing)
// Body boundaries are: 0-29, 29-58, 58-87, 87-116, 116-145, 145-174
MeasurementPoint CreateMeasurementPoint(double x_over_L, const MeasurementConfig& config, JointMode mode) {
    MeasurementPoint mp;
    mp.x_over_L = x_over_L;
    mp.global_x = x_over_L * config.total_length;

    // Determine which body based on joint configuration
    // Body segment length is 29m (total_length / 6)
    double segment_length = config.total_length / 6.0;  // 29m

    switch (mode) {
        case JointMode::HINGED_6: {
            // 6 separate bodies, each 29m long
            // Body boundaries: 0-29, 29-58, 58-87, 87-116, 116-145, 145-174
            int body = static_cast<int>(mp.global_x / segment_length);
            mp.body_idx = std::min(body, 5);  // Clamp to 0-5

            // CG is at center of each segment
            double body_cg_x = (mp.body_idx + 0.5) * segment_length;  // 14.5, 43.5, ...
            mp.local_offset = ChVector3d(mp.global_x - body_cg_x, 0, 0);
            break;
        }
        case JointMode::HINGED_3: {
            // 3 effective bodies: pairs locked together
            // Bodies 0+1 (0-58m), 2+3 (58-116m), 4+5 (116-174m)
            // Effective CGs at: 29m, 87m, 145m
            double pair_length = 2.0 * segment_length;  // 58m
            int pair = static_cast<int>(mp.global_x / pair_length);
            pair = std::min(pair, 2);  // Clamp to 0-2

            // Use first body of each pair for reference
            mp.body_idx = pair * 2;  // 0, 2, or 4

            // Pair CG is at center of the 58m section
            double pair_cg_x = (pair + 0.5) * pair_length;  // 29, 87, 145
            mp.local_offset = ChVector3d(mp.global_x - pair_cg_x, 0, 0);
            break;
        }
        case JointMode::RIGID_1: {
            // Single rigid body - all segments locked
            // CG is at center: 87m
            mp.body_idx = 0;                                 // Reference to first body (but all are locked)
            double system_cg_x = config.total_length / 2.0;  // 87m
            mp.local_offset = ChVector3d(mp.global_x - system_cg_x, 0, 0);
            break;
        }
    }

    return mp;
}

// -----------------------------------------------------------------------------

// Save output data to file
void SaveOutput(const std::string& out_dir,
                const OutputData& output,
                const std::vector<MeasurementPoint>& measurement_points) {
    std::string out_file = out_dir + "/vlfp_reg_waves.txt";
    std::ofstream file(out_file);
    file << std::fixed << std::setprecision(6);

    // Header
    file << "# VLFP 6-Segment Regular Wave Response\n";
    file << "# Columns:\n";
    file << "#   1: time [s]\n";
    for (int i = 0; i < NUM_BODIES; ++i) {
        file << "#   " << (2 + i * 3) << "-" << (4 + i * 3) << ": body" << (i + 1)
             << " surge/heave/pitch [m, m, rad]\n";
    }
    file << "#   " << (2 + NUM_BODIES * 3) << "-" << (1 + NUM_BODIES * 3 + NUM_BODIES - 1)
         << ": relative pitch angles at joints [rad]\n";
    file << "#\n";

    // Column headers
    file << std::left << std::setw(12) << "time";
    for (int i = 0; i < NUM_BODIES; ++i) {
        file << std::setw(14) << ("surge_" + std::to_string(i + 1));
        file << std::setw(14) << ("heave_" + std::to_string(i + 1));
        file << std::setw(14) << ("pitch_" + std::to_string(i + 1));
    }
    for (int i = 0; i < NUM_BODIES - 1; ++i) {
        file << std::setw(14) << ("rel_pitch_" + std::to_string(i + 1) + "_" + std::to_string(i + 2));
    }
    file << "\n";

    // Data
    for (size_t t = 0; t < output.time.size(); ++t) {
        file << std::setw(12) << output.time[t];
        for (int i = 0; i < NUM_BODIES; ++i) {
            file << std::setw(14) << output.surge[i][t];
            file << std::setw(14) << output.heave[i][t];
            file << std::setw(14) << output.pitch[i][t];
        }
        for (int i = 0; i < NUM_BODIES - 1; ++i) {
            file << std::setw(14) << output.relative_pitch[i][t];
        }
        file << "\n";
    }

    file.close();
    std::cout << "  Output saved to: " << out_file << "\n";

    // Save measurement points data to separate file
    if (!measurement_points.empty()) {
        std::string mp_out_file = out_dir + "/vlfp_reg_waves_mpoints.txt";
        std::ofstream mp_file(mp_out_file);
        mp_file << std::fixed << std::setprecision(6);

        // Header
        mp_file << "# VLFP Measurement Points Response\n";
        mp_file << "# Measurement points (x/L ratios):\n";
        for (size_t i = 0; i < measurement_points.size(); ++i) {
            const auto& mp = measurement_points[i];
            mp_file << "#   Point " << (i + 1) << ": x/L=" << mp.x_over_L << " -> x=" << mp.global_x << "m (body "
                    << (mp.body_idx + 1) << ")\n";
        }
        mp_file << "#\n";
        mp_file << "# Columns:\n";
        mp_file << "#   1: time [s]\n";
        for (size_t i = 0; i < measurement_points.size(); ++i) {
            int col_start = 2 + static_cast<int>(i * 3);
            mp_file << "#   " << col_start << "-" << (col_start + 2) << ": point " << (i + 1)
                    << " (x/L=" << measurement_points[i].x_over_L << ") surge/heave/pitch [m, m, rad]\n";
        }
        mp_file << "#\n";

        // Column headers
        mp_file << std::left << std::setw(12) << "time";
        for (size_t i = 0; i < measurement_points.size(); ++i) {
            std::string suffix = "_" + std::to_string(static_cast<int>(measurement_points[i].x_over_L * 100));
            mp_file << std::setw(14) << ("surge" + suffix);
            mp_file << std::setw(14) << ("heave" + suffix);
            mp_file << std::setw(14) << ("pitch" + suffix);
        }
        mp_file << "\n";

        // Data
        for (size_t t = 0; t < output.time.size(); ++t) {
            mp_file << std::setw(12) << output.time[t];
            for (size_t i = 0; i < measurement_points.size(); ++i) {
                mp_file << std::setw(14) << output.mp_data[i].surge[t];
                mp_file << std::setw(14) << output.mp_data[i].heave[t];
                mp_file << std::setw(14) << output.mp_data[i].pitch[t];
            }
            mp_file << "\n";
        }

        mp_file.close();
        std::cout << "  Measurement points saved to: " << mp_out_file << "\n";
    }
}