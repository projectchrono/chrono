// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// RoboSimian on rigid terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <string>

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "robosimian.h"

#include "chrono/physics/ChForce.h"

#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_granular/physics/ChGranular.h"

#include "chrono_granular/physics/ChGranularTriMesh.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "../ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::granular;

double time_step = 1e-4;

// Drop the robot on rigid terrain
bool drop = true;

// Phase durations
double duration_pose = 1;            // Interval to assume initial pose
double duration_settle_robot = 0.5;  // Interval to allow robot settling on terrain
double duration_sim = 10;            // Duration of actual locomotion simulation

// Output frequencies
double output_fps = 100;
double render_fps = 100;

// Output directories
const std::string out_dir = "../ROBOSIMIAN";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// POV-Ray amd/or IMG output
bool data_output = true;
bool povray_output = false;
bool image_output = false;

// =============================================================================

class RobotGUIEventReceiver;

class RobotIrrApp : public irrlicht::ChIrrApp {
  public:
    RobotIrrApp(robosimian::RoboSimian* robot,
                robosimian::Driver* driver,
                const wchar_t* title = 0,
                irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800));

    ~RobotIrrApp();

    void EnableGrid(const ChCoordsys<>& csys, int nu, int nv);

    virtual void DrawAll() override;

  private:
    void renderTextBox(const std::string& msg,
                       int xpos,
                       int ypos,
                       int length = 120,
                       int height = 15,
                       irr::video::SColor color = irr::video::SColor(255, 20, 20, 20));

  private:
    robosimian::RoboSimian* m_robot;
    robosimian::Driver* m_driver;

    RobotGUIEventReceiver* m_erecv;

    int m_HUD_x;  ///< x-coordinate of upper-left corner of HUD elements
    int m_HUD_y;  ///< y-coordinate of upper-left corner of HUD elements

    bool m_grid;
    ChCoordsys<> m_gridCsys;
    int m_gridNu;
    int m_gridNv;

    friend class RobotGUIEventReceiver;
};

class RobotGUIEventReceiver : public irr::IEventReceiver {
  public:
    RobotGUIEventReceiver(RobotIrrApp* app) : m_app(app), m_vis(robosimian::VisualizationType::COLLISION) {}

    virtual bool OnEvent(const irr::SEvent& event) override;

  private:
    robosimian::VisualizationType m_vis;
    RobotIrrApp* m_app;
};

RobotIrrApp::RobotIrrApp(robosimian::RoboSimian* robot,
                         robosimian::Driver* driver,
                         const wchar_t* title,
                         irr::core::dimension2d<irr::u32> dims)
    : ChIrrApp(robot->GetSystem(), title, dims, false, true, true, irr::video::EDT_OPENGL),
      m_robot(robot),
      m_driver(driver),
      m_HUD_x(650),
      m_HUD_y(20),
      m_grid(false) {
    m_erecv = new RobotGUIEventReceiver(this);
    SetUserEventReceiver(m_erecv);
}

RobotIrrApp::~RobotIrrApp() {
    delete m_erecv;
}

void RobotIrrApp::EnableGrid(const ChCoordsys<>& csys, int nu, int nv) {
    m_gridCsys = csys;
    m_gridNu = nu;
    m_gridNv = nv;
    m_grid = true;
}

void RobotIrrApp::renderTextBox(const std::string& msg,
                                int xpos,
                                int ypos,
                                int length,
                                int height,
                                irr::video::SColor color) {
    irr::core::rect<irr::s32> mclip(xpos, ypos, xpos + length, ypos + height);
    GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                      irr::core::rect<irr::s32>(xpos, ypos, xpos + length, ypos + height), &mclip);
    irr::gui::IGUIFont* font = GetIGUIEnvironment()->getBuiltInFont();
    font->draw(msg.c_str(), irr::core::rect<irr::s32>(xpos + 3, ypos + 3, xpos + length, ypos + height), color);
}

void RobotIrrApp::DrawAll() {
    ChIrrAppInterface::DrawAll();

    if (m_grid) {
        irrlicht::ChIrrTools::drawGrid(GetVideoDriver(), 0.1, 0.1, m_gridNu, m_gridNv, m_gridCsys,
                                       irr::video::SColor(255, 255, 130, 80), true);
    }

    char msg[100];

    sprintf(msg, "Time %.2f", m_robot->GetSystem()->GetChTime());
    renderTextBox(msg, m_HUD_x, m_HUD_y, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "Driver phase: %s", m_driver->GetCurrentPhase().c_str());
    renderTextBox(msg, m_HUD_x, m_HUD_y + 30, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FR: %.2f", m_robot->GetWheelOmega(robosimian::FR));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 60, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RR: %.2f", m_robot->GetWheelOmega(robosimian::RR));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 75, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FL: %.2f", m_robot->GetWheelOmega(robosimian::FL));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 90, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RL: %.2f", m_robot->GetWheelOmega(robosimian::RL));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 105, 120, 15, irr::video::SColor(255, 250, 200, 00));
}

bool RobotGUIEventReceiver::OnEvent(const irr::SEvent& event) {
    if (event.EventType != irr::EET_KEY_INPUT_EVENT)
        return false;

    if (!event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case irr::KEY_KEY_C:
                m_vis = (m_vis == robosimian::VisualizationType::MESH ? robosimian::VisualizationType::COLLISION
                                                                      : robosimian::VisualizationType::MESH);

                m_app->m_robot->SetVisualizationTypeChassis(m_vis);
                m_app->m_robot->SetVisualizationTypeSled(m_vis);
                m_app->m_robot->SetVisualizationTypeLimbs(m_vis);
                m_app->m_robot->SetVisualizationTypeWheels(m_vis);

                m_app->AssetBindAll();
                m_app->AssetUpdateAll();

                return true;
        }
    }
    return false;
}

// =============================================================================

class RobotDriverCallback : public robosimian::Driver::PhaseChangeCallback {
  public:
    RobotDriverCallback(robosimian::RoboSimian* robot) : m_robot(robot), m_start_x(0), m_start_time(0) {}
    virtual void OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) override;

    double GetDistance() const;
    double GetDuration() const;
    double GetAvgSpeed() const;

    double m_start_x;
    double m_start_time;

  private:
    robosimian::RoboSimian* m_robot;
};

void RobotDriverCallback::OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) {
    if (new_phase == robosimian::Driver::CYCLE && old_phase != robosimian::Driver::CYCLE) {
        m_start_x = m_robot->GetChassisPos().x();
        m_start_time = m_robot->GetSystem()->GetChTime();
    }
}

double RobotDriverCallback::GetDistance() const {
    return m_robot->GetChassisPos().x() - m_start_x;
}

double RobotDriverCallback::GetDuration() const {
    return m_robot->GetSystem()->GetChTime() - m_start_time;
}

double RobotDriverCallback::GetAvgSpeed() const {
    return GetDistance() / GetDuration();
}

// =============================================================================

class RayCaster {
  public:
    RayCaster(ChSystem* sys, const ChFrame<>& origin, const ChVector2<>& dims, double spacing);

    const std::vector<ChVector<>>& GetPoints() const { return m_points; }

    void Update();

  private:
    ChSystem* m_sys;
    ChFrame<> m_origin;
    ChVector2<> m_dims;
    double m_spacing;
    std::shared_ptr<ChBody> m_body;
    std::shared_ptr<ChGlyphs> m_glyphs;
    std::vector<ChVector<>> m_points;
};

RayCaster::RayCaster(ChSystem* sys, const ChFrame<>& origin, const ChVector2<>& dims, double spacing)
    : m_sys(sys), m_origin(origin), m_dims(dims), m_spacing(spacing) {
    m_body = std::shared_ptr<ChBody>(sys->NewBody());
    m_body->SetBodyFixed(true);
    m_body->SetCollide(false);
    sys->AddBody(m_body);

    m_glyphs = std::make_shared<ChGlyphs>();
    m_glyphs->SetGlyphsSize(0.004);
    m_glyphs->SetZbufferHide(true);
    m_glyphs->SetDrawMode(ChGlyphs::GLYPH_POINT);
    m_body->AddAsset(m_glyphs);
}

void RayCaster::Update() {
    m_points.clear();

    ChVector<> dir = m_origin.GetA().Get_A_Zaxis();
    int nx = static_cast<int>(std::round(m_dims.x() / m_spacing));
    int ny = static_cast<int>(std::round(m_dims.y() / m_spacing));
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            double x_local = -0.5 * m_dims.x() + ix * m_spacing;
            double y_local = -0.5 * m_dims.y() + iy * m_spacing;
            ChVector<> from = m_origin.TransformPointLocalToParent(ChVector<>(x_local, y_local, 0.0));
            ChVector<> to = from + dir * 100;
            collision::ChCollisionSystem::ChRayhitResult result;
            m_sys->GetCollisionSystem()->RayHit(from, to, result);
            if (result.hit)
                m_points.push_back(result.abs_hitPoint);
        }
    }

    m_glyphs->Reserve(0);
    for (unsigned int id = 0; id < m_points.size(); id++) {
        m_glyphs->SetGlyphPoint(id, m_points[id], ChColor(1, 1, 0));
    }
}

// =============================================================================

void CreateCamera(irrlicht::ChIrrApp& application,
                  const irr::core::vector3df& position,
                  const irr::core::vector3df& target) {
    irrlicht::RTSCamera* camera =
        new irrlicht::RTSCamera(application.GetDevice(), application.GetSceneManager()->getRootSceneNode(),
                                application.GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);

    camera->setPosition(position);
    camera->setTarget(target);
    camera->setUpVector(irr::core::vector3df(0, 0, 1));
    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);
}

// =============================================================================

std::shared_ptr<ChBody> CreateTerrain(ChSystem& sys, const ChVector<>& hdim, const ChVector<>& loc) {
    auto ground = std::shared_ptr<ChBody>(sys.NewBody());
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(hdim.x(), hdim.y(), hdim.z(), loc);
    ground->GetCollisionModel()->BuildModel();

    float friction = 0.4f;
    switch (ground->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            ground->GetMaterialSurfaceNSC()->SetFriction(friction);
            break;
        case ChMaterialSurface::SMC:
            ground->GetMaterialSurfaceSMC()->SetFriction(friction);
            break;
    }

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = hdim;
    box->GetBoxGeometry().Pos = loc;
    ground->AddAsset(box);

    sys.AddBody(ground);

    return ground;
}

void ShowUsage() {
    cout << "usage: ./demo_GRAN_TriMeshNoFric_SMC_ballcosim <json_file>" << endl;
}

// Take a ChBody and write its
void writeMeshFrames(std::ostringstream& outstream,
                     ChBody& body,
                     std::string obj_name,
                     float mesh_scaling,
                     double terrain_offset) {
    // Write the mesh name to find
    outstream << obj_name << ",";

    // Get frame position
    ChVector<> pos = (body.GetPos() + ChVector<>(0, 0, terrain_offset)) * mesh_scaling;
    ChQuaternion<> rot = body.GetRot();
    // Get basis vectors
    ChVector<> vx = rot.GetXaxis();
    ChVector<> vy = rot.GetYaxis();
    ChVector<> vz = rot.GetZaxis();

    // Output in order
    outstream << pos.x() << ",";
    outstream << pos.y() << ",";
    outstream << pos.z() << ",";
    outstream << vx.x() << ",";
    outstream << vx.y() << ",";
    outstream << vx.z() << ",";
    outstream << vy.x() << ",";
    outstream << vy.y() << ",";
    outstream << vy.z() << ",";
    outstream << vz.x() << ",";
    outstream << vz.y() << ",";
    outstream << vz.z();
    outstream << "\n";
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ------------
    // Timed events
    // ------------

    double time_create_terrain = duration_pose;                       // create terrain after robot assumes initial pose
    double time_start = time_create_terrain + duration_settle_robot;  // start actual simulation after robot settling
    double time_end = time_start + duration_sim;                      // end simulation after specified duration

    // -------------
    // Create system
    // -------------

    ////ChSystemSMC my_sys;
    ChSystemNSC my_sys;

    my_sys.SetMaxItersSolverSpeed(200);
    if (my_sys.GetContactMethod() == ChMaterialSurface::NSC)
        my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Set output directory

    robot.SetOutputDirectory(out_dir);

    // Set actuation mode for wheel motors

    ////robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    // Control collisions (default: true for sled and wheels only)

    ////robot.SetCollide(robosimian::CollisionFlags::NONE);
    ////robot.SetCollide(robosimian::CollisionFlags::ALL);
    ////robot.SetCollide(robosimian::CollisionFlags::LIMBS);
    ////robot.SetCollide(robosimian::CollisionFlags::CHASSIS | robosimian::CollisionFlags::WHEELS);

    // Set visualization modes (default: all COLLISION)

    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimb(robosimian::FL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::FR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::NONE);
    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // Initialize Robosimian robot

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    "",                                                           // start input file
    ////    GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
    ////    "",                                                           // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/sculling_start.txt"),   // start input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_cycle2.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),    // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
    ////    true);
    auto driver = std::make_shared<robosimian::Driver>(
        GetChronoDataFile("robosimian/actuation/driving_start.txt"),  // start input file
        GetChronoDataFile("robosimian/actuation/driving_cycle.txt"),  // cycle input file
        GetChronoDataFile("robosimian/actuation/driving_stop.txt"),   // stop input file
        true);

    RobotDriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetTimeOffsets(duration_pose, duration_settle_robot);
    robot.SetDriver(driver);

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }
    if (image_output) {
        if (ChFileutils::MakeDirectory(img_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int output_steps = (int)std::ceil((1.0 / output_fps) / time_step);
    int render_steps = (int)std::ceil((1.0 / render_fps) / time_step);
    int sim_frame = 0;
    int output_frame = 0;
    int render_frame = 0;

    bool terrain_created = false;

    double iteration_step = 1e-4;

    // Mesh values
    string mesh_filename = string("grousery_wheel.obj");

    float3 scaling;
    scaling.x = 100;
    scaling.y = 100;
    scaling.z = 100;
    std::vector<std::shared_ptr<ChBodyAuxRef>> wheel_bodies;
    {
        auto limbs = robot.GetLimbs();
        wheel_bodies.push_back(limbs[robosimian::FR]->GetWheelBody());
        wheel_bodies.push_back(limbs[robosimian::FL]->GetWheelBody());
        wheel_bodies.push_back(limbs[robosimian::RR]->GetWheelBody());
        wheel_bodies.push_back(limbs[robosimian::RL]->GetWheelBody());
    }

    unsigned int num_mesh_wheels = wheel_bodies.size();

    std::vector<string> mesh_filenames;
    std::vector<float3> mesh_scalings;

    // add mesh to granular system
    for (unsigned int i = 0; i < num_mesh_wheels; i++) {
        mesh_scalings.push_back(scaling);
    }
    for (unsigned int i = 0; i < num_mesh_wheels; i++) {
        mesh_filenames.push_back(mesh_filename);
    }

    sim_param_holder params;
    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup granular simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys_gran(params.sphere_radius, params.sphere_density);
    m_sys_gran.setBOXdims(params.box_X, params.box_Y, params.box_Z);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points;

    double fill_bottom = -params.box_Z / 2;
    double fill_top = 2.05 * params.sphere_radius;
    chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);

    // fill box, layer by layer
    ChVector<> hdims(params.box_X / 2 - params.sphere_radius, params.box_Y / 2 - params.sphere_radius, 0);
    ChVector<> center(0, 0, fill_bottom);
    // shift up for bottom of box
    center.z() += 3 * params.sphere_radius;

    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    m_sys_gran.setParticlePositions(body_points);

    m_sys_gran.set_BD_Fixed(true);

    m_sys_gran.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys_gran.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys_gran.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys_gran.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2WALL(params.normalDampS2S);
    m_sys_gran.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);
    m_sys_gran.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys_gran.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys_gran.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys_gran.set_timeStepping(GRN_TIME_STEPPING::FIXED);
    m_sys_gran.set_timeIntegrator(GRN_TIME_INTEGRATOR::CHUNG);
    m_sys_gran.set_fixed_stepSize(params.step_size);

    m_sys_gran.load_meshes(mesh_filenames, mesh_scalings);

    m_sys_gran.disableMeshCollision();  // disable meshes for settling

    /// output preferences
    m_sys_gran.setOutputDirectory(params.output_dir);
    m_sys_gran.setOutputMode(params.write_mode);
    m_sys_gran.setVerbose(params.verbose);
    ChFileutils::MakeDirectory(params.output_dir.c_str());

    unsigned int nSoupFamilies = m_sys_gran.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys_gran.initialize();
    int currframe = 0;

    unsigned int out_fps = 100;
    unsigned int out_steps = 1 / (out_fps * iteration_step);

    double robot_granular_offset = 1.5;  // 1.5;

    double curr_time = 0;
    while (curr_time < time_end && curr_time < params.time_end) {
        if (drop && !terrain_created && my_sys.GetChTime() > time_create_terrain) {
            // Set terrain height to be _just_ below wheel
            double wheel_z = robot.GetWheelPos(robosimian::FR).z() - 0.13;

            // double z = wheel_z - 1;  // put solid terrain well below rover
            double max_gran_z = m_sys_gran.get_max_z() / 100;
            // we want the wheels just above terrain height
            robot_granular_offset = -wheel_z + max_gran_z;
            printf("offset is %f\n", robot_granular_offset);

            // add meshes back in
            m_sys_gran.enableMeshCollision();

            // Release robot
            robot.GetChassis()->GetBody()->SetBodyFixed(false);

            terrain_created = true;
        }

        if (data_output && sim_frame % output_steps == 0) {
            robot.Output();
        }

        // empty forces on each wheel
        for (unsigned int i = 0; i < num_mesh_wheels; i++) {
            auto wheel = wheel_bodies[i];
            wheel->Empty_forces_accumulators();
        }

        for (unsigned int i = 0; i < num_mesh_wheels; i++) {
            auto ball = wheel_bodies[i];
            auto ball_pos = ball->GetPos();
            auto ball_rot = ball->GetRot();

            unsigned int body_family_offset = i * 7;

            meshSoupLocOri[body_family_offset + 0] = (ball_pos.x()) * 100;
            meshSoupLocOri[body_family_offset + 1] = (ball_pos.y()) * 100;
            meshSoupLocOri[body_family_offset + 2] = (robot_granular_offset + ball_pos.z()) * 100;
            meshSoupLocOri[body_family_offset + 3] = ball_rot[0];
            meshSoupLocOri[body_family_offset + 4] = ball_rot[1];
            meshSoupLocOri[body_family_offset + 5] = ball_rot[2];
            meshSoupLocOri[body_family_offset + 6] = ball_rot[3];
        }
        m_sys_gran.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh

        float wheel_force[6 * num_mesh_wheels];
        m_sys_gran.collectGeneralizedForcesOnMeshSoup(wheel_force);
        // Apply forces to the ball for the duration of the iteration
        for (unsigned int i = 0; i < num_mesh_wheels; i++) {
            auto wheel = wheel_bodies[i];

            auto wheel_pos = wheel->GetPos();
            auto wheel_rot = wheel->GetRot();

            unsigned int body_family_offset = i * 6;

            double F_cgs_to_SI = 1e-5;
            double r_cgs_to_SI = 1e-2;

            wheel->Accumulate_force(
                F_cgs_to_SI * ChVector<>(wheel_force[body_family_offset + 0], wheel_force[body_family_offset + 1],
                                         wheel_force[body_family_offset + 2]),
                wheel_pos, false);
            wheel->Accumulate_torque(r_cgs_to_SI * F_cgs_to_SI *
                                         ChVector<>(wheel_force[body_family_offset + 3],
                                                    wheel_force[body_family_offset + 4], wheel_force[5]),
                                     false);
            if (sim_frame % render_steps == 0) {
                cout << "wheel " << i << " pos(" << wheel_pos.x() << ", " << wheel_pos.y() << ", " << wheel_pos.z()
                     << ") " << endl;
                cout << "force (" << wheel_force[0] << ", " << wheel_force[1] << ", " << wheel_force[2] << "); torque ("
                     << wheel_force[3] << ", " << wheel_force[4] << ", " << wheel_force[5] << ")" << endl;
            }
        }

        // Output POV-Ray date and/or snapshot images
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(&my_sys, filename);
            }

            cout << "Rendering frame " << render_frame << endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), render_frame);
            m_sys_gran.writeFileUU(string(filename));
            // write some VTKs for debug
            m_sys_gran.write_meshes(string(filename));

            // write mesh transforms for ospray renderer
            char mesh_output[100];
            sprintf(mesh_output, "%s/step%06d_meshes.csv", params.output_dir.c_str(), render_frame);
            std::ofstream meshfile{string(mesh_output)};
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3\n";

            // write each mesh to the output file
            for (auto b : wheel_bodies) {
                writeMeshFrames(outstream, *b, "grousery_wheel.obj", scaling.z, robot_granular_offset);
            }
            meshfile << outstream.str();

            render_frame++;
        }

        m_sys_gran.advance_simulation(time_step);

        robot.DoStepDynamics(time_step);

        curr_time += time_step;

        sim_frame++;
    }

    std::cout << "avg. speed: " << cbk.GetAvgSpeed() << std::endl;
    delete[] meshSoupLocOri;

    return 0;
}
