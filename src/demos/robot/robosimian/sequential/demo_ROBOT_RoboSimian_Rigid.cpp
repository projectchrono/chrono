// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
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

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/robot/robosimian/RoboSimian.h"
#include "chrono_models/robot/robosimian/RoboSimianVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;

double time_step = 5e-4;

// Drop the robot on rigid terrain
bool drop = true;

// Robot locomotion mode
robosimian::LocomotionMode mode = robosimian::LocomotionMode::WALK;

// Contact method (system type)
ChContactMethod contact_method = ChContactMethod::SMC;

// Phase durations
double duration_pose = 1.0;          // Interval to assume initial pose
double duration_settle_robot = 0.5;  // Interval to allow robot settling on terrain
double duration_sim = 10;            // Duration of actual locomotion simulation

// Output frequencies
double output_fps = 100;
double render_fps = 60;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ROBOSIMIAN_RIGID";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// POV-Ray and/or IMG output
bool data_output = true;
bool povray_output = false;
bool image_output = false;

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

    m_glyphs = chrono_types::make_shared<ChGlyphs>();
    m_glyphs->SetGlyphsSize(0.004);
    m_glyphs->SetZbufferHide(true);
    m_glyphs->SetDrawMode(ChGlyphs::GLYPH_POINT);
    m_body->AddVisualShape(m_glyphs);
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

std::shared_ptr<ChBody> CreateTerrain(ChSystem* sys, double length, double width, double height, double offset) {
    float friction = 0.8f;
    float Y = 1e7f;
    float cr = 0.0f;

    auto ground_mat = ChMaterialSurface::DefaultMaterial(sys->GetContactMethod());
    ground_mat->SetFriction(friction);
    ground_mat->SetRestitution(cr);

    if (sys->GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat)->SetYoungModulus(Y);
    }

    auto ground = std::shared_ptr<ChBody>(sys->NewBody());
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(ground_mat, length / 2, width / 2, 0.1, ChVector<>(offset, 0, height - 0.1));
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(length / 2, width / 2, 0.1);
    box->SetTexture(GetChronoDataFile("textures/pinkwhite.png"), 10 * (float)length, 10 * (float)width);
    ground->AddVisualShape(box, ChFrame<>(ChVector<>(offset, 0, height - 0.1), QUNIT));

    sys->AddBody(ground);

    return ground;
}

void SetContactProperties(robosimian::RoboSimian* robot) {
    float friction = 0.8f;
    float Y = 1e7f;
    float cr = 0.0f;

    robot->GetSledContactMaterial()->SetFriction(friction);
    robot->GetSledContactMaterial()->SetRestitution(cr);

    robot->GetWheelContactMaterial()->SetFriction(friction);
    robot->GetWheelContactMaterial()->SetRestitution(cr);

    if (robot->GetSystem()->GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChMaterialSurfaceSMC>(robot->GetSledContactMaterial())->SetYoungModulus(Y);
        std::static_pointer_cast<ChMaterialSurfaceSMC>(robot->GetWheelContactMaterial())->SetYoungModulus(Y);
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ------------
    // Timed events
    // ------------

    double time_create_terrain = duration_pose;  // create terrain after robot assumes initial pose

    // -------------
    // Create system
    // -------------

    ChSystem* my_sys = nullptr;
    switch (contact_method) {
        case ChContactMethod::NSC:
            my_sys = new ChSystemNSC;
            break;
        case ChContactMethod::SMC:
            my_sys = new ChSystemSMC;
            break;
    }

    my_sys->SetSolverMaxIterations(200);
    my_sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    my_sys->Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys->Set_G_acc(ChVector<double>(0, 0, 0));

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(my_sys, true, true);

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

    std::shared_ptr<robosimian::RS_Driver> driver;
    switch (mode) {
        case robosimian::LocomotionMode::WALK:
            driver = chrono_types::make_shared<robosimian::RS_Driver>(
                "",                                                                 // start input file
                GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt"),  // cycle input file
                "",                                                                 // stop input file
                true);
            break;
        case robosimian::LocomotionMode::SCULL:
            driver = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/sculling_start.txt"),   // start input file
                GetChronoDataFile("robot/robosimian/actuation/sculling_cycle2.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/sculling_stop.txt"),    // stop input file
                true);
            break;
        case robosimian::LocomotionMode::INCHWORM:
            driver = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/inchworming_start.txt"),  // start input file
                GetChronoDataFile("robot/robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/inchworming_stop.txt"),   // stop input file
                true);
            break;
        case robosimian::LocomotionMode::DRIVE:
            driver = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/driving_start.txt"),  // start input file
                GetChronoDataFile("robot/robosimian/actuation/driving_cycle.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/driving_stop.txt"),   // stop input file
                true);
            break;
    }

    robosimian::RS_DriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetTimeOffsets(duration_pose, duration_settle_robot);
    robot.SetDriver(driver);

    // -------------------------------
    // Cast rays into collision models
    // -------------------------------

    ////RayCaster caster(my_sys, ChFrame<>(ChVector<>(2, 0, -1), Q_from_AngY(-CH_C_PI_2)), ChVector2<>(2.5, 2.5), 0.02);
    RayCaster caster(my_sys, ChFrame<>(ChVector<>(0, -2, -1), Q_from_AngX(-CH_C_PI_2)), ChVector2<>(2.5, 2.5), 0.02);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<robosimian::RoboSimianVisualSystemIrrlicht>(&robot, driver.get());
    vis->AttachSystem(my_sys);
    vis->SetWindowTitle("RoboSimian - Rigid terrain");
    vis->SetWindowSize(800, 600);
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, -2.75, 0.2), ChVector<>(1, 0, 0));
    vis->AddLight(ChVector<>(100, +100, 100), 290, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(100, -100, 80), 190, ChColor(0.7f, 0.8f, 0.8f));
    ////vis->AddLightWithShadow(ChVector<>(10.0, -6.0, 3.0), ChVector<>(0, 0, 0), 3, -10, 10, 40, 512);
    ////vis->EnableShadows();

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }
    if (image_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
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
    int render_frame = 0;

    bool terrain_created = false;

    while (vis->Run()) {
        ////caster.Update();

        if (drop && !terrain_created && my_sys->GetChTime() > time_create_terrain) {
            // Set terrain height
            double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;

            // Rigid terrain parameters
            double length = 8;
            double width = 2;

            // Create terrain
            ChVector<> hdim(length / 2, width / 2, 0.1);
            ChVector<> loc(length / 4, 0, z - 0.1);
            auto ground = CreateTerrain(my_sys, length, width, z, length / 4);
            vis->BindItem(ground);
            SetContactProperties(&robot);

            // Release robot
            robot.GetChassisBody()->SetBodyFixed(false);

            terrain_created = true;
        }

        vis->BeginScene();
        vis->Render();

        if (data_output && sim_frame % output_steps == 0) {
            robot.Output();
        }

        // Output POV-Ray date and/or snapshot images
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_sys, filename);
            }
            if (image_output) {
                char filename[100];
                sprintf(filename, "%s/img_%04d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }

            render_frame++;
        }

        ////double time = my_sys->GetChTime();
        ////double A = CH_C_PI / 6;
        ////double freq = 2;
        ////double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        ////robot.Activate(robosimian::FR, "joint2", time, val);
        ////robot.Activate(robosimian::RL, "joint5", time, val);

        robot.DoStepDynamics(time_step);

        ////if (my_sys->GetNcontacts() > 0) {
        ////    robot.ReportContacts();
        ////}

        sim_frame++;

        vis->EndScene();
    }

    std::cout << "avg. speed: " << cbk.GetAvgSpeed() << std::endl;

    delete my_sys;
    return 0;
}
