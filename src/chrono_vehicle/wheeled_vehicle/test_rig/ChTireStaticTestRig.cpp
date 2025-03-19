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
// Authors: Radu Serban
// =============================================================================
//
// Implementation of a single-tire static test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireStaticTestRig.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChLoadContainer.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChTireStaticTestRig::ChTireStaticTestRig(std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, ChSystem* system)
    : m_system(system),
      m_grav(9.8),
      m_wheel(wheel),
      m_tire(tire),
      m_r_load(5000),
      m_r_speed(0.001),
      m_x_speed(0.001),
      m_y_speed(0.001),
      m_z_speed(0.01),
      m_transition_delay(0.1),
      m_transition_time(0),
      m_tire_step(1e-3),
      m_output(false),
      m_gnuplot(false),
      m_mode(Mode::SUSPEND),
      m_state(State::FIXED),
      m_tire_vis(VisualizationType::PRIMITIVES) {
    // Default tire-terrain collision method
    m_tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
}

ChTireStaticTestRig::~ChTireStaticTestRig() {
    if (m_output)
        WriteOutput();
}

// -----------------------------------------------------------------------------

void ChTireStaticTestRig::SetTireCollisionType(ChTire::CollisionType coll_type) {
    m_tire->SetCollisionType(coll_type);
}

void ChTireStaticTestRig::SetPlateMaterialProperties(double friction, double restitution, double Young_modulus) {
    m_plate_friction = (float)friction;
    m_plate_restitution = (float)restitution;
    m_plate_Young_modulus = (float)Young_modulus;
}

void ChTireStaticTestRig::SetOutput(const std::string& out_dir, bool gnuplot) {
    m_output = true;
    m_gnuplot = gnuplot;
    m_outdir = out_dir;
}

// -----------------------------------------------------------------------------

// Wrapper to add a delay before a given ChFunction
class DelayedFun : public ChFunction {
  public:
    DelayedFun() : m_fun(nullptr), m_delay(0) {}
    DelayedFun(std::shared_ptr<ChFunction> fun, double delay) : m_fun(fun), m_delay(delay) {}
    virtual DelayedFun* Clone() const override { return new DelayedFun(); }
    virtual double GetVal(double x) const override {
        if (x < m_delay)
            return 0;
        return m_fun->GetVal(x - m_delay);
    }
    std::shared_ptr<ChFunction> m_fun;
    double m_delay;
};

// -----------------------------------------------------------------------------

void ChTireStaticTestRig::Initialize(Mode mode) {
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, -m_grav));

    auto tire_radius = m_tire->GetRadius();

    const double dim = 0.1;                     // base dimension
    double post_offset = 3 * dim;               // in y direciton
    double spindle_height = tire_radius + 0.2;  // initial spindle/wheel height
    double plate_dim = 1.5;                     // plate width and length
    double plate_thickness = 0.1;               // plate thickness

    // Create bodies
    ////const double mass = m_wheel->GetWheelMass() + m_tire->GetTireMass();
    ////const ChVector3d inertia = m_wheel->GetWheelInertia() + m_tire->GetTireInertia();

    // Create the ground body
    m_ground_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_ground_body);
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetFixed(true);
    {
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));
        auto post_box = chrono_types::make_shared<ChVisualShapeBox>(dim, dim / 3, 2 * tire_radius + 2 * dim);
        post_box->SetMaterial(0, vis_mat);
        m_ground_body->AddVisualShape(post_box, ChFramed(ChVector3d(0, -post_offset, 2 * tire_radius), QUNIT));
        m_ground_body->AddVisualShape(post_box, ChFramed(ChVector3d(0, +post_offset, 2 * tire_radius), QUNIT));
        auto base_box = chrono_types::make_shared<ChVisualShapeBox>(50 * dim, 50 * dim, plate_thickness);
        m_ground_body->AddVisualShape(base_box, ChFramed(ChVector3d(0, 0, -1.5 * plate_thickness), QUNIT));
    }

    // Create the spindle (wheel) body
    m_spindle_body = chrono_types::make_shared<ChBody>();
    m_spindle_body->SetFixed(mode == Mode::SUSPEND);
    m_system->AddBody(m_spindle_body);
    m_spindle_body->SetName("rig_spindle");
    m_spindle_body->SetMass(0);
    m_spindle_body->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
    m_spindle_body->SetPos(ChVector3d(0, 0, spindle_height));
    m_spindle_body->SetRot(QUNIT);
    {
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetKdTexture(GetChronoDataFile("textures/blue.png"));
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(dim / 2, 2 * post_offset);
        cyl->SetMaterial(0, vis_mat);
        m_spindle_body->AddVisualShape(cyl, ChFramed(VNULL, Q_ROTATE_Z_TO_Y));
    }

    // Initialize wheel and tire subsystems
    m_wheel->Initialize(nullptr, m_spindle_body, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);
    m_wheel->SetTire(m_tire);
    m_tire->SetStepsize(m_tire_step);
    m_tire->Initialize(m_wheel);
    m_tire->SetVisualizationType(m_tire_vis);

    // Create the rig plate body
    m_plate_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_plate_body);
    m_plate_body->SetName("rig_plate");
    m_plate_body->SetMass(100);
    m_plate_body->SetInertiaXX({1, 1, 1});
    m_plate_body->SetPos(VNULL);
    m_plate_body->SetRot(QUNIT);
    m_plate_body->EnableCollision(true);

    // Create collision and visualization geometry
    ChContactMaterialData mat_info;
    mat_info.mu = m_plate_friction;
    mat_info.cr = m_plate_restitution;
    mat_info.Y = m_plate_Young_modulus;
    auto contact_mat = mat_info.CreateMaterial(m_system->GetContactMethod());
    {
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));
        utils::AddBoxGeometry(m_plate_body.get(), contact_mat, ChVector3d(plate_dim, plate_dim, plate_thickness),
                              ChVector3d(0, 0, -0.5 * plate_thickness), QUNIT, true, vis_mat);
    }

    // Create a motor to apply radial load
    m_motor_r = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    m_system->AddLink(m_motor_r);
    m_motor_r->Initialize(m_ground_body, m_spindle_body, ChFrame<>());
    m_motor_r->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());

    // Connect plate to ground through a plane-plane joint
    auto plate_plate_link = chrono_types::make_shared<ChLinkMatePlanar>();
    m_system->AddLink(plate_plate_link);
    plate_plate_link->Initialize(m_ground_body, m_plate_body, ChFrame<>(VNULL, QUNIT));

    // Create plate actuators (set to FREE, so that they do not introduce additional constraints)
    m_motor_x = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    m_system->AddLink(m_motor_x);
    m_motor_x->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::FREE);
    m_motor_x->Initialize(m_ground_body, m_plate_body, ChFrame<>(VNULL, Q_ROTATE_Z_TO_X));
    m_motor_x->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());

    m_motor_y = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    m_system->AddLink(m_motor_y);
    m_motor_y->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::FREE);
    m_motor_y->Initialize(m_ground_body, m_plate_body, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));
    m_motor_y->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());

    m_motor_z = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    m_system->AddLink(m_motor_z);
    m_motor_z->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::FREE);
    m_motor_z->Initialize(m_ground_body, m_plate_body, ChFrame<>(VNULL, QUNIT));
    m_motor_z->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());

    // Create a stand-in terrain system
    m_terrain.h = 0;
    m_terrain.mu = m_plate_friction;

    // Set the initial state and specified mode
    m_mode = mode;
    m_state = (mode == Mode::SUSPEND) ? State::FIXED : State::DROPPING;

    cout << "Mode: " << ModeName(m_mode) << endl;

    // Create output directory (if needed)
    if (m_output) {
        m_outdir = m_outdir + m_tire->GetName() + "/";
        if (!filesystem::create_directory(filesystem::path(m_outdir))) {
            cerr << "Error creating directory. Disabling output " << m_outdir << endl;
            m_output = false;
        }
    }
    if (m_output) {
        m_outdir = m_outdir + ModeName(m_mode) + "/";
        if (!filesystem::create_directory(filesystem::path(m_outdir))) {
            cerr << "Error creating directory. Disabling output " << m_outdir << endl;
            m_output = false;
        }
    }
    if (m_output) {
        m_csv.SetDelimiter(" ");
    }
}

// -----------------------------------------------------------------------------

bool ChTireStaticTestRig::Advance(double step) {
    double time = m_system->GetChTime();

    UpdateActuators(time);
    StateTransition(time);
    Output(time);

    if (m_state == State::DONE)
        return false;

    // Synchronize subsystems
    m_terrain.Synchronize(time);
    m_tire->Synchronize(time, m_terrain);
    m_spindle_body->EmptyAccumulators();
    m_wheel->Synchronize();

    // Advance state
    m_terrain.Advance(step);
    m_tire->Advance(step);
    m_system->DoStepDynamics(step);

    return true;
}

// -----------------------------------------------------------------------------

void ChTireStaticTestRig::UpdateActuators(double time) {
    double r_speed = 0;
    double x_speed = 0;
    double y_speed = 0;
    double z_speed = 0;

    switch (m_state) {
        case State::FIXED:
            m_motor_r->SetDisabled(true);
            break;
        case State::DROPPING:
            r_speed = 0.1;
            break;
        case State::COMPRESSING:
            r_speed = m_r_speed;
            break;
        case State::DISPLACING:
            switch (m_mode) {
                case Mode::TEST_X:
                    x_speed = m_x_speed;
                    break;
                case Mode::TEST_Y:
                    y_speed = m_y_speed;
                    break;
                case Mode::TEST_Z:
                    z_speed = m_z_speed;
                    break;
                default:
                    cerr << "Incorrect state for the requested operation mode" << endl;
                    break;
            }
            break;
    }

    {
        auto func = std::static_pointer_cast<ChFunctionSetpoint>(m_motor_r->GetMotorFunction());
        func->SetSetpointAndDerivatives(10 * r_speed, 0.0, 0.0);
    }
    {
        auto func = std::static_pointer_cast<ChFunctionSetpoint>(m_motor_x->GetMotorFunction());
        func->SetSetpointAndDerivatives(x_speed, 0.0, 0.0);
    }
    {
        auto func = std::static_pointer_cast<ChFunctionSetpoint>(m_motor_y->GetMotorFunction());
        func->SetSetpointAndDerivatives(y_speed, 0.0, 0.0);
    }
    {
        auto func = std::static_pointer_cast<ChFunctionSetpoint>(m_motor_z->GetMotorFunction());
        func->SetSetpointAndDerivatives(z_speed, 0.0, 0.0);
    }
}

void ChTireStaticTestRig::StateTransition(double time) {
    ////if (time < m_transition_time + m_transition_delay)
    ////    return;

    State new_state = m_state;

    switch (m_state) {
        case State::FIXED: {
            assert(m_mode == Mode::SUSPEND);
            break;
        }
        case State::DROPPING: {
            // Switch to compression phase when contact occurs and cache current spindle z
            if (m_system->GetNumContacts() > 0) {
                m_spindle_z_ref = m_spindle_body->GetPos().z();
                new_state = State::COMPRESSING;
                cout << "\nt = " << time << endl;
                cout << "  num. contacts:    " << m_system->GetNumContacts() << endl;
                cout << "  reference height: " << m_spindle_z_ref << endl;
                cout << "  switch to State::COMPRESSING (nominal_load = " << m_r_load << ")" << endl;
            }
            break;
        }
        case State::COMPRESSING: {
            auto current_load = m_motor_r->GetMotorForce();

            // End compression phase when reaching the prescribed radial load
            if (current_load > m_r_load) {
                if (m_mode == Mode::TEST_R)
                    new_state = State::DONE;
                else
                    new_state = State::DISPLACING;
                cout << "\nt = " << time << endl;
                cout << "  radial load:  " << current_load;
                cout << "  nominal load: " << m_r_load << endl;
                cout << "  switch to state: " << StateName(new_state) << endl;
            }
            break;
        }
        case State::DISPLACING: {
            // End displacement testing phase when relative motion tire-plate small enough
            //// TODO

            break;
        }
    }

    if (new_state != m_state) {
        m_state = new_state;
        m_transition_time = time;
    }
}

// -----------------------------------------------------------------------------

void ChTireStaticTestRig::Output(double time) {
    if (!m_output)
        return;

    if (m_mode == Mode::SUSPEND)
        return;

    if (m_state == State::FIXED || m_state == State::DROPPING)
        return;

    if (m_state == State::COMPRESSING && m_mode != Mode::TEST_R)
        return;

    // Write current outputs (in mm, deg, N, Nm)
    m_csv << time;

    switch (m_mode) {
        case Mode::TEST_R: {
            m_csv << 1e-3 * (m_spindle_z_ref - m_spindle_body->GetPos().z()) << m_motor_r->GetMotorForce();
            break;
        }
        case Mode::TEST_X: {
            m_csv << 1e-3 * std::abs(m_plate_body->GetPos().x()) << m_motor_x->GetMotorForce();
            break;
        }
        case Mode::TEST_Y: {
            m_csv << 1e-3 * std::abs(m_plate_body->GetPos().y()) << m_motor_y->GetMotorForce();
            break;
        }
        case Mode::TEST_Z: {
            const auto& angles = AngleSetFromQuat(RotRepresentation::CARDAN_ANGLES_ZXY, m_plate_body->GetRot()).angles;
            m_csv << CH_RAD_TO_DEG * std::abs(angles.x()) << m_motor_z->GetMotorTorque();
            break;
        }
    }

    m_csv << endl;
}

void ChTireStaticTestRig::WriteOutput() {
    std::string out_file = m_outdir + "results.txt";

    // Write output file
    std::string header;
    std::string x_label;
    std::string y_label;
    switch (m_mode) {
        case Mode::TEST_R:
            header = "time, displ (mm), load (N)";
            x_label = "Deflection (mm)";
            y_label = "Radial load (N)";
            break;
        case Mode::TEST_X:
            header = "time, displ (mm), load (N)";
            x_label = "Deflection (mm)";
            y_label = "Longitudinal load (N)";
            break;
        case Mode::TEST_Y:
            header = "time, displ (mm), load (N)";
            x_label = "Deflection (mm)";
            y_label = "Lateral load (N)";
            break;
        case Mode::TEST_Z:
            header = "time, displ (deg), load (Nm)";
            x_label = "Deflection (deg)";
            y_label = "Torsional load (Nm)";
            break;
    }
    m_csv.WriteToFile(out_file);

    cout << "Output written to " << std::quoted(out_file) << endl;

    // Plot results
    if (!m_gnuplot)
        return;

#ifndef CHRONO_POSTPROCESS
    cout << "ERROR: GnuPlot not available. Enable the Chrono::Postprocess module." << endl;
    return;
#else
    std::string gplfile = m_outdir + "/plot.gpl";
    postprocess::ChGnuPlot mplot(gplfile);

    std::string title = "Tire test rig - " + ModeName(m_mode);
    mplot.OutputWindow(0);
    mplot.SetTitle(title);

    mplot.SetLabelX(x_label);
    mplot.SetLabelY(y_label);
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.SetCommand("set yrange[0:]");
    mplot.Plot(out_file, 2, 3, "", " with lines lw 2");

#endif
}

// -----------------------------------------------------------------------------

std::string ChTireStaticTestRig::ModeName(Mode mode) {
    switch (mode) {
        case Mode::SUSPEND:
            return "Suspend";
        case Mode::TEST_R:
            return "Radial_test";
        case Mode::TEST_X:
            return "Longitudinal_test";
        case Mode::TEST_Y:
            return "Lateral_test";
        case Mode::TEST_Z:
            return "Torsional_test";
    }
    return "Unknown mode";
}

std::string ChTireStaticTestRig::StateName(State state) {
    switch (state) {
        case State::FIXED:
            return "Fixed";
        case State::DROPPING:
            return "Dropping";
        case State::COMPRESSING:
            return "Compressing";
        case State::DISPLACING:
            return "Displacing";
        case State::DONE:
            return "Done";
        default:
            break;
    }
    return "Unknown state";
}

// -----------------------------------------------------------------------------

std::string ChTireStaticTestRig::GetStateName() const {
    return StateName(m_state);
}

double ChTireStaticTestRig::GetCompressionLoad() const {
    return m_motor_r->GetMotorForce();
}

double ChTireStaticTestRig::GetLongitudinalLoad() const {
    return m_motor_x->GetMotorForce();
}

double ChTireStaticTestRig::GetLateralLoad() const {
    return m_motor_y->GetMotorForce();
}

double ChTireStaticTestRig::GetTorsionalLoad() const {
    return m_motor_z->GetMotorTorque();
}

double ChTireStaticTestRig::GetLoad() const {
    switch (m_mode) {
        case Mode::TEST_R:
            return GetCompressionLoad();
        case Mode::TEST_X:
            return GetLongitudinalLoad();
        case Mode::TEST_Y:
            return GetLateralLoad();
        case Mode::TEST_Z:
            return GetTorsionalLoad();
    }
    return 0;
}

ChVector3d ChTireStaticTestRig::GetWheelPos() const {
    return m_wheel->GetPos();
}

TerrainForce ChTireStaticTestRig::ReportTireForce() {
    return m_tire->ReportTireForce(&m_terrain);
}

double ChTireStaticTestRig::GetRadialLoad() const {
    return m_motor_r->GetMotorForce();
}

}  // end namespace vehicle
}  // end namespace chrono
