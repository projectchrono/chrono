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
// Implementation of a single-tire test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

namespace chrono {
namespace vehicle {

ChTireTestRig::ChTireTestRig(std::shared_ptr<ChWheel> wheel,
                             std::shared_ptr<ChTire> tire,
                             ChMaterialSurface::ContactMethod contact_method)
    : m_grav(9.8),
      m_wheel(wheel),
      m_tire(tire),
      m_camber_angle(0),
      m_normal_load(0),
      m_terrain_type(TerrainType::NONE),
      m_terrain_length(10),
      m_terrain_offset(0),
      m_terrain_height(0),
      m_tire_step(1e-3),
      m_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_vis(VisualizationType::PRIMITIVES) {
    m_system = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                          : static_cast<ChSystem*>(new ChSystemSMC);

    // Integration and Solver settings
    m_system->SetMaxItersSolverSpeed(150);
    m_system->SetMaxItersSolverStab(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Default motion functions
    m_ls_fun = std::make_shared<ChFunction_Const>(0);
    m_rs_fun = std::make_shared<ChFunction_Const>(0);
    m_sa_fun = std::make_shared<ChFunction_Const>(0);
}

// -----------------------------------------------------------------------------

void ChTireTestRig::SetTerrainSCM(double Bekker_Kphi,
                                  double Bekker_Kc,
                                  double Bekker_n,
                                  double Mohr_cohesion,
                                  double Mohr_friction,
                                  double Janosi_shear) {
    m_terrain_type = TerrainType::SCM;

    m_terrain_paramsSCM.Bekker_Kphi = Bekker_Kphi;
    m_terrain_paramsSCM.Bekker_Kc = Bekker_Kc;
    m_terrain_paramsSCM.Bekker_n = Bekker_n;
    m_terrain_paramsSCM.Mohr_cohesion = Mohr_cohesion;
    m_terrain_paramsSCM.Mohr_friction = Mohr_friction;
    m_terrain_paramsSCM.Janosi_shear = Janosi_shear;
}

void ChTireTestRig::SetTerrainRigid(double friction, double Y, double cr) {
    m_terrain_type = TerrainType::RIGID;

    m_terrain_paramsRigid.friction = (float)friction;
    m_terrain_paramsRigid.Y = (float)Y;
    m_terrain_paramsRigid.cr = (float)cr;
}

// -----------------------------------------------------------------------------

void ChTireTestRig::Initialize() {
    CreateMechanism();

    m_lin_motor->SetSpeedFunction(m_ls_fun);
    m_rot_motor->SetSpeedFunction(m_rs_fun);
    m_slip_lock->SetMotion_ang(m_sa_fun);

    CreateTerrain();
}

// -----------------------------------------------------------------------------

class BaseFunction {
  protected:
    BaseFunction(double speed) : m_speed(speed) {}
    double calc(double t) const {
        double delay = 0.25;
        double ramp = 0.5;
        if (t <= delay)
            return 0;
        double tt = t - delay;
        if (tt >= ramp)
            return m_speed;
        return m_speed * tt / ramp;
    }
    double m_speed;
};

class LinSpeedFunction : public BaseFunction, public ChFunction {
  public:
    LinSpeedFunction(double speed) : BaseFunction(speed) {}
    virtual double Get_y(double t) const override { return calc(t); }
    virtual LinSpeedFunction* Clone() const override { return new LinSpeedFunction(*this); }
};

class RotSpeedFunction : public BaseFunction, public ChFunction {
  public:
    RotSpeedFunction(double slip, double speed, double radius) : BaseFunction(speed), m_slip(slip), m_radius(radius) {}
    virtual double Get_y(double t) const override {
        double v = calc(t);
        return (1 + m_slip) * v / m_radius;
    }
    virtual RotSpeedFunction* Clone() const override { return new RotSpeedFunction(*this); }

    double m_slip;
    double m_radius;
};

void ChTireTestRig::Initialize(double long_slip, double base_speed) {
    CreateMechanism();

    m_ls_fun = std::make_shared<LinSpeedFunction>(base_speed);
    m_rs_fun = std::make_shared<RotSpeedFunction>(long_slip, base_speed, m_tire->GetRadius());

    m_lin_motor->SetSpeedFunction(m_ls_fun);
    m_rot_motor->SetSpeedFunction(m_rs_fun);
    m_slip_lock->SetMotion_ang(m_sa_fun);

    CreateTerrain();
}

// -----------------------------------------------------------------------------

void ChTireTestRig::Advance(double step) {
    double time = m_system->GetChTime();

    // Synchronize subsystems
    auto tire_force = m_tire->GetTireForce();

    WheelState wheel_state;
    wheel_state.pos = m_spindle_body->GetPos();
    wheel_state.rot = m_spindle_body->GetRot();
    wheel_state.lin_vel = m_spindle_body->GetPos_dt();
    wheel_state.ang_vel = m_spindle_body->GetWvel_par();
    ChVector<> ang_vel_loc = wheel_state.rot.RotateBack(wheel_state.ang_vel);
    wheel_state.omega = ang_vel_loc.y();

    m_terrain->Synchronize(time);
    m_tire->Synchronize(time, wheel_state, *m_terrain.get(), m_collision_type);
    m_spindle_body->Empty_forces_accumulators();
    m_spindle_body->Accumulate_force(tire_force.force, tire_force.point, false);
    m_spindle_body->Accumulate_torque(tire_force.moment, false);

    // Advance state
    m_terrain->Advance(step);
    m_tire->Advance(step);
    m_system->DoStepDynamics(step);
}

// -----------------------------------------------------------------------------

void ChTireTestRig::CreateMechanism() {
    m_system->Set_G_acc(ChVector<>(0, 0, -m_grav));

    // Create bodies
    const double dim = 0.1;

    m_ground_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_ground_body);
    m_ground_body->SetBodyFixed(true);
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(100, dim / 3, dim / 3));
        m_ground_body->AddAsset(box);
    }

    m_carrier_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_carrier_body);
    m_carrier_body->SetPos(ChVector<>(0, 0, 0));
    m_carrier_body->SetMass(m_wheel->GetMass());
    m_carrier_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(+2 * dim, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(-2 * dim, 0, 0);
        m_carrier_body->AddAsset(cyl);

        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(dim / 3, dim / 3, 10 * dim));
        box->Pos = ChVector<>(0, 0, -5 * dim);
        m_carrier_body->AddAsset(box);

        m_carrier_body->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.2f, 0.2f));
    }

    m_chassis_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_chassis_body);
    m_chassis_body->SetPos(ChVector<>(0, 0, 0));
    m_chassis_body->SetMass(m_wheel->GetMass());
    m_chassis_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto sphere = std::make_shared<ChSphereShape>();
        sphere->GetSphereGeometry().rad = dim;
        m_chassis_body->AddAsset(sphere);

        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * dim);
        m_chassis_body->AddAsset(cyl);

        m_chassis_body->AddAsset(std::make_shared<ChColorAsset>(0.2f, 0.8f, 0.2f));
    }

    m_slip_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_slip_body);
    m_slip_body->SetPos(ChVector<>(0, 0, -4 * dim));
    m_slip_body->SetMass(m_wheel->GetMass());
    m_slip_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(4 * dim, dim, 4 * dim));
        m_slip_body->AddAsset(box);

        m_slip_body->AddAsset(std::make_shared<ChColorAsset>(0.2f, 0.2f, 0.8f));
    }

    ChQuaternion<> qc;
    qc.Q_from_AngX(-m_camber_angle);
    m_spindle_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_spindle_body);
    m_spindle_body->SetMass(1);
    m_spindle_body->SetInertiaXX(ChVector<>(0.01, 0.01, 0.01));
    m_spindle_body->SetPos(ChVector<>(0, 3 * dim, -4 * dim));
    m_spindle_body->SetRot(qc);
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -3 * dim, 0);
        m_spindle_body->AddAsset(cyl);
    }

    // Create joints and motors
    m_lin_motor = std::make_shared<ChLinkMotorLinearSpeed>();
    m_system->AddLink(m_lin_motor);
    m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));

    auto prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_system->AddLink(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChCoordsys<>(VNULL, QUNIT));

    m_slip_lock = std::make_shared<ChLinkLockLock>();
    m_system->AddLink(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChCoordsys<>(VNULL, QUNIT));
    m_slip_lock->SetMotion_axis(ChVector<>(0, 0, 1));

    ChQuaternion<> z2y;
    z2y.Q_from_AngAxis(-CH_C_PI / 2 - m_camber_angle, ChVector<>(1, 0, 0));
    m_rot_motor = std::make_shared<ChLinkMotorRotationSpeed>();
    m_system->AddLink(m_rot_motor);
    m_rot_motor->Initialize(m_spindle_body, m_slip_body, ChFrame<>(ChVector<>(0, 3 * dim, -4 * dim), z2y));

    // Calculate required body force on chassis (to enforce given normal load)
    double total_mass = m_chassis_body->GetMass() + m_slip_body->GetMass() + m_spindle_body->GetMass() +
                        m_wheel->GetMass() + m_tire->GetMass();
    double actual_load = m_normal_load - total_mass * m_grav;
    auto load = std::make_shared<ChLoadBodyForce>(m_chassis_body, ChVector<>(0, 0, -actual_load), false, VNULL, true);
    auto load_container = std::make_shared<ChLoadContainer>();
    load_container->Add(load);
    m_system->Add(load_container);

    // Initialize subsystems
    m_wheel->Initialize(m_spindle_body);
    m_wheel->SetVisualizationType(VisualizationType::NONE);
    m_tire->SetStepsize(m_tire_step);
    m_tire->Initialize(m_spindle_body, LEFT);
    m_tire->SetVisualizationType(m_tire_vis);

    // Set terrain offset (based on wheel center) and terrain height (below tire)
    m_terrain_offset = 3 * dim;
    m_terrain_height = -4 * dim - m_tire->GetRadius() - 0.1;
}

// -----------------------------------------------------------------------------

void ChTireTestRig::CreateTerrain() {
    switch (m_terrain_type) {
        case TerrainType::SCM:
            CreateTerrainSCM();
            break;
        case TerrainType::RIGID:
            CreateTerrainRigid();
            break;
        default:
            break;
    }
}

void ChTireTestRig::CreateTerrainSCM() {
    double terrain_width = 1;  //// TODO: m_tire->GetWidth();

    ChVector<> location(m_terrain_length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, 0);

    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Mesh divisions
    double factor = 8;  // Initial number of divisions per unit (m)
    int ndivX = (int)std::ceil(m_terrain_length * factor);
    int ndivY = (int)std::ceil(terrain_width * factor);

    auto terrain = std::make_shared<vehicle::SCMDeformableTerrain>(m_system);
    terrain->SetPlane(ChCoordsys<>(location, Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParametersSCM(
        m_terrain_paramsSCM.Bekker_Kphi, m_terrain_paramsSCM.Bekker_Kc, m_terrain_paramsSCM.Bekker_n,            //
        m_terrain_paramsSCM.Mohr_cohesion, m_terrain_paramsSCM.Mohr_friction, m_terrain_paramsSCM.Janosi_shear,  //
        E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain->SetAutomaticRefinement(true);
    terrain->SetAutomaticRefinementResolution(1.0 / 32);
    terrain->Initialize(m_terrain_height, m_terrain_length, terrain_width, ndivX, ndivY);
    terrain->EnableMovingPatch(m_chassis_body, ChVector<>(0, 0, 0), 2 * m_tire->GetRadius(), 1.0);

    m_terrain = terrain;
}

void ChTireTestRig::CreateTerrainRigid() {
    double terrain_width = 1; //// TODO: m_tire->GetWidth();

    ChVector<> location(m_terrain_length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, m_terrain_height - 0.1);

    auto terrain = std::make_shared<vehicle::RigidTerrain>(m_system);
    auto patch = terrain->AddPatch(ChCoordsys<>(location, QUNIT), ChVector<>(m_terrain_length, terrain_width, 0.1));
    patch->SetContactFrictionCoefficient(m_terrain_paramsRigid.friction);
    patch->SetContactRestitutionCoefficient(m_terrain_paramsRigid.cr);
    patch->SetContactMaterialProperties(m_terrain_paramsRigid.Y, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(GetChronoDataFile("pinkwhite.png"), 10 * (float)m_terrain_length, 10 * (float)terrain_width);
    terrain->Initialize();

    m_terrain = terrain;
}

// -----------------------------------------------------------------------------

TerrainForce ChTireTestRig::GetTireForce() const {
    if (m_terrain_type == TerrainType::RIGID)
        return m_tire->ReportTireForce(nullptr);

    return m_tire->ReportTireForce(m_terrain.get());
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // end namespace chrono
