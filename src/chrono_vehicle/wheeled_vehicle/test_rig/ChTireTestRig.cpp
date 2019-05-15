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
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

namespace chrono {
namespace vehicle {

ChTireTestRig::ChTireTestRig(std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, ChSystem* system)
    : m_system(system),
      m_grav(9.8),
      m_wheel(wheel),
      m_tire(tire),
      m_camber_angle(0),
      m_normal_load(0),
      m_applied_load(0),
      m_total_mass(0),
      m_time_delay(0),
      m_ls_actuated(false),
      m_rs_actuated(false),
      m_terrain_type(TerrainType::NONE),
      m_terrain_offset(0),
      m_terrain_height(0),
      m_tire_step(1e-3),
      m_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_vis(VisualizationType::PRIMITIVES) {
    // Default motion function for slip angle control
    m_sa_fun = std::make_shared<ChFunction_Const>(0);
}

// -----------------------------------------------------------------------------

void ChTireTestRig::SetLongSpeedFunction(std::shared_ptr<ChFunction> funct) { 
    m_ls_fun = funct; 
    m_ls_actuated = true;
}

void ChTireTestRig::SetAngSpeedFunction(std::shared_ptr<ChFunction> funct) { 
    m_rs_fun = funct; 
    m_rs_actuated = true;
}

// -----------------------------------------------------------------------------

void ChTireTestRig::SetTerrainSCM(double Bekker_Kphi,
                                  double Bekker_Kc,
                                  double Bekker_n,
                                  double Mohr_cohesion,
                                  double Mohr_friction,
                                  double Janosi_shear,
                                  double terrain_length) {
    m_terrain_type = TerrainType::SCM;

    m_params_SCM.Bekker_Kphi = Bekker_Kphi;
    m_params_SCM.Bekker_Kc = Bekker_Kc;
    m_params_SCM.Bekker_n = Bekker_n;
    m_params_SCM.Mohr_cohesion = Mohr_cohesion;
    m_params_SCM.Mohr_friction = Mohr_friction;
    m_params_SCM.Janosi_shear = Janosi_shear;

    m_params_SCM.length = terrain_length;
    m_params_SCM.width = 1;  //// TODO: m_tire->GetWidth();
}

void ChTireTestRig::SetTerrainRigid(double friction, double restitution, double Young_modulus, double terrain_length) {
    m_terrain_type = TerrainType::RIGID;

    m_params_rigid.friction = (float)friction;
    m_params_rigid.restitution = (float)restitution;
    m_params_rigid.Young_modulus = (float)Young_modulus;

    m_params_rigid.length = terrain_length;
    m_params_rigid.width = 1;  //// TODO: m_tire->GetWidth();
}

void ChTireTestRig::SetTerrainGranular(double radius,
                                       unsigned int num_layers,
                                       double density,
                                       double friction,
                                       double cohesion,
                                       double Young_modulus) {
    m_terrain_type = TerrainType::GRANULAR;

    m_params_granular.radius = radius;
    m_params_granular.num_layers = num_layers;
    m_params_granular.density = density;
    m_params_granular.friction = friction;
    m_params_granular.cohesion = cohesion;
    m_params_granular.Young_modulus = Young_modulus;

    m_params_granular.length = 5 * m_tire->GetRadius();
    m_params_granular.width = 1;  //// TODO: m_tire->GetWidth();
}

// -----------------------------------------------------------------------------

void ChTireTestRig::Initialize() {
    CreateMechanism();

    if (m_ls_actuated)
        m_lin_motor->SetSpeedFunction(m_ls_fun);

    if (m_rs_actuated)
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
    m_ls_actuated = true;
    m_rs_actuated = true;

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

    // Apply load on chassis body
    double external_force = m_total_mass * m_grav;
    if (time > m_time_delay)
        external_force = m_applied_load;

    m_chassis_body->Empty_forces_accumulators();
    m_chassis_body->Accumulate_force(ChVector<>(0, 0, external_force), ChVector<>(0, 0, 0), true);

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
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetIdentifier(0);
    m_ground_body->SetBodyFixed(true);
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(100, dim / 3, dim / 3));
        m_ground_body->AddAsset(box);
    }

    m_carrier_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_carrier_body);
    m_carrier_body->SetName("rig_carrier");
    m_carrier_body->SetIdentifier(1);
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
    m_chassis_body->SetName("rig_chassis");
    m_chassis_body->SetIdentifier(2);
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
    m_slip_body->SetName("rig_slip");
    m_slip_body->SetIdentifier(3);
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
    m_spindle_body->SetName("rig_spindle");
    m_spindle_body->SetIdentifier(4);
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
    if (m_ls_actuated) {
        m_lin_motor = std::make_shared<ChLinkMotorLinearSpeed>();
        m_system->AddLink(m_lin_motor);
        m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    } else {
        ChQuaternion<> z2x;
        z2x.Q_from_AngY(CH_C_PI_2);
        auto prismatic = std::make_shared<ChLinkLockPrismatic>();
        m_system->AddLink(prismatic);
        prismatic->Initialize(m_carrier_body, m_ground_body, ChCoordsys<>(VNULL, z2x));
    }

    auto prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_system->AddLink(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChCoordsys<>(VNULL, QUNIT));

    m_slip_lock = std::make_shared<ChLinkLockLock>();
    m_system->AddLink(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChCoordsys<>(VNULL, QUNIT));
    m_slip_lock->SetMotion_axis(ChVector<>(0, 0, 1));

    ChQuaternion<> z2y;
    z2y.Q_from_AngAxis(-CH_C_PI / 2 - m_camber_angle, ChVector<>(1, 0, 0));
    if (m_rs_actuated) {
        m_rot_motor = std::make_shared<ChLinkMotorRotationSpeed>();
        m_system->AddLink(m_rot_motor);
        m_rot_motor->Initialize(m_spindle_body, m_slip_body, ChFrame<>(ChVector<>(0, 3 * dim, -4 * dim), z2y));
    } else {
        auto revolute = std::make_shared<ChLinkLockRevolute>();
        m_system->AddLink(revolute);
        revolute->Initialize(m_spindle_body, m_slip_body, ChCoordsys<>(ChVector<>(0, 3 * dim, -4 * dim), z2y));
    }

    // Calculate required body force on chassis (to enforce given normal load)
    m_total_mass = m_chassis_body->GetMass() + m_slip_body->GetMass() + m_spindle_body->GetMass() +
                        m_wheel->GetMass() + m_tire->GetMass();
    m_applied_load = m_total_mass * m_grav - m_normal_load;

    // Approach using ChLoad does not work with Chrono::Parallel (loads currently not supported).
    // Instead use a force accumulator (updated in ChTireTestRig::Advance)
    /*
    auto load = std::make_shared<ChLoadBodyForce>(m_chassis_body, ChVector<>(0, 0, m_applied_load), false, VNULL, true);
    auto load_container = std::make_shared<ChLoadContainer>();
    load_container->Add(load);
    m_system->Add(load_container);
    */

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
        case TerrainType::GRANULAR:
            CreateTerrainGranular();
            break;
        default:
            break;
    }
}

void ChTireTestRig::CreateTerrainSCM() {
    ChVector<> location(m_params_SCM.length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, 0);

    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Mesh divisions
    double factor = 8;  // Initial number of divisions per unit (m)
    int ndivX = (int)std::ceil(m_params_SCM.length * factor);
    int ndivY = (int)std::ceil(m_params_SCM.width * factor);

    auto terrain = std::make_shared<vehicle::SCMDeformableTerrain>(m_system);
    terrain->SetPlane(ChCoordsys<>(location, Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParametersSCM(m_params_SCM.Bekker_Kphi, m_params_SCM.Bekker_Kc, m_params_SCM.Bekker_n,            //
                                  m_params_SCM.Mohr_cohesion, m_params_SCM.Mohr_friction, m_params_SCM.Janosi_shear,  //
                                  E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.05);
    terrain->SetAutomaticRefinement(true);
    terrain->SetAutomaticRefinementResolution(1.0 / 32);
    terrain->Initialize(m_terrain_height, m_params_SCM.length, m_params_SCM.width, ndivX, ndivY);
    terrain->EnableMovingPatch(m_chassis_body, ChVector<>(0, 0, 0), 2 * m_tire->GetRadius(), 1.0);

    m_terrain = terrain;
}

void ChTireTestRig::CreateTerrainRigid() {
    ChVector<> location(m_params_rigid.length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, m_terrain_height - 0.1);

    auto terrain = std::make_shared<vehicle::RigidTerrain>(m_system);
    auto patch =
        terrain->AddPatch(ChCoordsys<>(location, QUNIT), ChVector<>(m_params_rigid.length, m_params_rigid.width, 0.1));
    patch->SetContactFrictionCoefficient(m_params_rigid.friction);
    patch->SetContactRestitutionCoefficient(m_params_rigid.restitution);
    patch->SetContactMaterialProperties(m_params_rigid.Young_modulus, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(GetChronoDataFile("pinkwhite.png"), 10 * (float)m_params_rigid.length,
                      10 * (float)m_params_rigid.width);
    terrain->Initialize();

    m_terrain = terrain;
}

void ChTireTestRig::CreateTerrainGranular() {
    double vertical_offset = m_params_granular.num_layers * (2 * m_params_granular.radius);
    ChVector<> location(0, m_terrain_offset, m_terrain_height - vertical_offset);

    auto terrain = std::make_shared<vehicle::GranularTerrain>(m_system);

    double coh_force = (CH_C_PI * m_params_granular.radius * m_params_granular.radius) * m_params_granular.cohesion;
    switch (m_system->GetContactMethod()) {
        case ChMaterialSurface::SMC: {
            auto mat_g = std::make_shared<ChMaterialSurfaceSMC>();
            mat_g->SetFriction(static_cast<float>(m_params_granular.friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetYoungModulus(8e5f);
            mat_g->SetPoissonRatio(0.3f);
            mat_g->SetAdhesion(static_cast<float>(coh_force));
            mat_g->SetKn(1.0e6f);
            mat_g->SetGn(6.0e1f);
            mat_g->SetKt(4.0e5f);
            mat_g->SetGt(4.0e1f);
            terrain->SetContactMaterialSMC(std::static_pointer_cast<ChMaterialSurfaceSMC>(mat_g));
            break;
        }
        case ChMaterialSurface::NSC: {
            double step_size = 1e-3; ///< estimate for integration step size
            auto mat_g = std::make_shared<ChMaterialSurfaceNSC>();
            mat_g->SetFriction(static_cast<float>(m_params_granular.friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetCohesion(static_cast<float>(coh_force * step_size));
            terrain->SetContactMaterialNSC(std::static_pointer_cast<ChMaterialSurfaceNSC>(mat_g));
            terrain->SetCollisionEnvelope(0.05 * m_params_granular.radius);
            break;
        }
    }

    terrain->SetStartIdentifier(1000000);
    ////terrain->EnableVisualization(true);
    terrain->EnableVerbose(true);
    terrain->Initialize(location, m_params_granular.length, m_params_granular.width, m_params_granular.num_layers,
                        m_params_granular.radius, m_params_granular.density);

    double buffer_dist = 2.0 * m_tire->GetRadius();
    double shift_dist = 0.5 * m_tire->GetRadius();
    terrain->EnableMovingPatch(m_spindle_body, buffer_dist, shift_dist, ChVector<>(0, 0, -2));

    m_terrain = terrain;
}

// -----------------------------------------------------------------------------

void ChTireTestRig::GetSuggestedCollisionSettings(double& collision_envelope, ChVector<int>& collision_bins) const {
    if (m_terrain_type != TerrainType::GRANULAR) {
        collision_envelope = 0.01;
        collision_bins = ChVector<int>(1, 1, 1);
        return;
    }

    collision_envelope = 0.05 * m_params_granular.radius;

    int factor = 2;
    collision_bins.x() = (int)std::ceil((0.5 * m_params_granular.length) / m_params_granular.radius) / factor;
    collision_bins.y() = (int)std::ceil((0.5 * m_params_granular.width) / m_params_granular.radius) / factor;
    collision_bins.z() = 1;
}

// -----------------------------------------------------------------------------

TerrainForce ChTireTestRig::GetTireForce() const {
    return m_tire->ReportTireForce(m_terrain.get());
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // end namespace chrono
