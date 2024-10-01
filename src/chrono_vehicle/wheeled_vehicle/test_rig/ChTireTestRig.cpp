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

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChTireTestRig::ChTireTestRig(std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, ChSystem* system)
    : m_system(system),
      m_grav(9.8),
      m_wheel(wheel),
      m_tire(tire),
      m_camber_angle(0),
      m_normal_load(1000),
      m_total_mass(0),
      m_time_delay(0),
      m_ls_actuated(false),
      m_rs_actuated(false),
      m_long_slip_constant(false),
      m_terrain_type(TerrainType::NONE),
      m_terrain_offset(0),
      m_terrain_height(0),
      m_tire_step(1e-3),
      m_tire_vis(VisualizationType::PRIMITIVES) {
    // Default motion function for slip angle control
    m_sa_fun = chrono_types::make_shared<ChFunctionConst>(0);
    // Default tire-terrain collision method
    m_tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
}

ChTireTestRig::~ChTireTestRig() {
    auto sys = m_ground_body->GetSystem();
    if (sys) {
        sys->Remove(m_ground_body);
        sys->Remove(m_carrier_body);
        sys->Remove(m_chassis_body);
        sys->Remove(m_slip_body);
        sys->Remove(m_spindle_body);
        sys->Remove(m_lin_motor);
        sys->Remove(m_rot_motor);
        sys->Remove(m_slip_lock);
    }
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

void ChTireTestRig::SetConstantLongitudinalSlip(double long_slip, double base_speed) {
    m_ls_actuated = true;
    m_rs_actuated = true;
    m_long_slip_constant = true;
    m_long_slip = long_slip;
    m_base_speed = base_speed;
}

void ChTireTestRig::SetTireCollisionType(ChTire::CollisionType coll_type) {
    m_tire->SetCollisionType(coll_type);
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
    virtual double GetVal(double t) const override { return calc(t); }
    virtual LinSpeedFunction* Clone() const override { return new LinSpeedFunction(*this); }
};

class RotSpeedFunction : public BaseFunction, public ChFunction {
  public:
    RotSpeedFunction(double slip, double speed, double radius) : BaseFunction(speed), m_slip(slip), m_radius(radius) {}
    virtual double GetVal(double t) const override {
        double v = calc(t);
        return (1 + m_slip) * v / m_radius;
    }
    virtual RotSpeedFunction* Clone() const override { return new RotSpeedFunction(*this); }

    double m_slip;
    double m_radius;
};

void ChTireTestRig::Initialize(Mode mode) {
    CreateMechanism(mode);
    CreateTerrain();

    if (mode != Mode::TEST)
        return;

    if (m_long_slip_constant) {
        // Override motion functions to enforce specified constant longitudinal slip
        m_ls_fun = chrono_types::make_shared<LinSpeedFunction>(m_base_speed);
        m_rs_fun = chrono_types::make_shared<RotSpeedFunction>(m_long_slip, m_base_speed, m_tire->GetRadius());
    }

    struct DelayedFun : public ChFunction {
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

    if (m_ls_actuated)
        m_lin_motor->SetSpeedFunction(chrono_types::make_shared<DelayedFun>(m_ls_fun, m_time_delay));

    if (m_rs_actuated)
        m_rot_motor->SetSpeedFunction(chrono_types::make_shared<DelayedFun>(m_rs_fun, m_time_delay));

    m_slip_lock->SetMotionAng1(chrono_types::make_shared<DelayedFun>(m_sa_fun, m_time_delay));
}

// -----------------------------------------------------------------------------

void ChTireTestRig::Advance(double step) {
    double time = m_system->GetChTime();

    // Synchronize subsystems
    m_terrain->Synchronize(time);
    m_tire->Synchronize(time, *m_terrain.get());
    m_spindle_body->EmptyAccumulators();
    m_wheel->Synchronize();

    // Advance state
    m_terrain->Advance(step);
    m_tire->Advance(step);
    m_system->DoStepDynamics(step);
}

// -----------------------------------------------------------------------------

void ChTireTestRig::CreateMechanism(Mode mode) {
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, -m_grav));

    // Create bodies.
    // Rig bodies are constructed with mass and inertia commensurate with those of the wheel-tire system.
    // The spindle body is constructed with zero mass and inertia (these will be increased by at least the wheel mass
    // and inertia).
    const double dim = 0.1;
    const double mass = m_wheel->GetWheelMass() + m_tire->GetTireMass();
    const ChVector3d inertia = m_wheel->GetWheelInertia() + m_tire->GetTireInertia();

    m_ground_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_ground_body);
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetFixed(true);
    {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(100, dim / 3, dim / 3);
        m_ground_body->AddVisualShape(box);
    }

    m_carrier_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_carrier_body);
    m_carrier_body->SetName("rig_carrier");
    m_carrier_body->SetPos(ChVector3d(0, 0, 0));
    m_carrier_body->SetMass(mass);
    m_carrier_body->SetInertiaXX(inertia);
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.8f, 0.2f, 0.2f});

        utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier_body,              //
                                                        ChVector3d(+2 * dim, 0, 0),  //
                                                        ChVector3d(-2 * dim, 0, 0),  //
                                                        dim / 2,                     //
                                                        mat);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(dim / 3, dim / 3, 10 * dim);
        box->AddMaterial(mat);
        m_carrier_body->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -5 * dim)));
    }

    m_chassis_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_chassis_body);
    m_chassis_body->SetName("rig_chassis");
    m_chassis_body->SetPos(ChVector3d(0, 0, 0));
    m_chassis_body->SetMass(mass);
    m_chassis_body->SetInertiaXX(inertia);
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.2f, 0.8f, 0.2f});

        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(dim);
        sphere->AddMaterial(mat);
        m_chassis_body->AddVisualShape(sphere);

        utils::ChBodyGeometry::AddVisualizationCylinder(m_chassis_body,              //
                                                        ChVector3d(0, 0, 0),         //
                                                        ChVector3d(0, 0, -2 * dim),  //
                                                        dim / 2,                     //
                                                        mat);
    }

    m_slip_body = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_slip_body);
    m_slip_body->SetName("rig_slip");
    m_slip_body->SetPos(ChVector3d(0, 0, -4 * dim));
    m_slip_body->SetMass(mass);
    m_slip_body->SetInertiaXX(inertia);
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.2f, 0.8f, 0.2f});

        auto box = chrono_types::make_shared<ChVisualShapeBox>(4 * dim, dim, 4 * dim);
        box->AddMaterial(mat);
        m_slip_body->AddVisualShape(box);
    }

    m_spindle_body = chrono_types::make_shared<ChBody>();
    m_spindle_body->SetFixed(mode == Mode::SUSPEND);
    ChQuaternion<> qc;
    qc.SetFromAngleX(-m_camber_angle);
    m_system->AddBody(m_spindle_body);
    m_spindle_body->SetName("rig_spindle");
    m_spindle_body->SetMass(0);
    m_spindle_body->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
    m_spindle_body->SetPos(ChVector3d(0, 3 * dim, -4 * dim));
    m_spindle_body->SetRot(qc);
    utils::ChBodyGeometry::AddVisualizationCylinder(m_spindle_body,              //
                                                    ChVector3d(0, 0, 0),         //
                                                    ChVector3d(0, -3 * dim, 0),  //
                                                    dim / 2);

    // Create joints and motors
    if (mode == Mode::TEST && m_ls_actuated) {
        m_lin_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        m_system->AddLink(m_lin_motor);
        m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleY(CH_PI_2)));
    } else {
        ChQuaternion<> z2x;
        z2x.SetFromAngleY(CH_PI_2);
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        m_system->AddLink(prismatic);
        prismatic->Initialize(m_carrier_body, m_ground_body, ChFrame<>(VNULL, z2x));
    }

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_system->AddLink(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChFrame<>(VNULL, QUNIT));

    m_slip_lock = chrono_types::make_shared<ChLinkLockLock>();
    m_system->AddLink(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChFrame<>(VNULL, QUNIT));
    m_slip_lock->SetMotionAxis(ChVector3d(0, 0, 1));

    ChQuaternion<> z2y;
    z2y.SetFromAngleX(-CH_PI_2 - m_camber_angle);
    if (mode == Mode::TEST && m_rs_actuated) {
        m_rot_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        m_system->AddLink(m_rot_motor);
        m_rot_motor->Initialize(m_spindle_body, m_slip_body, ChFrame<>(ChVector3d(0, 3 * dim, -4 * dim), z2y));
    } else {
        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        m_system->AddLink(revolute);
        revolute->Initialize(m_spindle_body, m_slip_body, ChFrame<>(ChVector3d(0, 3 * dim, -4 * dim), z2y));
    }

    // Initialize subsystems
    m_wheel->Initialize(nullptr, m_spindle_body, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);
    m_wheel->SetTire(m_tire);
    m_tire->SetStepsize(m_tire_step);
    m_tire->Initialize(m_wheel);
    m_tire->SetVisualizationType(m_tire_vis);

    // Update chassis mass to satisfy requested normal load
    if (m_grav > 0) {
        m_total_mass = m_normal_load / m_grav;
        double other_mass = m_slip_body->GetMass() + m_spindle_body->GetMass() + m_wheel->GetMass() + m_tire->GetMass();
        double chassis_mass = m_total_mass - other_mass;
        if (chassis_mass > mass) {
            m_chassis_body->SetMass(chassis_mass);
        } else {
            std::cout << "\nWARNING!  Prescribed normal load too small. Discarded.\n" << std::endl;
        }
    }

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
    ChVector3d location(m_params_SCM.length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, m_terrain_height);

    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Mesh divisions
    double delta = 0.125;  // initial SCM grid spacing

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(m_system);
    terrain->SetPlane(ChCoordsys<>(location));
    terrain->SetSoilParameters(m_params_SCM.Bekker_Kphi, m_params_SCM.Bekker_Kc, m_params_SCM.Bekker_n,            //
                               m_params_SCM.Mohr_cohesion, m_params_SCM.Mohr_friction, m_params_SCM.Janosi_shear,  //
                               E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
    terrain->Initialize(m_params_SCM.length, m_params_SCM.width, delta);
    terrain->AddMovingPatch(m_chassis_body, ChVector3d(0, 0, 0),
                            ChVector3d(2 * m_tire->GetRadius(), 1.0, 2 * m_tire->GetRadius()));

    m_terrain = terrain;
}

void ChTireTestRig::CreateTerrainRigid() {
    ChVector3d location(m_params_rigid.length / 2 - 2 * m_tire->GetRadius(), m_terrain_offset, m_terrain_height);

    auto terrain = chrono_types::make_shared<vehicle::RigidTerrain>(m_system);

    ChContactMaterialData minfo;
    minfo.mu = m_params_rigid.friction;
    minfo.cr = m_params_rigid.restitution;
    minfo.Y = m_params_rigid.Young_modulus;
    auto patch_mat = minfo.CreateMaterial(m_system->GetContactMethod());

    auto patch =
        terrain->AddPatch(patch_mat, ChCoordsys<>(location, QUNIT), m_params_rigid.length, m_params_rigid.width, 0.1);

    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(GetChronoDataFile("textures/pinkwhite.png"), 10 * (float)m_params_rigid.length,
                      10 * (float)m_params_rigid.width);
    terrain->Initialize();

    m_terrain = terrain;
}

void ChTireTestRig::CreateTerrainGranular() {
    double vertical_offset = m_params_granular.num_layers * (2 * m_params_granular.radius);
    ChVector3d location(0, m_terrain_offset, m_terrain_height - vertical_offset);

    auto terrain = chrono_types::make_shared<vehicle::GranularTerrain>(m_system);

    double coh_force = (CH_PI * m_params_granular.radius * m_params_granular.radius) * m_params_granular.cohesion;
    switch (m_system->GetContactMethod()) {
        case ChContactMethod::SMC: {
            auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_g->SetFriction(static_cast<float>(m_params_granular.friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetYoungModulus(8e5f);
            mat_g->SetPoissonRatio(0.3f);
            mat_g->SetAdhesion(static_cast<float>(coh_force));
            mat_g->SetKn(1.0e6f);
            mat_g->SetGn(6.0e1f);
            mat_g->SetKt(4.0e5f);
            mat_g->SetGt(4.0e1f);
            terrain->SetContactMaterial(mat_g);
            break;
        }
        case ChContactMethod::NSC: {
            double step_size = 1e-3;  ///< estimate for integration step size
            auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_g->SetFriction(static_cast<float>(m_params_granular.friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetCohesion(static_cast<float>(coh_force * step_size));
            terrain->SetContactMaterial(mat_g);
            terrain->SetCollisionEnvelope(0.05 * m_params_granular.radius);
            break;
        }
    }

    ////terrain->EnableVisualization(true);
    terrain->EnableVerbose(true);
    terrain->Initialize(location, m_params_granular.length, m_params_granular.width, m_params_granular.num_layers,
                        m_params_granular.radius, m_params_granular.density);

    double buffer_dist = 2.0 * m_tire->GetRadius();
    double shift_dist = 0.5 * m_tire->GetRadius();
    terrain->EnableMovingPatch(m_spindle_body, buffer_dist, shift_dist, ChVector3d(0, 0, -2));

    m_terrain = terrain;
}

// -----------------------------------------------------------------------------

void ChTireTestRig::GetSuggestedCollisionSettings(double& collision_envelope, ChVector3i& collision_bins) const {
    if (m_terrain_type != TerrainType::GRANULAR) {
        collision_envelope = 0.01;
        collision_bins = ChVector3i(1, 1, 1);
        return;
    }

    collision_envelope = 0.05 * m_params_granular.radius;

    int factor = 2;
    collision_bins.x() = (int)std::ceil((0.5 * m_params_granular.length) / m_params_granular.radius) / factor;
    collision_bins.y() = (int)std::ceil((0.5 * m_params_granular.width) / m_params_granular.radius) / factor;
    collision_bins.z() = 1;
}

// -----------------------------------------------------------------------------

TerrainForce ChTireTestRig::ReportTireForce() const {
    return m_tire->ReportTireForce(m_terrain.get());
}

// -----------------------------------------------------------------------------

double ChTireTestRig::GetDBP() const {
    return -m_lin_motor->GetMotorForce();
}

}  // end namespace vehicle
}  // end namespace chrono
