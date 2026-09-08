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
// Implementation of a single-wheel test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#ifdef CHRONO_FEA
    #include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#endif

#ifdef CHRONO_CRM
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
#endif

namespace chrono {
namespace vehicle {

// =============================================================================

// ChWheelTestRig::SuspensionAssembly that wraps a Chrono::Vehicle suspension.
class VehicleSuspensionAssembly : public ChWheelTestRigBase::SuspensionAssembly {
  public:
    VehicleSuspensionAssembly(ChSystem& system, std::shared_ptr<ChSuspension> suspension);

    virtual double GetMass() const override { return suspension->GetMass(); }

    virtual std::shared_ptr<ChBody> GetSpindle() const override;

    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis_body, const ChVector3d& loc, bool fixed, VisualizationType vis_type) override;
    virtual void Synchronize(double time, const ChTerrain& terrain) override;
    virtual void Advance(double step_size) override;

  private:
    std::shared_ptr<ChChassis> chassis;
    std::shared_ptr<ChSuspension> suspension;

    friend class VehicleWheelAssembly;
};

// Stand-in chassis object for a VehicleSuspensionAssembly.
class DummyChassis : public ChChassis {
  public:
    DummyChassis(std::shared_ptr<ChBodyAuxRef> chassis_body) : ChChassis("dummy_chassis"), chassis_body(chassis_body) {}

    virtual std::string GetTemplateName() const override { return "dummy_template"; }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsys<>(); }
    virtual ChFrame<> GetBodyCOMFrame() const { return ChFrame<>(); }
    virtual double GetBodyMass() const { return chassis_body->GetMass(); }
    virtual ChMatrix33<> GetBodyInertia() const { return chassis_body->GetInertia(); }
    virtual void EnableCollision(bool state) {}
    virtual void OnInitialize(ChVehicle* vehicle, const ChCoordsys<>& chassisPos, double chassisFwdVel, int collision_family) override {
        m_body = chassis_body;  // set the underlying ChChassis body
    }

    std::shared_ptr<ChBodyAuxRef> chassis_body;
};

VehicleSuspensionAssembly::VehicleSuspensionAssembly(ChSystem& system, std::shared_ptr<ChSuspension> suspension) : SuspensionAssembly(system), suspension(suspension) {}

std::shared_ptr<ChBody> VehicleSuspensionAssembly::GetSpindle() const {
    return suspension->GetSpindle(VehicleSide::LEFT);
}

void VehicleSuspensionAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis_body, const ChVector3d& loc, bool fixed, VisualizationType vis_type) {
    chassis = chrono_types::make_shared<DummyChassis>(chassis_body);
    chassis->Initialize(nullptr, ChCoordsysd(chassis_body->GetPos(), QUNIT), 0.0);
    chassis->SetVisualizationType(vis_type);

    suspension->Initialize(chassis, nullptr, nullptr, loc);
    suspension->SetVisualizationType(vis_type);
}

void VehicleSuspensionAssembly::Synchronize(double time, const ChTerrain& terrain) {
    chassis->Synchronize(time);
    suspension->Synchronize(time);
}

void VehicleSuspensionAssembly::Advance(double step_size) {
    suspension->Advance(step_size);
}

// =============================================================================

// ChWheelTestRig::WheelAssembly that wraps a Chrono::Vehicle tire and wheel assembly.
class VehicleWheelAssembly : public ChWheelTestRigBase::WheelAssembly {
  public:
    VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, bool attached_to_suspension);

    virtual double GetMass() const override { return wheel->GetMass() + tire->GetTireMass(); }
    virtual double GetRadius() const override { return tire->GetRadius(); }
    virtual double GetWidth() const override { return tire->GetWidth(); }

    virtual std::shared_ptr<ChBody> GetHub() const override { return spindle; }

#ifdef CHRONO_CRM
    virtual void AddFSIBodies(CRMTerrain& terrain, double spacing) override;
#endif

    virtual void Initialize(const ChFramed& frame, bool fixed, double step_size, VisualizationType vis_type) override;
    virtual void Initialize(std::shared_ptr<ChWheelTestRigBase::SuspensionAssembly> suspension, double step_size, VisualizationType vis_type) override;
    virtual void Synchronize(double time, const ChTerrain& terrain) override;
    virtual void Advance(double step_size) override;

    virtual TerrainForce ReportForces(ChTerrain& terrain) const override;

  private:
    std::shared_ptr<ChSpindle> spindle;
    std::shared_ptr<ChWheel> wheel;
    std::shared_ptr<ChTire> tire;
};

VehicleWheelAssembly::VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, bool attached_to_suspension)
    : WheelAssembly(system, attached_to_suspension), wheel(wheel), tire(tire) {}

#ifdef CHRONO_CRM

void VehicleWheelAssembly::AddFSIBodies(CRMTerrain& terrain, double spacing) {
    #ifdef CHRONO_FEA
    if (auto fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        auto mesh = fea_tire->GetMesh();
        terrain.AddFeaMesh(mesh, false);
        return;
    }
    #endif

    if (auto rgd_tire = std::static_pointer_cast<ChRigidTire>(tire)) {
        assert(rgd_tire->UseContactMesh());
        auto trimesh = rgd_tire->GetContactMesh();
        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, trimesh, 1.0, 0.0, 0));
        terrain.AddRigidBody(spindle, geometry, false);
        return;
    }

    if (std::dynamic_pointer_cast<ChForceElementTire>(tire)) {
        std::cerr << "ERROR: Handling tire models cannot be used with CRM terrain." << std::endl;
        throw std::runtime_error("ERROR: Handling tire models cannot be used with CRM terrain.");
    }
}

#endif

void VehicleWheelAssembly::Initialize(const ChFramed& frame, bool fixed, double step_size, VisualizationType vis_type) {
    ChAssertAlways(!attached_to_suspension);

    spindle = chrono_types::make_shared<ChSpindle>();
    spindle->SetName("rig_spindle");
    spindle->SetMass(0);
    spindle->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
    system.AddBody(spindle);

    spindle->SetPos(frame.GetPos());
    spindle->SetRot(frame.GetRot());
    spindle->SetFixed(fixed);

    wheel->Initialize(nullptr, spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);
    wheel->SetTire(tire);

    tire->Initialize(wheel);
    tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
    tire->SetStepsize(step_size);
    tire->SetVisualizationType(vis_type);
}

void VehicleWheelAssembly::Initialize(std::shared_ptr<ChWheelTestRigBase::SuspensionAssembly> suspension, double step_size, VisualizationType vis_type) {
    auto vsa = std::static_pointer_cast<VehicleSuspensionAssembly>(suspension);

    spindle = vsa->suspension->GetSpindle(VehicleSide::LEFT);

    wheel->Initialize(nullptr, spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);
    wheel->SetTire(tire);

    tire->Initialize(wheel);
    tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
    tire->SetStepsize(step_size);
    tire->SetVisualizationType(vis_type);
}

void VehicleWheelAssembly::Synchronize(double time, const ChTerrain& terrain) {
    spindle->EmptyTireAccumulator();
    tire->Synchronize(time, terrain);
    wheel->Synchronize();
}

void VehicleWheelAssembly::Advance(double step_size) {
    tire->Advance(step_size);
}

TerrainForce VehicleWheelAssembly::ReportForces(ChTerrain& terrain) const {
    return tire->ReportTireForce(&terrain);
}

// =============================================================================

ChWheelTestRigBase::ChWheelTestRigBase(ChSystem& system, std::shared_ptr<WheelAssembly> wheel)
    : m_wheel_assembly(wheel),
      m_system(system),
      m_grav(9.8),
      m_normal_load(1000),
      m_total_mass(0),
      m_mode(Mode::SUSPEND),
      m_output(false),
      m_time_delay(0),
      m_ls_actuated(false),
      m_rs_actuated(false),
      m_long_slip_constant(false),
      m_terrain_type(TerrainType::NONE),
      m_terrain_offset(0),
      m_terrain_height(0),
      m_step_size(1e-3),
      m_vis_type(VisualizationType::PRIMITIVES) {}

ChWheelTestRigBase::~ChWheelTestRigBase() {
    m_system.Remove(m_ground_body);
    m_system.Remove(m_carrier_body);
    m_system.Remove(m_chassis_body);
    m_system.Remove(m_drop_motor);
    m_system.Remove(m_lin_motor);
    m_system.Remove(m_rot_motor);
}

// -----------------------------------------------------------------------------

void ChWheelTestRigBase::SetLongSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_ls_fun = funct;
    m_ls_actuated = true;
}

void ChWheelTestRigBase::SetAngSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_rs_fun = funct;
    m_rs_actuated = true;
}

void ChWheelTestRigBase::SetConstantLongitudinalSlip(double long_slip, double base_speed) {
    m_ls_actuated = true;
    m_rs_actuated = true;
    m_long_slip_constant = true;
    m_long_slip = long_slip;
    m_base_speed = base_speed;
}

// -----------------------------------------------------------------------------

void ChWheelTestRigBase::SetTerrainRigid(const TerrainPatchSize& size, const TerrainParamsRigid& params) {
    m_terrain_type = TerrainType::RIGID;
    m_terrain_size = size;
    m_params_rigid = params;
}

void ChWheelTestRigBase::SetTerrainRigid(const TerrainPatchSize& size, float mu, float cr, float Y) {
    m_terrain_type = TerrainType::RIGID;
    m_terrain_size = size;

    m_params_rigid.mu = mu;
    m_params_rigid.cr = cr;
    m_params_rigid.Y = Y;
}

void ChWheelTestRigBase::SetTerrainSCM(const TerrainPatchSize& size, const TerrainParamsSCM& params) {
    m_terrain_type = TerrainType::SCM;
    m_terrain_size = size;
    m_params_SCM = params;
}

void ChWheelTestRigBase::SetTerrainSCM(const TerrainPatchSize& size,
                                       double Bekker_Kphi,
                                       double Bekker_Kc,
                                       double Bekker_n,
                                       double Mohr_cohesion,
                                       double Mohr_friction,
                                       double Janosi_shear,
                                       double grid_spacing) {
    m_terrain_type = TerrainType::SCM;
    m_terrain_size = size;

    m_params_SCM.Bekker_Kphi = Bekker_Kphi;
    m_params_SCM.Bekker_Kc = Bekker_Kc;
    m_params_SCM.Bekker_n = Bekker_n;
    m_params_SCM.Mohr_cohesion = Mohr_cohesion;
    m_params_SCM.Mohr_friction = Mohr_friction;
    m_params_SCM.Janosi_shear = Janosi_shear;
    m_params_SCM.grid_spacing = grid_spacing;
}

void ChWheelTestRigBase::SetTerrainGranular(const TerrainPatchSize& size, const TerrainParamsGranular& params) {
    m_terrain_type = TerrainType::GRANULAR;
    m_terrain_size = size;

    m_params_granular = params;
}

void ChWheelTestRigBase::SetTerrainGranular(const TerrainPatchSize& size, double radius, double density, double friction, double cohesion, double Young_modulus) {
    m_terrain_type = TerrainType::GRANULAR;
    m_terrain_size = size;

    m_params_granular.radius = radius;
    m_params_granular.density = density;
    m_params_granular.friction = friction;
    m_params_granular.cohesion = cohesion;
    m_params_granular.Young_modulus = Young_modulus;
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

void ChWheelTestRigBase::Initialize(Mode mode, double drop_speed) {
    m_mode = mode;

    CreateMechanism();
    CreateTerrain();

    OnInitialize(mode);

    std::cout << "Wheel radius: " << m_wheel_assembly->GetRadius() << std::endl;

    if (m_mode != Mode::TEST)
        return;

    // Set the drop speed
    m_drop_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(drop_speed));

    // Override motion functions to enforce specified constant longitudinal slip
    if (m_long_slip_constant) {
        m_ls_fun = chrono_types::make_shared<LinSpeedFunction>(m_base_speed);
        m_rs_fun = chrono_types::make_shared<RotSpeedFunction>(m_long_slip, m_base_speed, m_wheel_assembly->GetRadius());
    }
}

void ChWheelTestRigBase::Advance(double step) {
    double time = m_system.GetChTime();

    // Check end of dropping phase
    if (m_mode == Mode::TEST &&                                                                      // in TEST mode
        !m_drop_motor->IsDisabled() &&                                                               // dropping is ongoing
        m_wheel_assembly->GetHub()->GetPos().z() < m_terrain_height + m_wheel_assembly->GetRadius()  // wheel bottom reached terrain
    ) {
        std::cout << "\n  time : " << time << " - end drop phase" << std::endl;

        // Disable the actuator for wheel drop
        m_drop_motor->SetDisabled(true);

        // Set motor functions with a delay measured from current time
        m_time_delay += time;
        std::cout << "  motor activation delayed until t = " << m_time_delay << std::endl;

        if (m_ls_actuated)
            m_lin_motor->SetSpeedFunction(chrono_types::make_shared<DelayedFun>(m_ls_fun, m_time_delay));

        if (m_rs_actuated)
            m_rot_motor->SetSpeedFunction(chrono_types::make_shared<DelayedFun>(m_rs_fun, m_time_delay));

        OnStartActuation();
    }

    // Turn on calculation of measured quantities
    if (m_mode == Mode::TEST &&        // in TEST mode
        !m_output &&                   // measurements not yet enabled
        m_drop_motor->IsDisabled() &&  // dropping phase done
        time > m_time_delay            // past activation delay
    ) {
        std::cout << "\n  time : " << time << " - enable measurements" << std::endl;
        m_output = true;
    }

    OnAdvance(step);

    if (m_terrain_type == TerrainType::CRM) {
#ifdef CHRONO_CRM
        std::static_pointer_cast<CRMTerrain>(m_terrain)->GetFsiSystemSPH()->DoStepDynamics(step);
#endif
    } else {
        // Synchronize subsystems
        m_terrain->Synchronize(time);
        m_wheel_assembly->Synchronize(time, *m_terrain.get());

        // Advance state
        m_terrain->Advance(step);
        m_wheel_assembly->Advance(step);
        m_system.DoStepDynamics(step);
    }
}

// -----------------------------------------------------------------------------

void ChWheelTestRigBase::CreateMechanism() {
    m_system.SetGravitationalAcceleration(ChVector3d(0, 0, -m_grav));

    // Set characteristic dimension
    const double dim = 0.1;

    // Set total sprung mass based on requested normal load
    if (m_grav > 0)
        m_total_mass = m_normal_load / m_grav;

    // ---- Create the main rig bodies

    m_ground_body = chrono_types::make_shared<ChBody>();
    m_system.AddBody(m_ground_body);
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetFixed(true);
    {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(100, dim * CH_1_3, dim * CH_1_3);
        m_ground_body->AddVisualShape(box);
    }

    m_carrier_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_system.AddBody(m_carrier_body);
    m_carrier_body->SetName("rig_carrier");
    m_carrier_body->SetPos(ChVector3d(0, 0, 0));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.6f, 0.2f, 0.2f});

        utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier_body,              //
                                                        ChVector3d(+2 * dim, 0, 0),  //
                                                        ChVector3d(-2 * dim, 0, 0),  //
                                                        dim / 2,                     //
                                                        mat);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(dim * CH_1_3, dim * CH_1_3, 10 * dim);
        box->AddMaterial(mat);
        m_carrier_body->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -5 * dim)));
    }

    m_chassis_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_system.AddBody(m_chassis_body);
    m_chassis_body->SetName("rig_chassis");
    m_chassis_body->SetPos(ChVector3d(0, 0, 0));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.2f, 0.6f, 0.2f});

        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(dim);
        sphere->AddMaterial(mat);
        m_chassis_body->AddVisualShape(sphere);

        utils::ChBodyGeometry::AddVisualizationCylinder(m_chassis_body,              //
                                                        ChVector3d(0, 0, 0),         //
                                                        ChVector3d(0, 0, -2 * dim),  //
                                                        dim / 2,                     //
                                                        mat);
    }

    // ---- Create main joints and motors

    if (m_mode == Mode::TEST && m_ls_actuated) {
        m_lin_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        m_system.AddLink(m_lin_motor);
        m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleY(CH_PI_2)));
    } else {
        ChQuaternion<> z2x;
        z2x.SetFromAngleY(CH_PI_2);
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        m_system.AddLink(prismatic);
        prismatic->Initialize(m_carrier_body, m_ground_body, ChFrame<>(VNULL, z2x));
    }

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_system.AddLink(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChFrame<>(VNULL, QUNIT));

    if (m_mode == Mode::TEST) {
        m_drop_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        m_system.AddLink(m_drop_motor);
        m_drop_motor->Initialize(m_carrier_body, m_chassis_body, ChFrame<>(VNULL, QUNIT));
    }

    // ---- Let concrete rig test classes construct additional physics items, initialize and connect the wheel assembly, and adjust mass properties

    OnCreateMechanism(dim);

    // ---- Create shafts and shaft motor used to actuate the wheel (if  needed)

    if (m_mode == Mode::TEST && m_rs_actuated) {
        auto chassis_shaft = chrono_types::make_shared<ChShaft>();
        chassis_shaft->SetInertia(0.1);
        m_system.AddShaft(chassis_shaft);

        auto wheel_shaft = chrono_types::make_shared<ChShaft>();
        wheel_shaft->SetInertia(0.1);
        m_system.AddShaft(wheel_shaft);

        auto shaft_to_chassis = chrono_types::make_shared<ChShaftBodyRotation>();
        shaft_to_chassis->Initialize(chassis_shaft, m_chassis_body, ChVector3d(0, -1, 0));
        m_system.Add(shaft_to_chassis);

        auto shaft_to_wheel = chrono_types::make_shared<ChShaftBodyRotation>();
        shaft_to_wheel->Initialize(wheel_shaft, m_wheel_assembly->GetHub(), ChVector3d(0, -1, 0));
        m_system.Add(shaft_to_wheel);

        m_rot_motor = chrono_types::make_shared<ChShaftsMotorSpeed>();
        m_system.Add(m_rot_motor);
        m_rot_motor->Initialize(chassis_shaft, wheel_shaft);
    }

    // ---- Set terrain offset (based on wheel center) and terrain height (below wheel)

    m_terrain_offset = m_wheel_assembly->GetHub()->GetPos().y();
    m_terrain_height = m_wheel_assembly->GetHub()->GetPos().z() - m_wheel_assembly->GetRadius() - 0.1;
}

// -----------------------------------------------------------------------------

void ChWheelTestRigBase::CreateTerrain() {
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
        case TerrainType::CRM:
#ifdef CHRONO_CRM
            CreateTerrainCRM();
#endif
            break;
        default:
            break;
    }
}

void ChWheelTestRigBase::CreateTerrainSCM() {
    ChVector3d location(m_terrain_size.length / 2 - 2 * m_wheel_assembly->GetRadius(), m_terrain_offset, m_terrain_height);

    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&m_system);
    terrain->SetReferenceFrame(ChCoordsys<>(location));
    terrain->SetSoilParameters(                                                             //
        m_params_SCM.Bekker_Kphi, m_params_SCM.Bekker_Kc, m_params_SCM.Bekker_n,            //
        m_params_SCM.Mohr_cohesion, m_params_SCM.Mohr_friction, m_params_SCM.Janosi_shear,  //
        E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
    terrain->Initialize(m_terrain_size.length, m_terrain_size.width, m_params_SCM.grid_spacing);
    terrain->AddActiveDomain(m_chassis_body, ChVector3d(0, 0, 0), ChVector3d(2 * m_wheel_assembly->GetRadius(), 1.0, 2 * m_wheel_assembly->GetRadius()));

    m_terrain = terrain;
}

void ChWheelTestRigBase::CreateTerrainRigid() {
    ChVector3d location(m_terrain_size.length / 2 - 2 * m_wheel_assembly->GetRadius(), m_terrain_offset, m_terrain_height);

    auto terrain = chrono_types::make_shared<vehicle::RigidTerrain>(&m_system);

    ChContactMaterialData minfo;
    minfo.mu = m_params_rigid.mu;
    minfo.cr = m_params_rigid.cr;
    minfo.Y = m_params_rigid.Y;
    auto patch_mat = minfo.CreateMaterial(m_system.GetContactMethod());

    auto patch = terrain->AddPatch(patch_mat, ChCoordsys<>(location, QUNIT), m_terrain_size.length, m_terrain_size.width, 0.1);

    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(GetChronoDataFile("textures/pinkwhite.png"), 10 * (float)m_terrain_size.length, 10 * (float)m_terrain_size.width);
    terrain->Initialize();

    m_terrain = terrain;
}

void ChWheelTestRigBase::CreateTerrainGranular() {
    int num_layers = (int)(m_terrain_size.depth / (2 * m_params_granular.radius)) + 1;
    double vertical_offset = num_layers * (2 * m_params_granular.radius);
    ChVector3d location(0, m_terrain_offset, m_terrain_height - vertical_offset);

    auto terrain = chrono_types::make_shared<vehicle::GranularTerrain>(&m_system);

    double coh_force = (CH_PI * m_params_granular.radius * m_params_granular.radius) * m_params_granular.cohesion;
    switch (m_system.GetContactMethod()) {
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

    terrain->Initialize(location, m_terrain_size.length, m_terrain_size.width, num_layers, m_params_granular.radius, m_params_granular.density);

    double buffer_dist = 2.0 * m_wheel_assembly->GetRadius();
    double shift_dist = 0.5 * m_wheel_assembly->GetRadius();
    terrain->EnableMovingPatch(m_wheel_assembly->GetHub(), buffer_dist, shift_dist, ChVector3d(0, 0, -2));

    m_terrain = terrain;
}

#ifdef CHRONO_CRM

ChWheelTestRig::TerrainParamsCRM::TerrainParamsCRM() {
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = 0.02;
    sph_params.d0_multiplier = 1.2;
    sph_params.artificial_viscosity = 0.5;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 0.8;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
}

void ChWheelTestRigBase::SetTerrainCRM(const TerrainPatchSize& size, double spacing, double density, double Young_modulus, double friction, double cohesion) {
    m_terrain_type = TerrainType::CRM;
    m_terrain_size = size;

    m_params_crm.sph_params.initial_spacing = spacing;

    m_params_crm.mat_props.density = density;
    m_params_crm.mat_props.cohesion_coeff = cohesion;
    m_params_crm.mat_props.Young_modulus = Young_modulus;
    m_params_crm.mat_props.Poisson_ratio = 0.3;
    m_params_crm.mat_props.mu_I0 = 0.03;
    m_params_crm.mat_props.mu_fric_s = friction;
    m_params_crm.mat_props.mu_fric_2 = friction;
    m_params_crm.mat_props.average_diam = 0.0614;
}

void ChWheelTestRigBase::SetTerrainCRM(const TerrainPatchSize& size, const TerrainParamsCRM& params) {
    m_terrain_type = TerrainType::CRM;
    m_terrain_size = size;

    m_params_crm = params;
}

void ChWheelTestRigBase::SetWheelActiveDomain(const ChAABB& aabb) {
    m_wheel_AABB = aabb;
}

void ChWheelTestRigBase::SetWheelActiveDomain() {
    auto corner = ChVector3d(m_wheel_assembly->GetRadius(), m_wheel_assembly->GetWidth() / 2, m_wheel_assembly->GetRadius());
    m_wheel_AABB.min = -1.25 * corner;
    m_wheel_AABB.max = +1.25 * corner;
}

void ChWheelTestRigBase::CreateTerrainCRM() {
    std::shared_ptr<CRMTerrain> terrain = chrono_types::make_shared<CRMTerrain>(m_system, m_params_crm.sph_params.initial_spacing);

    terrain->SetOutputLevel(OutputLevel::STATE);
    terrain->SetGravitationalAcceleration(ChVector3d(0, 0, -m_grav));

    terrain->SetStepSizeCFD(m_step_size);

    terrain->SetStepsizeMBD(m_step_size);

    terrain->SetCrmSPH(m_params_crm.mat_props);
    terrain->SetSPHParameters(m_params_crm.sph_params);

    double loc_z = m_terrain_height - m_terrain_size.depth;
    ChVector3d location(m_terrain_size.length / 2 - 2 * m_wheel_assembly->GetRadius(), m_terrain_offset, loc_z);
    terrain->Construct({m_terrain_size.length, m_terrain_size.width, m_terrain_size.depth}, location, BoxSide::ALL & ~BoxSide::Z_POS);

    // Add wheel FSI bodies
    m_wheel_assembly->AddFSIBodies(*terrain, m_params_crm.sph_params.initial_spacing);

    // If a wheel-level active domain was defined, associate it with the hub body
    if (!m_wheel_AABB.IsInverted()) {
        // Add the hub as a (dummy) FSI body if not already so declared
        if (!terrain->IsFsiSolid(m_wheel_assembly->GetHub()))
            terrain->AddRigidBody(m_wheel_assembly->GetHub(), nullptr, false);
        terrain->SetActiveDomainBody(m_wheel_assembly->GetHub(), m_wheel_AABB);
    }

    terrain->Initialize();
    const auto& aabb = terrain->GetSPHBoundingBox();
    std::cout << "  SPH particles:        " << terrain->GetNumSPHParticles() << std::endl;
    std::cout << "  Boundary BCE markers: " << terrain->GetNumBoundaryBCEMarkers() << std::endl;
    std::cout << "  SPH AABB:             " << aabb.min << "   " << aabb.max << std::endl;

    m_terrain = terrain;
}

#endif

// -----------------------------------------------------------------------------

void ChWheelTestRigBase::GetSuggestedCollisionSettings(double& collision_envelope, ChVector3i& collision_bins) const {
    if (m_terrain_type != TerrainType::GRANULAR) {
        collision_envelope = 0.01;
        collision_bins = ChVector3i(1, 1, 1);
        return;
    }

    collision_envelope = 0.05 * m_params_granular.radius;

    int factor = 2;
    collision_bins.x() = (int)std::ceil((0.5 * m_terrain_size.length) / m_params_granular.radius) / factor;
    collision_bins.y() = (int)std::ceil((0.5 * m_terrain_size.width) / m_params_granular.radius) / factor;
    collision_bins.z() = 1;
}

// -----------------------------------------------------------------------------

TerrainForce ChWheelTestRigBase::ReportWheelForce() const {
    if (!m_output)
        return TerrainForce();

    return m_wheel_assembly->ReportForces(*m_terrain);
}

double ChWheelTestRigBase::GetDBP() const {
    if (!m_output)
        return 0;

    return -m_lin_motor->GetMotorForce();
}

double ChWheelTestRigBase::GetLongitudinalSlip() const {
    if (!m_output)
        return 0;

    double r = m_wheel_assembly->GetRadius();                     // current wheel radius
    double o = m_wheel_assembly->GetHub()->GetAngVelLocal().y();  // hub rotation angular speed (local)
    auto v = m_wheel_assembly->GetHub()->GetPosDt();              // hub 3D velocity (global)
    double vx = std::sqrt(v.x() * v.x() + v.y() * v.y());         // hub horizontal speed (global)
    double abs_vx = std::abs(vx);

    double long_slip = (abs_vx > 1e-4) ? (r * o - vx) / abs_vx : 0.0;
    return long_slip;
}

double ChWheelTestRigBase::GetSlipAngle() const {
    if (!m_output)
        return 0;

    auto dir = m_wheel_assembly->GetHub()->GetRotMat().GetAxisY();
    double slip_angle = std::atan(dir.x() / dir.y());
    return slip_angle;
}

double ChWheelTestRigBase::GetCamberAngle() const {
    if (!m_output)
        return 0;

    auto dir = m_wheel_assembly->GetHub()->GetRotMat().GetAxisY();
    double camber_angle = std::atan(-dir.z());
    return camber_angle;
}

// =============================================================================

ChWheelTestRig::ChWheelTestRig(ChSystem& system, std::shared_ptr<WheelAssembly> wheel)
    : ChWheelTestRigBase(system, wheel), m_camber_angle(0), m_sa_fun(chrono_types::make_shared<ChFunctionConst>(0)) {}

ChWheelTestRig::ChWheelTestRig(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire)
    : ChWheelTestRig(system, chrono_types::make_shared<VehicleWheelAssembly>(system, wheel, tire, false)) {}

ChWheelTestRig::~ChWheelTestRig() {
    m_system.Remove(m_slip_body);
    m_system.Remove(m_slip_lock);
}

void ChWheelTestRig::OnCreateMechanism(double dim) {
    // Initialize the wheel assembly
    ChQuaternion<> qc;
    qc.SetFromAngleX(-m_camber_angle);
    m_wheel_assembly->Initialize(ChFramed(ChVector3d(0, 3 * dim, -4 * dim), qc), (m_mode == Mode::SUSPEND), m_step_size, m_vis_type);

    // Adjust rig body mass and inertia commensurate with those of the wheel system
    const double mass = m_wheel_assembly->GetMass();
    const double radius = m_wheel_assembly->GetRadius();
    ChMatrix33d inertia = 0.25 * mass * ChSphere::CalcGyration(radius);

    m_carrier_body->SetMass(mass);
    m_carrier_body->SetInertia(inertia);

    m_chassis_body->SetMass(mass);
    m_chassis_body->SetInertia(inertia);

    // Create the slip body
    m_slip_body = chrono_types::make_shared<ChBody>();
    m_system.AddBody(m_slip_body);
    m_slip_body->SetName("rig_slip");
    m_slip_body->SetPos(ChVector3d(0, 0, -4 * dim));
    m_slip_body->SetMass(mass);
    m_slip_body->SetInertia(inertia);
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.2f, 0.2f, 0.6f});

        auto box = chrono_types::make_shared<ChVisualShapeBox>(4 * dim, dim, 4 * dim);
        box->AddMaterial(mat);
        m_slip_body->AddVisualShape(box);
    }

    // Create chassis to slip body connection which allows controlling slip angle
    m_slip_lock = chrono_types::make_shared<ChLinkLockLock>();
    m_system.AddLink(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChFrame<>(VNULL, QUNIT));
    m_slip_lock->SetMotionAxis(ChVector3d(0, 0, 1));

    // Connect wheel to slip body
    ChQuaternion<> z2y;
    z2y.SetFromAngleX(-CH_PI_2 - m_camber_angle);
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_system.AddLink(revolute);
    revolute->Initialize(m_wheel_assembly->GetHub(), m_slip_body, ChFrame<>(ChVector3d(0, 3 * dim, -4 * dim), z2y));

    // Update chassis mass to satisfy requested normal load
    if (m_grav > 0) {
        m_total_mass = m_normal_load / m_grav;
        double other_mass = m_slip_body->GetMass() + m_wheel_assembly->GetMass();
        double chassis_mass = m_total_mass - other_mass;
        if (chassis_mass > mass) {
            m_chassis_body->SetMass(chassis_mass);
        } else {
            std::cout << "\nWARNING!  Prescribed normal load too small. Discarded.\n" << std::endl;
        }
    }
}

void ChWheelTestRig::OnInitialize(Mode mode) {
    if (m_mode != Mode::TEST)
        return;
}

void ChWheelTestRig::OnAdvance(double step) {}

void ChWheelTestRig::OnStartActuation() {
    m_slip_lock->SetMotionAng1(chrono_types::make_shared<DelayedFun>(m_sa_fun, m_time_delay));
}

// =============================================================================

ChWheelSuspensionTestRig::ChWheelSuspensionTestRig(ChSystem& system, std::shared_ptr<WheelAssembly> wheel, std::shared_ptr<SuspensionAssembly> suspension)
    : ChWheelTestRigBase(system, wheel), m_suspension_assembly(suspension) {}

ChWheelSuspensionTestRig::ChWheelSuspensionTestRig(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, std::shared_ptr<ChSuspension> suspension)
    : ChWheelTestRigBase(system, chrono_types::make_shared<VehicleWheelAssembly>(system, wheel, tire, true)),
      m_suspension_assembly(chrono_types::make_shared<VehicleSuspensionAssembly>(system, suspension)) {}

ChWheelSuspensionTestRig::~ChWheelSuspensionTestRig() {}

void ChWheelSuspensionTestRig::OnCreateMechanism(double dim) {
    // Initialize the suspension and wheel assemblies (this also connects wheel to suspension)
    m_suspension_assembly->Initialize(m_chassis_body, ChVector3d(0, 0, -4 * dim), (m_mode == Mode::SUSPEND), m_vis_type);
    m_wheel_assembly->Initialize(m_suspension_assembly, m_step_size, m_vis_type);

    // Adjust rig body mass and inertia commensurate with those of the wheel system
    const double mass = m_wheel_assembly->GetMass();
    const double radius = m_wheel_assembly->GetRadius();
    ChMatrix33d inertia = 0.25 * mass * ChSphere::CalcGyration(radius);

    m_carrier_body->SetMass(mass);
    m_carrier_body->SetInertia(inertia);

    m_chassis_body->SetMass(mass);
    m_chassis_body->SetInertia(inertia);

    // Update chassis mass to satisfy requested normal load
    if (m_grav > 0) {
        m_total_mass = m_normal_load / m_grav;
        double other_mass = m_suspension_assembly->GetMass() + m_wheel_assembly->GetMass();
        double chassis_mass = m_total_mass - other_mass;
        if (chassis_mass > mass) {
            m_chassis_body->SetMass(chassis_mass);
        } else {
            std::cout << "\nWARNING!  Prescribed normal load too small. Discarded.\n" << std::endl;
        }
    }
}

void ChWheelSuspensionTestRig::OnInitialize(Mode mode) {
    if (m_mode != Mode::TEST)
        return;
}

void ChWheelSuspensionTestRig::OnAdvance(double step) {}

}  // end namespace vehicle
}  // end namespace chrono
