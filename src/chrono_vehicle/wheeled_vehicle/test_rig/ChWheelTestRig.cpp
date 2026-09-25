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

// Stand-in chassis object for a VehicleWheelAssembly.
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

        // ChChassis::Initialize adds the load containers only when given a vehicle; add them here so that
        // suspension bushings and chassis external loads are included in the system
        auto sys = chassis_body->GetSystem();
        sys->Add(m_container_bushings);
        sys->Add(m_container_external);
        sys->Add(m_container_terrain);
    }

    // Remove the chassis load containers from the system (if present).
    // The rig owns the chassis body, so ~ChChassis finds it already removed and does not remove the containers.
    void RemoveContainers() {
        for (auto& container : {m_container_bushings, m_container_external, m_container_terrain}) {
            if (container->GetSystem())
                container->GetSystem()->Remove(container);
        }
    }

    std::shared_ptr<ChBodyAuxRef> chassis_body;
};

// ChWheelTestRig::WheelAssembly that wraps a Chrono::Vehicle tire and wheel assembly and, optionally, a Chrono::vehicle suspension.
class VehicleWheelAssembly : public ChWheelTestRig::WheelAssembly {
  public:
    VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire);
    VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChSuspension> suspension, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire);

    virtual bool HasSuspension() const override { return suspension ? true : false; }

    virtual double GetMass() const override;
    virtual double GetWheelMass() const override { return wheel->GetMass(); }
    virtual double GetRadius() const override { return tire->GetRadius(); }
    virtual double GetWidth() const override { return tire->GetWidth(); }

    virtual std::shared_ptr<ChBody> GetHub() const override { return spindle; }

#ifdef CHRONO_CRM
    virtual void AddFSIBodies(CRMTerrain& terrain, double spacing) override;
#endif

    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis_body, const ChFramed& frame, bool fixed_wheel, double step_size, VisualizationType vis_type) override;

    virtual void Synchronize(double time, const ChTerrain& terrain) override;
    virtual void Advance(double step_size) override;

    virtual TerrainForce ReportForces(ChTerrain& terrain) const override;

    virtual void RemoveFromSystem() override;

  private:
    std::shared_ptr<DummyChassis> chassis;
    std::shared_ptr<ChSuspension> suspension;
    std::shared_ptr<ChSpindle> spindle;
    std::shared_ptr<ChWheel> wheel;
    std::shared_ptr<ChTire> tire;
};

VehicleWheelAssembly::VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire)
    : WheelAssembly(system), chassis(nullptr), suspension(nullptr), wheel(wheel), tire(tire) {}

VehicleWheelAssembly::VehicleWheelAssembly(ChSystem& system, std::shared_ptr<ChSuspension> suspension, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire)
    : WheelAssembly(system), chassis(nullptr), suspension(suspension), wheel(wheel), tire(tire) {}

void VehicleWheelAssembly::RemoveFromSystem() {
    // Suspension case: the rig chassis is wrapped in a DummyChassis
    if (chassis)
        chassis->RemoveContainers();

    // No-suspension case: the spindle body was created here (a suspension spindle is removed by the suspension itself)
    if (!suspension && spindle && spindle->GetSystem())
        spindle->GetSystem()->Remove(spindle);
}

double VehicleWheelAssembly::GetMass() const {
    double mass = wheel->GetMass() + tire->GetTireMass();
    if (suspension)
        mass += suspension->GetMass();
    return mass;
}

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

void VehicleWheelAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis_body, const ChFramed& frame, bool fixed_wheel, double step_size, VisualizationType vis_type) {
    if (suspension) {
        chassis = chrono_types::make_shared<DummyChassis>(chassis_body);
        chassis->Initialize(nullptr, ChCoordsysd(chassis_body->GetPos(), QUNIT), 0.0);
        chassis->SetVisualizationType(vis_type);

        suspension->Initialize(chassis, nullptr, nullptr, frame.GetPos());
        suspension->SetVisualizationType(vis_type);

        spindle = suspension->GetSpindle(VehicleSide::LEFT);
    } else {
        spindle = chrono_types::make_shared<ChSpindle>();
        spindle->SetName("rig_spindle");
        spindle->SetMass(0);
        spindle->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
        system.AddBody(spindle);

        spindle->SetPos(frame.GetPos());
        spindle->SetRot(frame.GetRot());
        spindle->SetFixed(fixed_wheel);
    }

    wheel->Initialize(nullptr, spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);
    wheel->SetTire(tire);

    tire->Initialize(wheel);
    tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
    tire->SetStepsize(step_size);
    tire->SetVisualizationType(vis_type);
}

void VehicleWheelAssembly::Synchronize(double time, const ChTerrain& terrain) {
    if (suspension) {
        chassis->Synchronize(time);
        suspension->Synchronize(time);
    } else {
        spindle->EmptyTireAccumulator();
    }

    tire->Synchronize(time, terrain);
    wheel->Synchronize();
}

void VehicleWheelAssembly::Advance(double step_size) {
    if (suspension)
        suspension->Advance(step_size);
    tire->Advance(step_size);
}

TerrainForce VehicleWheelAssembly::ReportForces(ChTerrain& terrain) const {
    return tire->ReportTireForce(&terrain);
}

// =============================================================================

ChWheelTestRig::ChWheelTestRig(ChSystem& system, std::shared_ptr<WheelAssembly> wheel)
    : m_wheel_assembly(wheel),
      m_system(system),
      m_grav(9.8),
      m_normal_load(1000),
      m_total_mass(0),
      m_mode(Mode::SUSPEND),
      m_camber_angle(0),
      m_output(false),
      m_time_delay(0),
      m_ls_actuated(false),
      m_rs_actuated(false),
      m_long_slip_constant(false),
      m_sa_fun(chrono_types::make_shared<ChFunctionConst>(0)),
      m_terrain_type(TerrainType::NONE),
      m_terrain_offset(0),
      m_terrain_height(0),
      m_step_size(1e-3),
      m_vis_type(VisualizationType::PRIMITIVES) {}

ChWheelTestRig::ChWheelTestRig(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire)
    : ChWheelTestRig(system, chrono_types::make_shared<VehicleWheelAssembly>(system, wheel, tire)) {}

ChWheelTestRig::ChWheelTestRig(ChSystem& system, std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, std::shared_ptr<ChSuspension> suspension)
    : ChWheelTestRig(system, chrono_types::make_shared<VehicleWheelAssembly>(system, suspension, wheel, tire)) {}

ChWheelTestRig::~ChWheelTestRig() {
    // Let the wheel assembly remove the items it created, then remove all rig items (in reverse order of creation)
    m_wheel_assembly->RemoveFromSystem();
    for (auto it = m_items.rbegin(); it != m_items.rend(); ++it)
        m_system.Remove(*it);
}

// -----------------------------------------------------------------------------

void ChWheelTestRig::SetLongSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_ls_fun = funct;
    m_ls_actuated = true;
}

void ChWheelTestRig::SetAngSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_rs_fun = funct;
    m_rs_actuated = true;
}

void ChWheelTestRig::SetConstantLongitudinalSlip(double long_slip, double base_speed) {
    m_ls_actuated = true;
    m_rs_actuated = true;
    m_long_slip_constant = true;
    m_long_slip = long_slip;
    m_base_speed = base_speed;
}

// -----------------------------------------------------------------------------

void ChWheelTestRig::SetTerrainRigid(const TerrainPatchSize& size, const TerrainParamsRigid& params) {
    m_terrain_type = TerrainType::RIGID;
    m_terrain_size = size;
    m_params_rigid = params;
}

void ChWheelTestRig::SetTerrainRigid(const TerrainPatchSize& size, float mu, float cr, float Y) {
    m_terrain_type = TerrainType::RIGID;
    m_terrain_size = size;

    m_params_rigid.mu = mu;
    m_params_rigid.cr = cr;
    m_params_rigid.Y = Y;
}

void ChWheelTestRig::SetTerrainSCM(const TerrainPatchSize& size, const TerrainParamsSCM& params) {
    m_terrain_type = TerrainType::SCM;
    m_terrain_size = size;
    m_params_SCM = params;
}

void ChWheelTestRig::SetTerrainSCM(const TerrainPatchSize& size,
                                   double Bekker_Kphi,
                                   double Bekker_Kc,
                                   double Bekker_n,
                                   double Mohr_cohesion,
                                   double Mohr_friction,
                                   double Janosi_shear,
                                   double grid_spacing,
                                   bool vis_mesh) {
    m_terrain_type = TerrainType::SCM;
    m_terrain_size = size;

    m_params_SCM.Bekker_Kphi = Bekker_Kphi;
    m_params_SCM.Bekker_Kc = Bekker_Kc;
    m_params_SCM.Bekker_n = Bekker_n;
    m_params_SCM.Mohr_cohesion = Mohr_cohesion;
    m_params_SCM.Mohr_friction = Mohr_friction;
    m_params_SCM.Janosi_shear = Janosi_shear;
    m_params_SCM.grid_spacing = grid_spacing;

    m_params_SCM.vis_mesh = vis_mesh;
}

void ChWheelTestRig::SetTerrainGranular(const TerrainPatchSize& size, const TerrainParamsGranular& params) {
    m_terrain_type = TerrainType::GRANULAR;
    m_terrain_size = size;

    m_params_granular = params;
}

void ChWheelTestRig::SetTerrainGranular(const TerrainPatchSize& size, double radius, double density, double friction, double cohesion, double Young_modulus) {
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

void ChWheelTestRig::Initialize(Mode mode, double drop_speed) {
    m_mode = mode;

    CreateMechanism();
    CreateTerrain();

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

void ChWheelTestRig::Advance(double step) {
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

        if (m_slip_lock)
            m_slip_lock->SetMotionAng1(chrono_types::make_shared<DelayedFun>(m_sa_fun, m_time_delay));
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

    if (m_terrain_type == TerrainType::CRM) {
#ifdef CHRONO_CRM
        m_wheel_assembly->Synchronize(time, *m_terrain.get());
        m_wheel_assembly->Advance(step);
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

void ChWheelTestRig::CreateMechanism() {
    m_system.SetGravitationalAcceleration(ChVector3d(0, 0, -m_grav));

    bool has_suspension = m_wheel_assembly->HasSuspension();

    // Set characteristic dimension
    const double dim = 0.1;

    // Set total sprung mass based on requested normal load
    if (m_grav > 0)
        m_total_mass = m_normal_load / m_grav;

    // ---- Create the main rig bodies

    m_ground_body = chrono_types::make_shared<ChBody>();
    AddItem(m_ground_body);
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetFixed(true);

    // Rig colors are muted so that the mechanism reads without competing with the wheel and terrain.
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.42f, 0.40f, 0.37f});
        mat->SetRoughness(0.55f);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(100, dim * CH_1_3, dim * CH_1_3);
        box->AddMaterial(mat);
        m_ground_body->AddVisualShape(box);
    }

    m_carrier_body = chrono_types::make_shared<ChBodyAuxRef>();
    AddItem(m_carrier_body);
    m_carrier_body->SetName("rig_carrier");
    m_carrier_body->SetPos(ChVector3d(0, 0, 0));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.56f, 0.45f, 0.35f});
        mat->SetRoughness(0.45f);

        utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier_body,              //
                                                        ChVector3d(+2 * dim, 0, 0),  //
                                                        ChVector3d(-2 * dim, 0, 0),  //
                                                        dim / 2,                     //
                                                        mat);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(dim * CH_1_3, dim * CH_1_3, 20 * dim);
        box->AddMaterial(mat);
        m_carrier_body->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -10 * dim)));
    }

    m_chassis_body = chrono_types::make_shared<ChBodyAuxRef>();
    AddItem(m_chassis_body);
    m_chassis_body->SetName("rig_chassis");
    m_chassis_body->SetPos(ChVector3d(0, 0, 0));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.63f, 0.53f, 0.43f});
        mat->SetRoughness(0.45f);

        utils::ChBodyGeometry::AddVisualizationCylinder(m_chassis_body,              //
                                                        ChVector3d(0, 0, -dim),      //
                                                        ChVector3d(0, 0, -7 * dim),  //
                                                        dim / 2,                     //
                                                        mat);
    }

    // ---- Create main joints and motors

    if (m_mode == Mode::TEST && m_ls_actuated) {
        m_lin_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        AddItem(m_lin_motor);
        m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleY(CH_PI_2)));
    } else {
        ChQuaternion<> z2x;
        z2x.SetFromAngleY(CH_PI_2);
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        AddItem(prismatic);
        prismatic->Initialize(m_carrier_body, m_ground_body, ChFrame<>(VNULL, z2x));
    }

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    AddItem(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChFrame<>(VNULL, QUNIT));

    if (m_mode == Mode::TEST) {
        m_drop_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        AddItem(m_drop_motor);
        m_drop_motor->Initialize(m_carrier_body, m_chassis_body, ChFrame<>(VNULL, QUNIT));
    }

    // ---- Let concrete rig test classes construct additional physics items, initialize and connect the wheel assembly, and adjust mass properties

    if (has_suspension)
        CreateWheelSuspensionMechanism(dim);
    else
        CreateWheelMechanism(dim);

    // ---- Create shafts and shaft motor used to actuate the wheel (if  needed)

    if (m_mode == Mode::TEST && m_rs_actuated) {
        auto chassis_shaft = chrono_types::make_shared<ChShaft>();
        chassis_shaft->SetInertia(0.1);
        AddItem(chassis_shaft);

        auto wheel_shaft = chrono_types::make_shared<ChShaft>();
        wheel_shaft->SetInertia(0.1);
        AddItem(wheel_shaft);

        auto shaft_to_chassis = chrono_types::make_shared<ChShaftBodyRotation>();
        shaft_to_chassis->Initialize(chassis_shaft, m_chassis_body, ChVector3d(0, -1, 0));
        AddItem(shaft_to_chassis);

        auto shaft_to_wheel = chrono_types::make_shared<ChShaftBodyRotation>();
        shaft_to_wheel->Initialize(wheel_shaft, m_wheel_assembly->GetHub(), ChVector3d(0, -1, 0));
        AddItem(shaft_to_wheel);

        m_rot_motor = chrono_types::make_shared<ChShaftsMotorSpeed>();
        AddItem(m_rot_motor);
        m_rot_motor->Initialize(chassis_shaft, wheel_shaft);
    }

    // ---- Set terrain offset (based on wheel center) and terrain height (below wheel)

    m_terrain_offset = m_wheel_assembly->GetHub()->GetPos().y();
    m_terrain_height = m_wheel_assembly->GetHub()->GetPos().z() - m_wheel_assembly->GetRadius() - 0.1;

    // ---- Update chassis mass to satisfy requested normal load
    if (m_grav > 0) {
        m_total_mass = m_normal_load / m_grav;
        double other_mass = m_wheel_assembly->GetMass() + (has_suspension ? 0.0 : m_slip_body->GetMass());
        double chassis_mass = m_total_mass - other_mass;
        if (chassis_mass > m_wheel_assembly->GetWheelMass()) {
            m_chassis_body->SetMass(chassis_mass);
        } else {
            std::cout << "\nWARNING!  Prescribed normal load too small. Discarded.\n" << std::endl;
        }
    }
}

void ChWheelTestRig::CreateWheelMechanism(double dim) {
    // Create the slip body
    m_slip_body = chrono_types::make_shared<ChBody>();
    AddItem(m_slip_body);
    m_slip_body->SetName("rig_slip");
    m_slip_body->SetPos(ChVector3d(0, 0, -4 * dim));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.35f, 0.32f, 0.29f});
        mat->SetRoughness(0.6f);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(4 * dim, dim, 4 * dim);
        box->AddMaterial(mat);
        m_slip_body->AddVisualShape(box);
    }

    // Initialize the wheel assembly
    ChQuaternion<> qc;
    qc.SetFromAngleX(-m_camber_angle);
    m_wheel_assembly->Initialize(nullptr, ChFramed(ChVector3d(0, 3 * dim, -4 * dim), qc), (m_mode == Mode::SUSPEND), m_step_size, m_vis_type);

    // Adjust rig body mass and inertia commensurate with those of the wheel
    double mass = m_wheel_assembly->GetWheelMass();
    const double radius = m_wheel_assembly->GetRadius();
    ChMatrix33d inertia = 0.25 * mass * ChSphere::CalcGyration(radius);

    m_carrier_body->SetMass(mass);
    m_carrier_body->SetInertia(inertia);

    m_chassis_body->SetMass(mass);
    m_chassis_body->SetInertia(inertia);

    m_slip_body->SetMass(mass);
    m_slip_body->SetInertia(inertia);

    // Create chassis to slip body connection which allows controlling slip angle
    m_slip_lock = chrono_types::make_shared<ChLinkLockLock>();
    AddItem(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChFrame<>(VNULL, QUNIT));
    m_slip_lock->SetMotionAxis(ChVector3d(0, 0, 1));

    // Connect wheel to slip body
    ChQuaternion<> z2y;
    z2y.SetFromAngleX(-CH_PI_2 - m_camber_angle);
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    AddItem(revolute);
    revolute->Initialize(m_wheel_assembly->GetHub(), m_slip_body, ChFrame<>(ChVector3d(0, 3 * dim, -4 * dim), z2y));
}

void ChWheelTestRig::CreateWheelSuspensionMechanism(double dim) {
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.63f, 0.53f, 0.43f});
        mat->SetRoughness(0.45f);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(4 * dim, dim, 4 * dim);
        box->AddMaterial(mat);
        m_chassis_body->AddVisualShape(box, ChFramed(ChVector3d(0, 0, -4 * dim), QUNIT));
    }

    // Initialize the suspension and wheel assembly
    m_wheel_assembly->Initialize(m_chassis_body, ChFramed(ChVector3d(0, 0, -4 * dim), QUNIT), (m_mode == Mode::SUSPEND), m_step_size, m_vis_type);

    // Adjust rig body mass and inertia commensurate with those of the wheel
    double mass = m_wheel_assembly->GetWheelMass();
    const double radius = m_wheel_assembly->GetRadius();
    ChMatrix33d inertia = 0.25 * mass * ChSphere::CalcGyration(radius);

    m_carrier_body->SetMass(mass);
    m_carrier_body->SetInertia(inertia);

    m_chassis_body->SetMass(mass);
    m_chassis_body->SetInertia(inertia);
}

// -----------------------------------------------------------------------------

void ChWheelTestRig::CreateTerrain() {
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

void ChWheelTestRig::CreateTerrainSCM() {
    ChVector3d location(m_terrain_size.length / 2 - 2 * m_wheel_assembly->GetRadius(), m_terrain_offset, m_terrain_height);

    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&m_system, m_params_SCM.vis_mesh);
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

void ChWheelTestRig::CreateTerrainRigid() {
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

void ChWheelTestRig::CreateTerrainGranular() {
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

void ChWheelTestRig::SetTerrainCRM(const TerrainPatchSize& size, double spacing, double density, double Young_modulus, double friction, double cohesion) {
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

void ChWheelTestRig::SetTerrainCRM(const TerrainPatchSize& size, const TerrainParamsCRM& params) {
    m_terrain_type = TerrainType::CRM;
    m_terrain_size = size;

    m_params_crm = params;
}

void ChWheelTestRig::SetWheelActiveDomain(const ChAABB& aabb) {
    m_wheel_AABB = aabb;
}

void ChWheelTestRig::SetWheelActiveDomain() {
    auto corner = ChVector3d(m_wheel_assembly->GetRadius(), m_wheel_assembly->GetWidth() / 2, m_wheel_assembly->GetRadius());
    m_wheel_AABB.min = -2.5 * corner;
    m_wheel_AABB.max = +2.5 * corner;
}

void ChWheelTestRig::CreateTerrainCRM() {
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

const ChAABB& ChWheelTestRig::GetWheelActiveDomain() const {
    return m_wheel_AABB;
}

const ChAABB& ChWheelTestRig::GetTerrainSPHBoundingBox() const {
    auto crm = std::dynamic_pointer_cast<CRMTerrain>(m_terrain);
    if (!crm) {
        std::cerr << "ERROR: GetTerrainSPHBoundingBox called for non-CRM terrain." << std::endl;
        throw std::runtime_error("ERROR: GetTerrainSPHBoundingBox called for non-CRM terrain.");
    }
    return crm->GetSPHBoundingBox();
}

#endif

// -----------------------------------------------------------------------------

void ChWheelTestRig::GetSuggestedCollisionSettings(double& collision_envelope, ChVector3i& collision_bins) const {
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

TerrainForce ChWheelTestRig::ReportWheelForce() const {
    if (!m_output)
        return TerrainForce();

    return m_wheel_assembly->ReportForces(*m_terrain);
}

double ChWheelTestRig::GetDBP() const {
    if (!m_output)
        return 0;

    return -m_lin_motor->GetMotorForce();
}

double ChWheelTestRig::GetLongitudinalSlip() const {
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

double ChWheelTestRig::GetSlipAngle() const {
    if (!m_output)
        return 0;

    auto dir = m_wheel_assembly->GetHub()->GetRotMat().GetAxisY();
    double slip_angle = std::atan(dir.x() / dir.y());
    return slip_angle;
}

double ChWheelTestRig::GetCamberAngle() const {
    if (!m_output)
        return 0;

    auto dir = m_wheel_assembly->GetHub()->GetRotMat().GetAxisY();
    double camber_angle = std::atan(-dir.z());
    return camber_angle;
}

}  // end namespace vehicle
}  // end namespace chrono
