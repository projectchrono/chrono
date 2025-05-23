// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/robot/robosimian/RoboSimian.h"

namespace chrono {
namespace robosimian {

// =============================================================================

// Concrete Link types
//     mesh_name, offset, color, mass, com, inertia_xx, inertia_xy, shapes

const Link FtsLink("robosim_fts",
                   ChVector3d(0, 0, 0),
                   ChColor(1.0f, 0.0f, 0.0f),
                   0.0,
                   ChVector3d(0, 0, 0),
                   ChVector3d(0, 0, 0),
                   ChVector3d(0, 0, 0),
                   {});

const Link PitchLink("robosim_pitch_link",
                     ChVector3d(0, 0, 0),
                     ChColor(0.4f, 0.7f, 0.4f),
                     0.428770,
                     ChVector3d(-0.050402, 0.012816, 0.000000),
                     ChVector3d(0.000670, 0.000955, 0.000726),
                     ChVector3d(0.000062, 0.000000, 0.000000),
                     {CylinderShape(ChVector3d(-0.07, 0, 0), QuatFromAngleY(CH_PI_2), 0.055, 0.07)});

const Link RollLink("robosim_roll_link",
                    ChVector3d(0, 0, 0),
                    ChColor(0.7f, 0.4f, 0.4f),
                    4.078540,
                    ChVector3d(0.066970, -0.090099, -0.000084),
                    ChVector3d(0.010580, 0.025014, 0.031182),
                    ChVector3d(-0.008765, -0.000002, 0.000007),
                    {CylinderShape(ChVector3d(0.065, -0.12, 0), QuatFromAngleY(CH_PI_2), 0.055, 0.24),
                     CylinderShape(ChVector3d(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLast("robosim_roll_link",
                        ChVector3d(0, 0, 0),
                        ChColor(0.7f, 0.4f, 0.4f),
                        4.078540,
                        ChVector3d(0.066970, -0.090099, -0.000084),
                        ChVector3d(0.010580, 0.025014, 0.031182),
                        ChVector3d(-0.008765, -0.000002, 0.000007),
                        {CylinderShape(ChVector3d(0.105, -0.12, 0), QuatFromAngleY(CH_PI_2), 0.055, 0.32),
                         CylinderShape(ChVector3d(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLastWheel("robosim_roll_link_w_wheel",
                             ChVector3d(0, 0, 0),
                             ChColor(0.7f, 0.4f, 0.4f),
                             4.078540,
                             ChVector3d(0.066970, -0.090099, -0.000084),
                             ChVector3d(0.010580, 0.025014, 0.031182),
                             ChVector3d(-0.008765, -0.000002, 0.000007),
                             {CylinderShape(ChVector3d(0.105, -0.12, 0), QuatFromAngleY(CH_PI_2), 0.055, 0.32),
                              CylinderShape(ChVector3d(0.0, -0.035, 0), QuatFromAngleX(CH_PI_2), 0.055, 0.075),
                              CylinderShape(ChVector3d(0.0, -0.19, 0), QuatFromAngleX(CH_PI_2), 0.080, 0.0375)});

const Link FtAdapterLink("robosim_ft_adapter",
                         ChVector3d(0, 0, 0),
                         ChColor(0.4f, 0.4f, 0.4f),
                         0.253735,
                         ChVector3d(-0.00531, -0.00060, -0.001873),
                         ChVector3d(0.00042, 0.00024, 0.00023),
                         ChVector3d(0, 0, 0),
                         {});

const Link FtLink("robosim_force_torque_sensor",
                  ChVector3d(0, 0, 0),
                  ChColor(0.4f, 0.4f, 0.4f),
                  0.195418,
                  ChVector3d(-0.000135, 0.000118, 0.000084),
                  ChVector3d(0.000086, 0.000056, 0.000057),
                  ChVector3d(0, 0, 0),
                  {});

const Link WheelMountLink("robosim_wheel_mount",
                          ChVector3d(0.12024, 0, 0),
                          ChColor(0.4f, 0.7f, 0.4f),
                          3.1775,
                          ChVector3d(-0.005260, 0.042308, 0.000088),
                          ChVector3d(0.010977, 0.005330, 0.011405),
                          ChVector3d(0.000702, 0.000026, -0.000028),
                          {CylinderShape(ChVector3d(0.12024, 0.02, 0), QuatFromAngleX(CH_PI_2), 0.0545, 0.175)});

const Link WheelLink("robosim_wheel",
                     ChVector3d(0, 0, 0),
                     ChColor(0.6f, 0.6f, 0.6f),
                     1.499326,
                     ChVector3d(0.0, 0.0, -0.000229),
                     ChVector3d(0.006378, 0.006377, 0.009155),
                     ChVector3d(0, 0, 0),
                     {CylinderShape(ChVector3d(0, 0, 0), QUNIT, 0.12, 0.123)});

// List of links for front and rear legs
//     name, link, body included?

const int num_links = 11;

const LinkData front_links[] = {
    {"link0", FtsLink, false},                //
    {"link1", PitchLink, true},               //
    {"link2", RollLink, true},                //
    {"link3", PitchLink, true},               //
    {"link4", RollLink, true},                //
    {"link5", PitchLink, true},               //
    {"link6", RollLinkLast, true},            //
    {"ftadapter_link", FtAdapterLink, true},  //
    {"ft_link", FtLink, true},                //
    {"link7", WheelMountLink, true},          //
    {"link8", WheelLink, true}                //
};

const LinkData rear_links[] = {
    {"link0", FtsLink, false},                //
    {"link1", PitchLink, true},               //
    {"link2", RollLink, true},                //
    {"link3", PitchLink, true},               //
    {"link4", RollLink, true},                //
    {"link5", PitchLink, true},               //
    {"link6", RollLinkLastWheel, true},       //
    {"ftadapter_link", FtAdapterLink, true},  //
    {"ft_link", FtLink, true},                //
    {"link7", WheelMountLink, true},          //
    {"link8", WheelLink, true}                //
};

// List of joints in a limb chain
//     name, parent_link, child_link, fixed?, xyz, rpy, axis

const int num_joints = 10;

const JointData joints[] = {

    {"joint1", "link0", "link1", false, ChVector3d(0.17203, 0.00000, 0.00000), ChVector3d(3.14159, 0.00000, 0.00000),
     ChVector3d(1, 0, 0)},

    {"joint2", "link1", "link2", false, ChVector3d(0.00000, 0.00000, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(0, -1, 0)},

    {"joint3", "link2", "link3", false, ChVector3d(0.28650, -0.11700, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(1, 0, 0)},

    {"joint4", "link3", "link4", false, ChVector3d(0.00000, 0.00000, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(0, -1, 0)},

    {"joint5", "link4", "link5", false, ChVector3d(0.28650, -0.11700, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(1, 0, 0)},

    {"joint6", "link5", "link6", false, ChVector3d(0.00000, 0.00000, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(0, -1, 0)},

    {"ftadapter_joint", "link6", "ftadapter_link", true, ChVector3d(0.20739, -0.12100, 0.00000),
     ChVector3d(0.00000, 0.00000, 0.00000), ChVector3d(1, 0, 0)},

    {"ft_joint", "ftadapter_link", "ft_link", true, ChVector3d(0.0263755, 0.00000, 0.00000),
     ChVector3d(0.00000, 0.00000, 0.00000), ChVector3d(1, 0, 0)},

    {"joint7", "link6", "link7", false, ChVector3d(0.19250, -0.11700, 0.00000), ChVector3d(0.00000, 0.00000, 0.00000),
     ChVector3d(1, 0, 0)},

    {"joint8", "link7", "link8", false, ChVector3d(0.12024, 0.17200, 0.00000), ChVector3d(-1.57000, 0.00000, 0.00000),
     ChVector3d(0, 0, 1)}

};

constexpr int Robosimian_body_tag = 100;

// =============================================================================

// Convert a triplet (roll-pitch-yaw) to a quaternion
ChQuaternion<> rpy2quat(const ChVector3d& rpy) {
    return QuatFromAngleZ(rpy.z()) * QuatFromAngleY(rpy.y()) * QuatFromAngleX(rpy.x());
}

// =============================================================================

// Calculate a coordinate system with the Z direction along 'axis' (given in 'base').
// Implicit assumption: 'axis' is always along X, Y, or Z.
ChFrame<> calcJointFrame(const ChFrame<>& base, const ChVector3d& axis) {
    ChVector3d u;
    ChVector3d v;
    ChVector3d w = axis;
    if (std::abs(axis.x()) > 0.5) {
        v = ChVector3d(0, 1, 0);
        u = Vcross(v, w);
    } else {
        u = ChVector3d(1, 0, 0);
        v = Vcross(w, u);
    }
    ChMatrix33<> A;
    A.SetFromDirectionAxes(u, v, w);
    ChMatrix33<> B = base.GetRotMat() * A;
    return ChFrame<>(base.GetPos(), B.GetQuaternion());
}

// =============================================================================

// Convert the specified inertia properties into the centroidal reference frame.
// It is assumed that the centroidal frame is parallel with the reference frame.
class InertiaConverter {
  public:
    InertiaConverter(double mass, const ChVector3d& com, const ChVector3d& inertia_xx, const ChVector3d& inertia_xy) {
        // Inertia matrix (wrt reference frame)
        ChMatrix33<> J(inertia_xx, inertia_xy);

        // Convert inertia to centroidal frame (parallel-axis theorem, no rotation)
        ChVector3d diag(com.y() * com.y() + com.z() * com.z(),  //
                        com.x() * com.x() + com.z() * com.z(),  //
                        com.x() * com.x() + com.y() * com.y());
        ChVector3d off_diag(-com.x() * com.y(),  //
                            -com.x() * com.z(),  //
                            -com.y() * com.z());
        ChMatrix33<> offset(diag, off_diag);

        ChMatrix33<> Jc = J - offset * mass;

        // Extract centroidal moments and products of inertia
        m_inertia_xx.x() = Jc(0, 0);
        m_inertia_xx.y() = Jc(1, 1);
        m_inertia_xx.z() = Jc(2, 2);

        m_inertia_xy.x() = Jc(0, 1);
        m_inertia_xy.y() = Jc(0, 2);
        m_inertia_xy.z() = Jc(1, 2);

        /*
        std::cout << mass <<                                                            // mass
            " " << com.x() << "  " << com.y() << " " << com.z() <<                      // COM offset
            " " << inertia_xx.x() << " " << inertia_xx.y() << " " << inertia_xx.z() <<  // moments (reference frame)
            " " << inertia_xy.x() << " " << inertia_xy.y() << " " << inertia_xy.z() <<  // products (reference frame)
            " " << m_inertia_xx.x() << " " << m_inertia_xx.y() << " " << m_inertia_xx.z() <<  // moments (centroidal)
            " " << m_inertia_xy.x() << " " << m_inertia_xy.y() << " " << m_inertia_xy.z() <<  // products (centroidal)
            std::endl;
        */
    }

    ChVector3d m_inertia_xx;  ///< moments of inertia (centroidal)
    ChVector3d m_inertia_xy;  ///< products of inertia (centroidal)
};

// =============================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager();
    void Process(RoboSimian* robot);

  private:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector3d& react_forces,
                                 const ChVector3d& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override;

    int m_num_contacts;
};

ContactManager::ContactManager() {}

void ContactManager::Process(RoboSimian* robot) {
    std::cout << "Report contacts" << std::endl;
    m_num_contacts = 0;
    std::shared_ptr<ContactManager> shared_this(this, [](ContactManager*) {});
    robot->GetSystem()->GetContactContainer()->ReportAllContacts(shared_this);
    std::cout << "  total actual contacts: " << m_num_contacts << std::endl << std::endl;
}

bool ContactManager::OnReportContact(const ChVector3d& pA,
                                     const ChVector3d& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const ChVector3d& react_forces,
                                     const ChVector3d& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    // Only report contacts with negative penetration (i.e. actual contacts).
    if (distance >= 0)
        return true;

    auto bodyA = dynamic_cast<ChBodyAuxRef*>(modA);
    auto bodyB = dynamic_cast<ChBodyAuxRef*>(modB);

    // Filter robot bodies based on their IDs.
    bool a = (bodyA && bodyA->GetTag() == Robosimian_body_tag);
    bool b = (bodyB && bodyB->GetTag() == Robosimian_body_tag);

    if (!a && !b)
        return true;

    std::cout << "   " << (a ? bodyA->GetName() : "other") << " - " << (b ? bodyB->GetName() : "other")
              << std::endl;

    m_num_contacts++;

    return true;
}

// =============================================================================

// Callback class for modifying composite material properties.
// Notes:
//   - currently, only ChSystemMulticoreNSC support user-provided callbacks for overwriting composite material
//   properties
//   - as such, this functor class is only created when using NSC frictional contact
//   - composite material properties are modified only for contacts involving the sled or one of the wheels
//   - in these cases, the friction coefficient is set to the user-specified value
//   - cohesion, restitution, and compliance are set to 0

class ContactMaterial : public ChContactContainer::AddContactCallback {
  public:
    ContactMaterial(RoboSimian* robot) : m_robot(robot) {
        std::shared_ptr<ContactMaterial> shared_this(this, [](ContactMaterial*) {});
        m_robot->GetSystem()->GetContactContainer()->RegisterAddContactCallback(shared_this);
    }

    virtual void OnAddContact(const ChCollisionInfo& contactinfo, ChContactMaterialComposite* const material) override {
        //// TODO: currently, only NSC multicore systems support user override of composite materials.
        auto mat = static_cast<ChContactMaterialCompositeNSC* const>(material);

        // Contactables in current collision pair
        auto contactableA = contactinfo.modelA->GetContactable();
        auto contactableB = contactinfo.modelB->GetContactable();

        // Overwrite composite material properties if collision involves the sled body
        auto sled = m_robot->GetSledBody().get();
        if (contactableA == sled || contactableB == sled) {
            mat->static_friction = m_robot->m_sled_friction;
            mat->sliding_friction = m_robot->m_sled_friction;
            mat->cohesion = 0;
            mat->restitution = 0;
            mat->compliance = 0;
            return;
        }

        // Overwrite composite material properties if collision involves a wheel body
        auto wheel0 = m_robot->GetWheelBody(FR).get();
        auto wheel1 = m_robot->GetWheelBody(FL).get();
        auto wheel2 = m_robot->GetWheelBody(RR).get();
        auto wheel3 = m_robot->GetWheelBody(RL).get();
        if (contactableA == wheel0 || contactableB == wheel0 || contactableA == wheel1 || contactableB == wheel1 ||
            contactableA == wheel2 || contactableB == wheel2 || contactableA == wheel3 || contactableB == wheel3) {
            mat->static_friction = m_robot->m_wheel_friction;
            mat->sliding_friction = m_robot->m_wheel_friction;
            mat->cohesion = 0;
            mat->restitution = 0;
            mat->compliance = 0;
            return;
        }
    }

  private:
    RoboSimian* m_robot;
};

// =============================================================================

RoboSimian::RoboSimian(ChContactMethod contact_method, bool has_sled, bool fixed)
    : m_owns_system(true),
      m_wheel_mode(ActuationMode::SPEED),
      m_contact_reporter(new ContactManager),
      m_material_override(nullptr),
      m_sled_friction(0.8f),
      m_wheel_friction(0.8f),
      m_outdir(""),
      m_root("results") {
    m_system = (contact_method == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                        : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Solver settings
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    m_system->GetSolver()->AsIterative()->SetMaxIterations(150);

    Create(has_sled, fixed);

    //// TODO: currently, only NSC multicore systems support user override of composite materials
    if (contact_method == ChContactMethod::NSC) {
        m_material_override = new ContactMaterial(this);
    }
}

RoboSimian::RoboSimian(ChSystem* system, bool has_sled, bool fixed)
    : m_owns_system(false),
      m_system(system),
      m_wheel_mode(ActuationMode::SPEED),
      m_contact_reporter(new ContactManager),
      m_material_override(nullptr),
      m_sled_friction(0.8f),
      m_wheel_friction(0.8f),
      m_outdir(""),
      m_root("results") {
    Create(has_sled, fixed);

    //// TODO: currently, only NSC multicore systems support user override of composite materials
    if (system->GetContactMethod() == ChContactMethod::NSC) {
        m_material_override = new ContactMaterial(this);
    }
}

RoboSimian::~RoboSimian() {
    if (m_owns_system)
        delete m_system;
    delete m_contact_reporter;
    delete m_material_override;
}

std::shared_ptr<ChContactMaterial> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.8f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

void RoboSimian::Create(bool has_sled, bool fixed) {
    auto contact_method = m_system->GetContactMethod();

    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials (all with default properties)
    m_chassis_material = DefaultContactMaterial(contact_method);
    m_sled_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);
    m_link_material = DefaultContactMaterial(contact_method);
    m_wheelDD_material = DefaultContactMaterial(contact_method);

    m_chassis = chrono_types::make_shared<RS_Chassis>("chassis", fixed, m_chassis_material, m_system);

    if (has_sled)
        m_sled = chrono_types::make_shared<RS_Sled>("sled", m_sled_material, m_system);

    m_limbs.push_back(
        chrono_types::make_shared<RS_Limb>("limb1", FR, front_links, m_wheel_material, m_link_material, m_system));
    m_limbs.push_back(
        chrono_types::make_shared<RS_Limb>("limb2", RR, rear_links, m_wheel_material, m_link_material, m_system));
    m_limbs.push_back(
        chrono_types::make_shared<RS_Limb>("limb3", RL, rear_links, m_wheel_material, m_link_material, m_system));
    m_limbs.push_back(
        chrono_types::make_shared<RS_Limb>("limb4", FL, front_links, m_wheel_material, m_link_material, m_system));

    // The differential-drive wheels will be removed from robosimian
    ////m_wheel_left = chrono_types::make_shared<RS_WheelDD>("dd_wheel_left", 2, m_wheelDD_material, m_system);
    ////m_wheel_right = chrono_types::make_shared<RS_WheelDD>("dd_wheel_right", 3, m_wheelDD_material, m_system);

    // Default visualization: COLLISION shapes
    SetVisualizationTypeChassis(VisualizationType::COLLISION);
    SetVisualizationTypeSled(VisualizationType::COLLISION);
    SetVisualizationTypeLimbs(VisualizationType::COLLISION);
    SetVisualizationTypeWheels(VisualizationType::COLLISION);
}

void RoboSimian::Initialize(const ChCoordsys<>& pos) {
    m_chassis->Initialize(pos);

    if (m_sled)
        m_sled->Initialize(m_chassis->m_body, ChVector3d(0.0, 0.0, 0.21), ChVector3d(1.570796, 0, 0));

    m_limbs[FR]->Initialize(m_chassis->m_body, ChVector3d(+0.29326, +0.20940, 0.03650),
                            ChVector3d(0.00000, -1.57080, -0.26180), CollisionFamily::LIMB_FR, m_wheel_mode);
    m_limbs[RR]->Initialize(m_chassis->m_body, ChVector3d(-0.29326, +0.20940, 0.03650),
                            ChVector3d(0.00000, -1.57080, +0.26180), CollisionFamily::LIMB_RR, m_wheel_mode);
    m_limbs[RL]->Initialize(m_chassis->m_body, ChVector3d(-0.29326, -0.20940, 0.03650),
                            ChVector3d(0.00000, -1.57080, 2.87979), CollisionFamily::LIMB_RL, m_wheel_mode);
    m_limbs[FL]->Initialize(m_chassis->m_body, ChVector3d(+0.29326, -0.20940, 0.03650),
                            ChVector3d(0.00000, -1.57080, 3.40339), CollisionFamily::LIMB_FL, m_wheel_mode);

    ////m_wheel_left->Initialize(m_chassis->m_body, ChVector3d(-0.42943, -0.19252, 0.06380),
    ////                         ChVector3d(0.00000, +1.57080, -1.57080));
    ////m_wheel_right->Initialize(m_chassis->m_body, ChVector3d(-0.42943, +0.19252, 0.06380),
    ////                          ChVector3d(0.00000, -1.57080, -1.57080));

    // Create output files
    for (int i = 0; i < 4; i++) {
        m_outf[i].open(m_outdir + "/" + m_root + "_limb" + std::to_string(i) + ".dat");
        m_outf[i].precision(7);
        m_outf[i] << std::scientific;
    }
}

void RoboSimian::EnableCollision(int flags) {
    m_chassis->EnableCollision((flags & static_cast<int>(CollisionFlags::CHASSIS)) != 0);

    if (m_sled)
        m_sled->EnableCollision((flags & static_cast<int>(CollisionFlags::SLED)) != 0);

    for (auto limb : m_limbs) {
        limb->SetCollideLinks((flags & static_cast<int>(CollisionFlags::LIMBS)) != 0);
        limb->SetCollideWheel((flags & static_cast<int>(CollisionFlags::WHEELS)) != 0);
    }
}

void RoboSimian::SetFrictionCoefficients(float sled_friction, float wheel_friction) {
    m_sled_friction = sled_friction;
    m_wheel_friction = wheel_friction;
}

void RoboSimian::SetVisualizationTypeChassis(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeSled(VisualizationType vis) {
    if (m_sled)
        m_sled->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimb(LimbID id, VisualizationType vis) {
    m_limbs[id]->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimbs(VisualizationType vis) {
    for (auto limb : m_limbs)
        limb->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeWheels(VisualizationType vis) {
    ////m_wheel_left->SetVisualizationType(vis);
    ////m_wheel_right->SetVisualizationType(vis);
}

void RoboSimian::SetDriver(std::shared_ptr<RS_Driver> driver) {
    m_driver = driver;
}

void RoboSimian::SetOutputDirectory(const std::string& outdir, const std::string& root) {
    m_outdir = outdir;
    m_root = root;
}

void RoboSimian::Activate(LimbID id, const std::string& motor_name, double time, double val) {
    m_limbs[id]->Activate(motor_name, time, val);
}

void RoboSimian::DoStepDynamics(double step) {
    static double w[4] = {0, 0, 0, 0};

    if (m_driver) {
        // Update driver
        double time = m_system->GetChTime();
        m_driver->Update(time);

        // Get driver activations
        Actuation actuation = m_driver->GetActuation();

        if (m_wheel_mode == ActuationMode::ANGLE) {
            // Overwrite wheel actuations (angle instead of speed)
            for (int i = 0; i < 4; i++) {
                double speed = actuation[i][7];
                double pos = w[i] + speed * step;
                actuation[i][7] = pos;
                w[i] = pos;
            }
        }

        // Apply activations to limbs
        for (int i = 0; i < 4; i++)
            m_limbs[i]->Activate(time, actuation[i]);
    }

    // Advance system state
    m_system->DoStepDynamics(step);
}

void RoboSimian::Translate(const ChVector3d& shift) {
    m_chassis->Translate(shift);
    if (m_sled)
        m_sled->Translate(shift);
    m_limbs[FR]->Translate(shift);
    m_limbs[RR]->Translate(shift);
    m_limbs[RL]->Translate(shift);
    m_limbs[FL]->Translate(shift);
}

void RoboSimian::ReportContacts() {
    m_contact_reporter->Process(this);
}

void RoboSimian::Output() {
    for (int i = 0; i < 4; i++) {
        // Current motor angles
        m_outf[i] << m_system->GetChTime();
        std::array<double, 8> angles = m_limbs[i]->GetMotorAngles();
        for (auto v : angles) {
            m_outf[i] << "  " << v;
        }
        // Current motor angular speeds
        std::array<double, 8> speeds = m_limbs[i]->GetMotorOmegas();
        for (auto v : speeds) {
            m_outf[i] << "  " << v;
        }
        // Current motor (reaction) torques
        std::array<double, 8> torques = m_limbs[i]->GetMotorTorques();
        for (auto v : torques) {
            m_outf[i] << "  " << v;
        }
        // Actuations data
        m_limbs[i]->GetMotorActuations(angles, speeds);
        for (auto v : angles) {
            m_outf[i] << "  " << v;
        }
        for (auto v : speeds) {
            m_outf[i] << "  " << v;
        }
        m_outf[i] << std::endl;
    }
}

double RoboSimian::GetMass() const {
    return GetChassisBody()->GetMass() + GetSledBody()->GetMass() +  //
           m_limbs[FR]->GetMass() + m_limbs[FL]->GetMass() +         //
           m_limbs[RR]->GetMass() + m_limbs[RL]->GetMass();
}

// =============================================================================

class ax {
  public:
    double operator()(const double& v) { return a * v; }
    double a;
};

class axpby {
  public:
    double operator()(const double& v1, const double& v2) { return a1 * v1 + a2 * v2; }
    double a1;
    double a2;
};

const std::string RS_Driver::m_phase_names[] = {"POSE", "HOLD", "START", "CYCLE", "STOP"};

RS_Driver::RS_Driver(const std::string& filename_start,
                     const std::string& filename_cycle,
                     const std::string& filename_stop,
                     bool repeat)
    : m_repeat(repeat), m_time_pose(0), m_time_hold(0), m_offset(0), m_phase(POSE), m_callback(nullptr) {
    assert(!filename_cycle.empty());
    m_ifs_cycle.open(filename_cycle);

    if (!filename_start.empty()) {
        m_ifs_start.open(filename_start);
        m_ifs = &m_ifs_start;
    } else {
        m_ifs = &m_ifs_cycle;
    }

    if (!filename_stop.empty()) {
        m_ifs_stop.open(filename_stop);
    }

    LoadDataLine(m_time_1, m_actuations_1);
    LoadDataLine(m_time_2, m_actuations_2);
}

RS_Driver::~RS_Driver() {}

void RS_Driver::SetTimeOffsets(double time_pose, double time_hold) {
    m_time_pose = time_pose;
    m_time_hold = time_hold;
    m_offset = time_pose + time_hold;
}

void RS_Driver::LoadDataLine(double& time, Actuation& activations) {
    *m_ifs >> time;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            *m_ifs >> activations[i][j];
        }
    }
}

void RS_Driver::Update(double time) {
    bool ext_actuation = false;
    // In the POSE phase, use a logistic function to reach first data entry
    if (m_phase == POSE) {
        ax op;
        double x = 20 * (time / m_time_pose) - 10;
        op.a = std::exp(x) / (1 + std::exp(x));
        for (int i = 0; i < 4; i++) {
            std::transform(m_actuations_1[i].begin(), m_actuations_1[i].end(), m_actuations[i].begin(), op);
        }
        if (time >= m_time_pose) {
            m_phase = HOLD;
            std::cout << "time = " << time << "  Switch to phase: " << GetCurrentPhase() << std::endl;
            if (m_callback)
                m_callback->OnPhaseChange(POSE, m_phase);
        }
        return;
    }

    // In the HOLD phase, always use the first data entry
    if (m_phase == HOLD) {
        m_actuations = m_actuations_1;
        if (time >= m_offset) {
            m_phase = (m_ifs_start.is_open()) ? START : CYCLE;
            std::cout << "time = " << time << "  Switch to phase: " << GetCurrentPhase() << std::endl;
            if (m_callback)
                m_callback->OnPhaseChange(HOLD, m_phase);
        }
        return;
    }

    // Offset time
    double t = time - m_offset;

    switch (m_phase) {
        case START:
            while (t > m_time_2) {
                m_time_1 = m_time_2;
                m_actuations_1 = m_actuations_2;
                if (!m_ifs->eof()) {
                    LoadDataLine(m_time_2, m_actuations_2);
                } else {
                    m_phase = CYCLE;
                    m_ifs = &m_ifs_cycle;
                    LoadDataLine(m_time_1, m_actuations_1);
                    LoadDataLine(m_time_2, m_actuations_2);
                    m_offset = time;
                    std::cout << "time = " << time << "  Switch to phase: " << GetCurrentPhase() << std::endl;
                    if (m_callback)
                        m_callback->OnPhaseChange(START, CYCLE);
                    return;
                }
            }

            break;

        case CYCLE:
            if (!driven) {
                while (t > m_time_2) {
                    m_time_1 = m_time_2;
                    m_actuations_1 = m_actuations_2;
                    if (m_ifs->eof()) {
                        if (m_repeat) {
                            m_ifs->clear();
                            m_ifs->seekg(0);
                            LoadDataLine(m_time_1, m_actuations_1);
                            LoadDataLine(m_time_2, m_actuations_2);
                            m_offset = time;
                            std::cout << "time = " << time << " New cycle" << std::endl;
                            if (m_callback)
                                m_callback->OnPhaseChange(CYCLE, CYCLE);
                        }
                        return;
                    }
                    LoadDataLine(m_time_2, m_actuations_2);
                }
            } else {
                ext_actuation = true;
            }

            break;

        case STOP:
            //// TODO
            break;

        default:
            break;
    }

    // Interpolate  v = alpha_1 * v_1 + alpha_2 * v_2
    if (!ext_actuation) {
        axpby op;
        op.a1 = (t - m_time_2) / (m_time_1 - m_time_2);
        op.a2 = (t - m_time_1) / (m_time_2 - m_time_1);
        for (int i = 0; i < 4; i++) {
            std::transform(m_actuations_1[i].begin(), m_actuations_1[i].end(), m_actuations_2[i].begin(),
                           m_actuations[i].begin(), op);
        }
    }
}

// =============================================================================

void RS_DriverCallback::OnPhaseChange(RS_Driver::Phase old_phase, RS_Driver::Phase new_phase) {
    if (new_phase == RS_Driver::HOLD) {
        auto& fl = m_robot->GetWheelPos(FL);
        auto& fr = m_robot->GetWheelPos(FR);
        auto& rl = m_robot->GetWheelPos(RL);
        auto& rr = m_robot->GetWheelPos(RR);
        std::cout << "  wheel FL: " << fl.x() << "  " << fl.y() << std::endl;
        std::cout << "  wheel FR: " << fr.x() << "  " << fr.y() << std::endl;
        std::cout << "  wheel RL: " << rl.x() << "  " << rl.y() << std::endl;
        std::cout << "  wheel RR: " << rr.x() << "  " << rr.y() << std::endl;
    }
    if (new_phase == RS_Driver::CYCLE && old_phase != RS_Driver::CYCLE) {
        m_start_x = m_robot->GetChassisPos().x();
        m_start_time = m_robot->GetSystem()->GetChTime();
    }
}

// =============================================================================

RS_Part::RS_Part(const std::string& name, std::shared_ptr<ChContactMaterial> mat, ChSystem* system)
    : m_name(name), m_mat(mat) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(name + "_body");
}

void RS_Part::SetVisualizationType(VisualizationType vis) {
    if (m_body->GetVisualModel())
        m_body->GetVisualModel()->Clear();

    AddVisualizationAssets(vis);
}

void RS_Part::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (vis == VisualizationType::MESH) {
        auto vis_mesh_file = GetChronoDataFile("robot/robosimian/obj/" + m_mesh_name + ".obj");
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, false);
        //// HACK: a trimesh visual asset ignores transforms! Explicitly offset vertices.
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_mesh_name);
        ////trimesh_shape->Pos = m_offset;
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(m_color);
        m_body->AddVisualShape(trimesh_shape, ChFrame<>(m_offset, QUNIT));
        return;
    }

    for (const auto& box : m_boxes) {
        auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.m_dims);
        box_shape->SetColor(m_color);
        m_body->AddVisualShape(box_shape, ChFrame<>(box.m_pos, box.m_rot));
    }

    for (const auto& cyl : m_cylinders) {
        auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.m_radius, cyl.m_length);
        cyl_shape->SetColor(m_color);
        m_body->AddVisualShape(cyl_shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
    }

    for (const auto& sphere : m_spheres) {
        auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.m_radius);
        sphere_shape->SetColor(m_color);
        m_body->AddVisualShape(sphere_shape, ChFrame<>(sphere.m_pos, QUNIT));
    }

    for (const auto& mesh : m_meshes) {
        auto vis_mesh_file = GetChronoDataFile("robot/robosimian/obj/" + mesh.m_name + ".obj");
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, false);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(mesh.m_name);
        ////trimesh_shape->Pos = m_offset;
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(m_color);
        m_body->AddVisualShape(trimesh_shape, ChFrame<>(mesh.m_pos, mesh.m_rot));
    }
}

void RS_Part::AddCollisionShapes() {
    for (const auto& sphere : m_spheres) {
        auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(m_mat, sphere.m_radius);
        m_body->AddCollisionShape(shape, ChFrame<>(sphere.m_pos, QUNIT));
    }
    for (const auto& box : m_boxes) {
        auto shape = chrono_types::make_shared<ChCollisionShapeBox>(m_mat, box.m_dims);
        m_body->AddCollisionShape(shape, ChFrame<>(box.m_pos, box.m_rot));
    }
    for (const auto& cyl : m_cylinders) {
        auto shape = chrono_types::make_shared<ChCollisionShapeCylinder>(m_mat, cyl.m_radius, cyl.m_length);
        m_body->AddCollisionShape(shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
    }
    for (const auto& mesh : m_meshes) {
        auto vis_mesh_file = GetChronoDataFile("robot/robosimian/obj/" + mesh.m_name + ".obj");
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
        switch (mesh.m_type) {
            case MeshShape::Type::CONVEX_HULL: {
                auto shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(m_mat, trimesh->GetCoordsVertices());
                m_body->AddCollisionShape(shape, ChFrame<>(mesh.m_pos, mesh.m_rot));
                break;
            }
            case MeshShape::Type::TRIANGLE_SOUP: {
                auto shape =
                    chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh, false, false, 0.002);
                m_body->AddCollisionShape(shape, ChFrame<>(mesh.m_pos, mesh.m_rot));
                break;
            }
            case MeshShape::Type::NODE_CLOUD: {
                auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(m_mat, 0.002);
                for (const auto& v : trimesh->GetCoordsVertices()) {
                    m_body->AddCollisionShape(shape, ChFrame<>(v, QUNIT));
                }
                break;
            }
        }
    }
}

// =============================================================================

RS_Chassis::RS_Chassis(const std::string& name, bool fixed, std::shared_ptr<ChContactMaterial> mat, ChSystem* system)
    : RS_Part(name, mat, system), m_collide(false) {
    double mass = 46.658335;
    ChVector3d com(0.040288, -0.001937, -0.073574);
    ChVector3d inertia_xx(1.272134, 2.568776, 3.086984);
    ChVector3d inertia_xy(0.008890, -0.13942, 0.000325);

    m_body->SetTag(Robosimian_body_tag);
    m_body->SetMass(mass);
    m_body->SetFrameCOMToRef(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
    m_body->SetFixed(fixed);
    system->Add(m_body);

    // Create the set of primitive shapes
    m_boxes.push_back(BoxShape(VNULL, QUNIT, ChVector3d(0.257, 0.50, 0.238)));
    m_boxes.push_back(BoxShape(VNULL, QUNIT, ChVector3d(0.93, 0.230, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector3d(+0.25393, +0.075769, 0), QuatFromAngleZ(-0.38153), ChVector3d(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector3d(-0.25393, +0.075769, 0), QuatFromAngleZ(+0.38153), ChVector3d(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector3d(+0.25393, -0.075769, 0), QuatFromAngleZ(+0.38153), ChVector3d(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector3d(-0.25393, -0.075769, 0), QuatFromAngleZ(-0.38153), ChVector3d(0.36257, 0.23, 0.238)));

    m_cylinders.push_back(CylinderShape(ChVector3d(0.417050, 0, -0.158640),
                                        QuatFromAngleZ(CH_PI_2) * QuatFromAngleX(CH_PI_2 - 0.383972), 0.05, 0.144));

    // Geometry for link0 (all limbs); these links are fixed to the chassis
    m_cylinders.push_back(
        CylinderShape(ChVector3d(+0.29326, +0.20940, 0.03650 - 0.025), QuatFromAngleX(CH_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector3d(-0.29326, +0.20940, 0.03650 - 0.025), QuatFromAngleX(CH_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector3d(-0.29326, -0.20940, 0.03650 - 0.025), QuatFromAngleX(CH_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector3d(+0.29326, -0.20940, 0.03650 - 0.025), QuatFromAngleX(CH_PI_2), 0.05, 0.145));

    // Set the name of the visualization mesh
    m_mesh_name = "robosim_chassis";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
}

void RS_Chassis::Initialize(const ChCoordsys<>& pos) {
    m_body->SetFrameRefToAbs(ChFrame<>(pos));

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_FR);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_RR);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_RL);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_FL);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::SLED);

    // Note: call this AFTER setting the collision family (required for Chrono::Multicore)
    m_body->EnableCollision(m_collide);
}

void RS_Chassis::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void RS_Chassis::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// =============================================================================

RS_Sled::RS_Sled(const std::string& name, std::shared_ptr<ChContactMaterial> mat, ChSystem* system)
    : RS_Part(name, mat, system), m_collide(true) {
    double mass = 2.768775;
    ChVector3d com(0.000000, 0.000000, 0.146762);
    ChVector3d inertia_xx(0.034856, 0.082427, 0.105853);
    ChVector3d inertia_xy(0.000007, -0.000002, 0);

    m_body->SetTag(Robosimian_body_tag);
    m_body->SetMass(mass);
    m_body->SetFrameCOMToRef(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
    system->Add(m_body);

    m_meshes.push_back(MeshShape(ChVector3d(0, 0, 0), QUNIT, "robosim_sled_coll", MeshShape::Type::CONVEX_HULL));

    m_mesh_name = "robosim_sled";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.7f, 0.7f, 0.7f);
}

void RS_Sled::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector3d& xyz, const ChVector3d& rpy) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                   // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child
    m_body->SetFrameRefToAbs(X_GC);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::SLED);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_FR);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_RR);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_RL);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::LIMB_FL);

    // Note: call this AFTER setting the collision family (required for Chrono::Multicore)
    m_body->EnableCollision(m_collide);

    // Add joint (weld)
    auto joint = chrono_types::make_shared<ChLinkLockLock>();
    joint->Initialize(chassis, m_body, calcJointFrame(X_GC, ChVector3d(1, 0, 0)));
    chassis->GetSystem()->AddLink(joint);
}

void RS_Sled::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void RS_Sled::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// =============================================================================

RS_WheelDD::RS_WheelDD(const std::string& name, int id, std::shared_ptr<ChContactMaterial> mat, ChSystem* system)
    : RS_Part(name, mat, system) {
    double mass = 3.492500;
    ChVector3d com(0, 0, 0);
    ChVector3d inertia_xx(0.01, 0.01, 0.02);
    ChVector3d inertia_xy(0, 0, 0);

    m_body->SetTag(Robosimian_body_tag);
    m_body->SetMass(mass);
    m_body->SetFrameCOMToRef(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
    system->Add(m_body);

    m_cylinders.push_back(CylinderShape(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2), 0.074, 0.038));

    m_mesh_name = "robosim_dd_wheel";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.3f, 0.3f, 0.3f);
}

void RS_WheelDD::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector3d& xyz, const ChVector3d& rpy) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                   // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child
    m_body->SetFrameRefToAbs(X_GC);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::WHEEL_DD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::SLED);

    // Note: call this AFTER setting the collision family (required for Chrono::Multicore)
    m_body->EnableCollision(true);

    // Add joint
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(chassis, m_body, calcJointFrame(X_GC, ChVector3d(0, 0, 1)));
    chassis->GetSystem()->AddLink(joint);
}

// =============================================================================

RS_Limb::RS_Limb(const std::string& name,
                 LimbID id,
                 const LinkData data[],
                 std::shared_ptr<ChContactMaterial> wheel_mat,
                 std::shared_ptr<ChContactMaterial> link_mat,
                 ChSystem* system)
    : m_name(name), m_collide_links(false), m_collide_wheel(true) {
    for (int i = 0; i < num_links; i++) {
        bool is_wheel = (data[i].name.compare("link8") == 0);

        std::shared_ptr<RS_Part> link;
        if (is_wheel) {
            link = chrono_types::make_shared<RS_Part>(m_name + "_" + data[i].name, wheel_mat, system);
        } else {
            link = chrono_types::make_shared<RS_Part>(m_name + "_" + data[i].name, link_mat, system);
        }

        double mass = data[i].link.m_mass;
        ChVector3d com = data[i].link.m_com;
        ChVector3d inertia_xx = data[i].link.m_inertia_xx;
        ChVector3d inertia_xy = data[i].link.m_inertia_xy;

        link->m_body->SetTag(Robosimian_body_tag);
        link->m_body->SetMass(mass);
        link->m_body->SetFrameCOMToRef(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
        link->m_body->SetInertiaXX(inertia_xx);
        link->m_body->SetInertiaXY(inertia_xy);

        link->m_mesh_name = data[i].link.m_mesh_name;
        link->m_offset = data[i].link.m_offset;
        link->m_color = data[i].link.m_color;

        for (const auto& cyl : data[i].link.m_shapes) {
            link->m_cylinders.push_back(cyl);
        }

        if (data[i].include)
            system->Add(link->m_body);

        m_links.insert(std::make_pair(data[i].name, link));
        if (is_wheel)
            m_wheel = link;
    }
}

void RS_Limb::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                         const ChVector3d& xyz,
                         const ChVector3d& rpy,
                         CollisionFamily::Enum collision_family,
                         ActuationMode wheel_mode) {
    // Set absolute position of link0
    auto parent_body = chassis;                               // parent body
    auto child_body = m_links.find("link0")->second->m_body;  // child body
    ChFrame<> X_GP = parent_body->GetFrameRefToAbs();         // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                       // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                             // global -> child
    child_body->SetFrameRefToAbs(X_GC);

    // Traverse chain (base-to-tip)
    //   set absolute position of the child body
    //   add collision shapes on child body
    //   create joint between parent and child
    for (int i = 0; i < num_joints; i++) {
        auto parent = m_links.find(joints[i].linkA)->second;       // parent part
        auto child = m_links.find(joints[i].linkB)->second;        // child part
        parent_body = parent->m_body;                              // parent body
        child_body = child->m_body;                                // child body
        X_GP = parent_body->GetFrameRefToAbs();                    // global -> parent
        X_PC = ChFrame<>(joints[i].xyz, rpy2quat(joints[i].rpy));  // parent -> child
        X_GC = X_GP * X_PC;                                        // global -> child
        child_body->SetFrameRefToAbs(X_GC);

        // First joint connects directly to chassis
        if (i == 0)
            parent_body = chassis;

        // Add contact geometry to child body
        child->AddCollisionShapes();

        if (child_body->GetCollisionModel()) {
            // Place all links from this limb in the same collision family
            child_body->GetCollisionModel()->SetFamily(collision_family);
            child_body->GetCollisionModel()->DisallowCollisionsWith(collision_family);
        }

        // Note: call this AFTER setting the collision family (required for Chrono::Multicore)
        if (child == m_wheel)
            child_body->EnableCollision(m_collide_wheel);
        else
            child_body->EnableCollision(m_collide_links);

        // Weld joints
        if (joints[i].fixed) {
            auto joint = chrono_types::make_shared<ChLinkLockLock>();
            joint->SetName(m_name + "_" + joints[i].name);
            joint->Initialize(parent_body, child_body, calcJointFrame(X_GC, joints[i].axis));
            chassis->GetSystem()->AddLink(joint);
            m_joints.insert(std::make_pair(joints[i].name, joint));
            continue;
        }

        // Actuated joints (except the wheel motor joint)
        if (joints[i].name.compare("joint8") != 0) {
            auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
            auto joint = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            joint->SetName(m_name + "_" + joints[i].name);
            joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
            joint->SetAngleFunction(motor_fun);
            chassis->GetSystem()->AddLink(joint);
            m_joints.insert(std::make_pair(joints[i].name, joint));
            m_motors.insert(std::make_pair(joints[i].name, joint));
            continue;
        }

        // Wheel motor joint
        if (wheel_mode == ActuationMode::SPEED) {
            auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
            auto joint = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            joint->SetName(m_name + "_" + joints[i].name);
            joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
            joint->SetSpeedFunction(motor_fun);
            chassis->GetSystem()->AddLink(joint);
            m_joints.insert(std::make_pair(joints[i].name, joint));
            m_motors.insert(std::make_pair(joints[i].name, joint));
            m_wheel_motor = joint;
        } else {
            auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
            auto joint = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            joint->SetName(m_name + "_" + joints[i].name);
            joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
            joint->SetAngleFunction(motor_fun);
            chassis->GetSystem()->AddLink(joint);
            m_joints.insert(std::make_pair(joints[i].name, joint));
            m_motors.insert(std::make_pair(joints[i].name, joint));
            m_wheel_motor = joint;
        }
    }
}

double RS_Limb::GetMass() const {
    double mass = 0;
    for (auto link : m_links)
        mass += link.second->m_body->GetMass();
    return mass;
}

void RS_Limb::SetVisualizationType(VisualizationType vis) {
    for (auto link : m_links)
        link.second->SetVisualizationType(vis);
}

void RS_Limb::Activate(const std::string& motor_name, double time, double val) {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end()) {
        std::cout << "Limb::Activate -- Unknown motor " << motor_name << std::endl;
        return;
    }

    // Note: currently hard-coded for angle motor
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(itr->second->GetMotorFunction());
    fun->SetSetpoint(-val, time);
}

double RS_Limb::GetMotorAngle(const std::string& motor_name) const {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end()) {
        std::cout << "Limb::GetMotorAngle -- Unknown motor " << motor_name << std::endl;
        return 0;
    }

    return itr->second->GetMotorAngle();
}

double RS_Limb::GetMotorOmega(const std::string& motor_name) const {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end()) {
        std::cout << "Limb::GetMotorOmega -- Unknown motor " << motor_name << std::endl;
        return 0;
    }

    return itr->second->GetMotorAngleDt();
}

double RS_Limb::GetMotorTorque(const std::string& motor_name) const {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end()) {
        std::cout << "Limb::GetMotorTorque -- Unknown motor " << motor_name << std::endl;
        return 0;
    }

    return itr->second->GetMotorTorque();
}

static std::string motor_names[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};

void RS_Limb::Activate(double time, const std::array<double, 8>& vals) {
    for (int i = 0; i < 8; i++) {
        auto fun = std::static_pointer_cast<ChFunctionSetpoint>(m_motors[motor_names[i]]->GetMotorFunction());
        fun->SetSetpoint(-vals[i], time);
    }
}

std::array<double, 8> RS_Limb::GetMotorAngles() {
    std::array<double, 8> result;

    for (int i = 0; i < 8; i++) {
        result[i] = m_motors[motor_names[i]]->GetMotorAngle();
    }

    return result;
}

std::array<double, 8> RS_Limb::GetMotorOmegas() {
    std::array<double, 8> result;

    for (int i = 0; i < 8; i++) {
        result[i] = m_motors[motor_names[i]]->GetMotorAngleDt();
    }

    return result;
}

std::array<double, 8> RS_Limb::GetMotorTorques() {
    std::array<double, 8> result;

    for (int i = 0; i < 8; i++) {
        result[i] = m_motors[motor_names[i]]->GetMotorTorque();
    }

    return result;
}

void RS_Limb::GetMotorActuations(std::array<double, 8>& angles, std::array<double, 8>& speeds) {
    for (int i = 0; i < 8; i++) {
        auto fun = std::static_pointer_cast<ChFunctionSetpoint>(m_motors[motor_names[i]]->GetMotorFunction());
        // Note: the time passed as argument here does not matter for a ChfunctionSetpoint
        angles[i] = fun->GetVal(0);
        speeds[i] = fun->GetDer(0);
    }
}

void RS_Limb::SetCollideLinks(bool state) {
    m_collide_links = state;
    for (auto link : m_links) {
        if (link.second != m_wheel)
            link.second->m_body->EnableCollision(state);
    }
}

void RS_Limb::SetCollideWheel(bool state) {
    m_collide_wheel = state;
    m_wheel->m_body->EnableCollision(state);
}

void RS_Limb::Translate(const ChVector3d& shift) {
    for (auto link : m_links) {
        link.second->m_body->SetPos(link.second->m_body->GetPos() + shift);
    }
}

}  // end namespace robosimian
}  // namespace chrono
