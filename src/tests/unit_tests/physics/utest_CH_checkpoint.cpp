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
//
// Unit tests for checkpoint restarts.
// Each test runs a continuous simulation and a restart, in a freshly built (not yet initialized) system, from a
// checkpoint written halfway through, and requires identical final states.
// - SYSTEM checkpoint of an ANCF beam
// - SYSTEM checkpoint of a cable whose reference configuration differs from its initial state
// - SYSTEM checkpoint of a yielding IGA beam (element internal state, accelerations, time-dependent motor)
// - loading a checkpoint without the internal state block (older file format) loses the plastic history
// - COMPONENT checkpoint of an FEA mesh and a DAE physics item
// - SYSTEM and COMPONENT checkpoints of an RSDA wound over several turns, a TSDA with an ODE state, a speed motor and
//   an angle motor turned over several turns
// - SYSTEM checkpoints with the HHT integrator (accelerations, Lagrange multipliers, step size control state, quaternion
//   derivatives of bodies), for an ANCF beam and for a pendulum on a revolute joint with a cable clamped to it
// - loading a checkpoint into a different system fails
//
// =============================================================================

#include <cmath>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChExternalDynamicsDAE.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/functions/ChFunctionSine.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/input_output/ChCheckpointASCII.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

const std::string out_dir = GetChronoTestOutputPath() + "/ch_checkpoint/";

// -----------------------------------------------------------------------------

// Deterministic solver settings, so that restarts can be compared exactly
static void Configure(ChSystem& sys, ChTimestepper::Type integrator = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED) {
    sys.SetNumThreads(1);
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    sys.SetSolver(solver);
    sys.SetTimestepperType(integrator);
    if (auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(50);
        hht->SetAbsTolerances(1e-5);
    }
}

// Maximum difference between the states of two systems
static double StateDiff(ChSystem& a, ChSystem& b) {
    a.Setup();
    b.Setup();
    if (a.GetNumCoordsPosLevel() != b.GetNumCoordsPosLevel() || a.GetNumCoordsVelLevel() != b.GetNumCoordsVelLevel())
        return 1e300;
    ChState xa(a.GetNumCoordsPosLevel(), &a);
    ChState xb(b.GetNumCoordsPosLevel(), &b);
    ChStateDelta va(a.GetNumCoordsVelLevel(), &a);
    ChStateDelta vb(b.GetNumCoordsVelLevel(), &b);
    double t;
    a.StateGather(xa, va, t);
    b.StateGather(xb, vb, t);
    return std::max((xa - xb).cwiseAbs().maxCoeff(), (va - vb).cwiseAbs().maxCoeff());
}

// Sum of accumulated plastic strains over all IGA beam elements in the system
static double PlasticStrain(ChSystem& sys) {
    double sum = 0;
    for (const auto& mesh : sys.GetAssembly().GetMeshes()) {
        for (const auto& element : mesh->GetElements()) {
            if (auto iga = std::dynamic_pointer_cast<ChElementBeamIGA>(element)) {
                for (const auto& data : iga->GetPlasticData())
                    sum += std::abs(data->p_strain_acc);
            }
        }
    }
    return sum;
}

// Copy a SYSTEM checkpoint file without its last line (accelerations and internal state), as in older files
static std::string StripInternalState(const std::string& filename) {
    std::ifstream ifile(filename);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(ifile, line))
        lines.push_back(line);
    std::string stripped = filename + ".stripped";
    std::ofstream ofile(stripped);
    for (size_t i = 0; i + 1 < lines.size(); i++)
        ofile << lines[i] << "\n";
    return stripped;
}

// Run a continuous simulation of n1+n2 steps in sys1, writing a SYSTEM checkpoint after n1 steps.
// Then restart from that checkpoint in sys2, freshly built, and simulate n2 steps.
static void RunSystemRestart(ChSystem& sys1,
                             ChSystem& sys2,
                             const std::function<void(ChSystem&)>& build,
                             int n1,
                             int n2,
                             double step,
                             const std::string& filename,
                             bool strip = false,
                             ChTimestepper::Type integrator = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED) {
    ASSERT_TRUE(CreateOutputDirectory(std::filesystem::path(out_dir)));

    Configure(sys1, integrator);
    build(sys1);
    for (int i = 0; i < n1; i++)
        sys1.DoStepDynamics(step);
    {
        ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
        cp.Save(&sys1);
        cp.WriteFile(out_dir + filename);
    }
    for (int i = 0; i < n2; i++)
        sys1.DoStepDynamics(step);

    Configure(sys2, integrator);
    build(sys2);
    {
        ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
        cp.ReadFile(strip ? StripInternalState(out_dir + filename) : out_dir + filename);
        cp.Load(&sys2);
    }
    for (int i = 0; i < n2; i++)
        sys2.DoStepDynamics(step);
}

// -----------------------------------------------------------------------------

// ANCF 3243 cantilever with a constant tip load
static void BuildANCF(ChSystem& sys) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    double length = 5;
    double width = 0.1;
    double thickness = 0.1;
    int num_elements = 8;
    double dx = length / num_elements;
    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(8000, 4e8, 0, 10.0 / 12, 10.0 / 12);

    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(0, 0, 0));
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);
    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(dx * i, 0, 0));
        mesh->AddNode(nodeB);
        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(dx, thickness, width);
        element->SetMaterial(material);
        mesh->AddElement(element);
        nodeA = nodeB;
    }
    nodeA->SetForce(ChVector3d(0, 0, -500));
}

// Physics item that moves the free end of a cable in phase 2, after the cable recorded its reference configuration
class Tensioner : public ChPhysicsItem {
  public:
    Tensioner(std::shared_ptr<ChNodeFEAxyzD> node) : m_node(node) {}

  private:
    virtual void SetupInitialState() override { m_node->SetPos(m_node->GetPos() + ChVector3d(0.01, 0.02, 0)); }

    std::shared_ptr<ChNodeFEAxyzD> m_node;
};

// ANCF cable cantilever, pretensioned by a Tensioner
static void BuildPretensionedCable(ChSystem& sys) {
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    auto mesh = chrono_types::make_shared<ChMesh>();
    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.015);
    section->SetYoungModulus(0.01e9);
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, 10, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
    builder.GetLastBeamNodes().front()->SetFixed(true);
    sys.Add(mesh);

    sys.Add(chrono_types::make_shared<Tensioner>(builder.GetLastBeamNodes().back()));
}

// IGA beam with lumped plasticity, pushed and pulled by a motor into repeated yielding
static void BuildYieldingIGA(ChSystem& sys) {
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    double wy = 0.012;
    double wz = 0.025;
    auto inertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    inertia->SetAsRectangularSection(wy, wz, 1000);
    auto elasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    elasticity->SetYoungModulus(0.02e10);
    elasticity->SetShearModulus(0.02e10 * 0.3);
    elasticity->SetAsRectangularSection(wy, wz);
    auto plasticity = chrono_types::make_shared<ChPlasticityCosseratLumped>();
    plasticity->n_yeld_x = chrono_types::make_shared<ChFunctionConst>(3000);
    plasticity->n_beta_x = chrono_types::make_shared<ChFunctionRamp>(0, 1e3);
    plasticity->n_yeld_My = chrono_types::make_shared<ChFunctionConst>(0.3);
    plasticity->n_beta_My = chrono_types::make_shared<ChFunctionRamp>(0, 0.001e2);
    plasticity->n_yeld_Mz = chrono_types::make_shared<ChFunctionConst>(0.3);
    plasticity->n_beta_Mz = chrono_types::make_shared<ChFunctionRamp>(0, 0.001e2);
    auto section = chrono_types::make_shared<ChBeamSectionCosserat>(inertia, elasticity, plasticity);

    ChBuilderBeamIGA builder;
    builder.BuildBeam(mesh, section, 5, ChVector3d(0, 0, 0), ChVector3d(0.4, 0, 0), VECT_Y, 3);
    builder.GetLastBeamNodes().front()->SetFixed(true);

    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    auto tip = builder.GetLastBeamNodes().back();
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->Initialize(tip, truss, ChFrame<>(tip->GetPos(), Q_ROTATE_Z_TO_X));
    motor->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::SPHERICAL);
    motor->SetMotionFunction(chrono_types::make_shared<ChFunctionSine>(0.05, 0.1));
    sys.Add(motor);
}

// Planar pendulum modeled as a DAE (3 states, 2 algebraic constraints)
class PendulumDAE : public ChExternalDynamicsDAE {
  public:
    PendulumDAE() : g(9.8), L(0.5), m(2.0), I(m * L * L / 3) {}

    virtual PendulumDAE* Clone() const override { return new PendulumDAE(*this); }

    virtual unsigned int GetNumStates() const override { return 3; }
    virtual unsigned int GetNumStateDerivatives() const override { return 3; }
    virtual unsigned int GetNumAlgebraicConstraints() const override { return 2; }
    virtual bool InExplicitForm() const override { return false; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) override {
        y0 << L, 0, 0;
        yd0.setZero();
    }

    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) override {
        M.setZero();
        M(0, 0) = m;
        M(1, 1) = m;
        M(2, 2) = I;
    }

    virtual void CalculateForce(double time, const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& F) override { F << 0, -m * g, 0; }

    virtual void CalculateConstraintViolation(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& c) override {
        c(0) = y(0) - L * std::cos(y(2));
        c(1) = y(1) - L * std::sin(y(2));
    }

    virtual void CalculateConstraintJacobian(double time, const ChVectorDynamic<>& y, const ChVectorDynamic<>& c, ChMatrixDynamic<>& J) override {
        J << 1, 0, L * std::sin(y(2)), 0, 1, -L * std::cos(y(2));
    }

  private:
    double g, L, m, I;
};

// TSDA force with a first-order lag: the force s follows a linear spring-damper force through s' = (f - s) / tau
class LaggedForceODE : public ChLinkTSDA::ODE {
  public:
    virtual unsigned int GetNumStates() const override { return 1; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states, const ChLinkTSDA& link) override { states(0) = 0; }
    virtual void CalculateRHS(double time, const ChVectorDynamic<>& states, ChVectorDynamic<>& rhs, const ChLinkTSDA& link) override {
        double f = -100 * link.GetDeformation() - 2 * link.GetVelocity();
        rhs(0) = (f - states(0)) / 0.05;
    }
};

class LaggedForce : public ChLinkTSDA::ForceFunctor {
  public:
    virtual double evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) override { return link.GetStates()(0); }
};

// Bodies connected to the ground by springs and motors:
// - a flywheel on a revolute joint, spinning against a soft RSDA that winds up over several turns
// - a body suspended from the ground by a TSDA with an internal ODE state
// - a body driven by a speed motor
// - a body driven by an angle motor over several turns
static void BuildSpringsAndMotor(ChSystem& sys) {
    static LaggedForceODE ode;

    sys.SetGravitationalAcceleration(ChVector3d(0, -9.8, 0));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    auto flywheel = chrono_types::make_shared<ChBody>();
    flywheel->SetInertiaXX(ChVector3d(1, 1, 1));
    flywheel->SetAngVelLocal(ChVector3d(0, 0, 20));
    sys.Add(flywheel);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(flywheel, ground, ChFramed());
    sys.AddLink(revolute);

    auto rsda = chrono_types::make_shared<ChLinkRSDA>();
    rsda->Initialize(flywheel, ground, ChFramed());
    rsda->SetSpringCoefficient(1);
    sys.AddLink(rsda);

    auto bob = chrono_types::make_shared<ChBody>();
    bob->SetPos(ChVector3d(1.2, -0.5, 0.3));
    sys.Add(bob);

    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(bob, ground, false, bob->GetPos(), ChVector3d(1, 0, 0));
    tsda->SetRestLength(0.4);
    tsda->RegisterODE(&ode);
    tsda->RegisterForceFunctor(chrono_types::make_shared<LaggedForce>());
    sys.AddLink(tsda);

    auto rotor = chrono_types::make_shared<ChBody>();
    rotor->SetPos(ChVector3d(-1, 0, 0));
    sys.Add(rotor);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(rotor, ground, ChFramed(rotor->GetPos()));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionSine>(3, 0.8));
    sys.AddLink(motor);

    auto spinner = chrono_types::make_shared<ChBody>();
    spinner->SetPos(ChVector3d(-1, 1, 0));
    sys.Add(spinner);

    auto angle_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    angle_motor->Initialize(spinner, ground, ChFramed(spinner->GetPos()));
    angle_motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, 12));
    sys.AddLink(angle_motor);
}

// Component lists of the model built by BuildSpringsAndMotor
static ChAssembly::Components SpringsAndMotorComponents(ChSystem& sys) {
    ChAssembly::Components c;
    const auto& bodies = sys.GetAssembly().GetBodies();
    c.bodies.assign(bodies.begin() + 1, bodies.end());
    for (const auto& link : sys.GetAssembly().GetLinks()) {
        if (auto rsda = std::dynamic_pointer_cast<ChLinkRSDA>(link))
            c.rsdas.push_back(rsda);
        else if (auto tsda = std::dynamic_pointer_cast<ChLinkTSDA>(link))
            c.tsdas.push_back(tsda);
        else if (auto motor = std::dynamic_pointer_cast<ChLinkMotorRotation>(link))
            c.rot_motors.push_back(motor);
        else
            c.joints.push_back(std::dynamic_pointer_cast<ChLink>(link));
    }
    return c;
}

// Rigid pendulum on a revolute joint, with an ANCF cable clamped to its tip (joint and node constraints)
static void BuildPendulumCable(ChSystem& sys) {
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    auto arm = chrono_types::make_shared<ChBody>();
    arm->SetMass(2);
    arm->SetInertiaXX(ChVector3d(0.01, 0.1, 0.1));
    arm->SetPos(ChVector3d(0.25, 0, 0));
    sys.Add(arm);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(ground, arm, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sys.Add(revolute);

    auto mesh = chrono_types::make_shared<ChMesh>();
    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.01);
    section->SetYoungModulus(1e8);
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, 8, ChVector3d(0.5, 0, 0), ChVector3d(1.0, 0, 0));
    sys.Add(mesh);

    auto node = builder.GetLastBeamNodes().front();
    auto point = chrono_types::make_shared<ChLinkNodeFrame>();
    point->Initialize(node, arm);
    sys.Add(point);
    auto direction = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    direction->Initialize(node, arm);
    sys.Add(direction);
}

// Angle of the RSDA in the model built by BuildSpringsAndMotor (including full turns)
static double RSDAAngle(ChSystem& sys) {
    return SpringsAndMotorComponents(sys).rsdas.front()->GetAngle();
}

// Angle of the angle motor in the model built by BuildSpringsAndMotor (including full turns)
static double MotorAngle(ChSystem& sys) {
    for (const auto& motor : SpringsAndMotorComponents(sys).rot_motors) {
        if (std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(motor))
            return motor->GetMotorAngle();
    }
    return 0;
}

// -----------------------------------------------------------------------------

TEST(ChCheckpoint, system_ANCF) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildANCF, 500, 500, 1e-3, "ancf.txt");
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_pretensioned_cable) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildPretensionedCable, 300, 300, 1e-3, "cable.txt");
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_yielding_IGA) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildYieldingIGA, 500, 500, 1e-2, "iga.txt");
    EXPECT_GT(PlasticStrain(sys1), 0);
    EXPECT_EQ(PlasticStrain(sys1), PlasticStrain(sys2));
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_without_internal_state) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildYieldingIGA, 500, 500, 1e-2, "iga_old.txt", true);
    EXPECT_GT(std::abs(PlasticStrain(sys1) - PlasticStrain(sys2)), 1e-3 * PlasticStrain(sys1));
}

TEST(ChCheckpoint, component_mesh_and_item) {
    ASSERT_TRUE(CreateOutputDirectory(std::filesystem::path(out_dir)));
    const std::string filename = out_dir + "component.txt";

    auto build = [](ChSystem& sys) {
        Configure(sys);
        BuildYieldingIGA(sys);
        sys.Add(chrono_types::make_shared<PendulumDAE>());
    };
    auto components = [](ChSystem& sys) {
        ChAssembly::Components c;
        c.meshes.push_back(sys.GetAssembly().GetMeshes().front());
        c.items.push_back(sys.GetAssembly().GetOtherPhysicsItems().front());
        return c;
    };

    ChSystemSMC sys1, sys2;
    build(sys1);
    build(sys2);

    for (int i = 0; i < 500; i++)
        sys1.DoStepDynamics(1e-2);
    {
        ChCheckpointASCII cp(ChCheckpoint::Type::COMPONENT);
        cp.Save(sys1.GetChTime(), components(sys1));
        cp.WriteFile(filename);
    }
    for (int i = 0; i < 500; i++)
        sys1.DoStepDynamics(1e-2);

    {
        auto c = components(sys2);
        ChCheckpointASCII cp(ChCheckpoint::Type::COMPONENT);
        cp.ReadFile(filename);
        double time;
        cp.Load(time, c);
        sys2.SetChTime(time);
    }
    for (int i = 0; i < 500; i++)
        sys2.DoStepDynamics(1e-2);

    EXPECT_EQ(PlasticStrain(sys1), PlasticStrain(sys2));
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_springs_and_motor) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildSpringsAndMotor, 500, 500, 1e-3, "springs.txt");
    EXPECT_GT(std::abs(RSDAAngle(sys1)), 2 * CH_2PI);
    EXPECT_EQ(RSDAAngle(sys1), RSDAAngle(sys2));
    EXPECT_GT(std::abs(MotorAngle(sys1)), CH_2PI);
    EXPECT_EQ(MotorAngle(sys1), MotorAngle(sys2));
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, component_springs_and_motor) {
    ASSERT_TRUE(CreateOutputDirectory(std::filesystem::path(out_dir)));
    const std::string filename = out_dir + "springs_component.txt";

    ChSystemSMC sys1, sys2;
    Configure(sys1);
    BuildSpringsAndMotor(sys1);
    Configure(sys2);
    BuildSpringsAndMotor(sys2);

    for (int i = 0; i < 500; i++)
        sys1.DoStepDynamics(1e-3);
    ASSERT_GT(std::abs(RSDAAngle(sys1)), CH_2PI);
    ASSERT_GT(std::abs(MotorAngle(sys1)), CH_PI);
    {
        ChCheckpointASCII cp(ChCheckpoint::Type::COMPONENT);
        cp.Save(sys1.GetChTime(), SpringsAndMotorComponents(sys1));
        cp.WriteFile(filename);
    }
    for (int i = 0; i < 500; i++)
        sys1.DoStepDynamics(1e-3);

    {
        auto c = SpringsAndMotorComponents(sys2);
        ChCheckpointASCII cp(ChCheckpoint::Type::COMPONENT);
        cp.ReadFile(filename);
        double time;
        cp.Load(time, c);
        sys2.SetChTime(time);
    }
    for (int i = 0; i < 500; i++)
        sys2.DoStepDynamics(1e-3);

    EXPECT_EQ(RSDAAngle(sys1), RSDAAngle(sys2));
    EXPECT_EQ(MotorAngle(sys1), MotorAngle(sys2));
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_ANCF_HHT) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildANCF, 500, 500, 1e-3, "ancf_hht.txt", false, ChTimestepper::Type::HHT);
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, system_constrained_HHT) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildPendulumCable, 500, 500, 1e-3, "pendulum_hht.txt", false, ChTimestepper::Type::HHT);
    EXPECT_EQ(StateDiff(sys1, sys2), 0);
}

TEST(ChCheckpoint, mismatched_system) {
    ChSystemSMC sys1, sys2;
    RunSystemRestart(sys1, sys2, BuildANCF, 10, 0, 1e-3, "mismatch.txt");

    ChSystemSMC sys;
    Configure(sys);
    BuildPretensionedCable(sys);
    ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
    cp.ReadFile(out_dir + "mismatch.txt");
    EXPECT_THROW(cp.Load(&sys), std::exception);
}
