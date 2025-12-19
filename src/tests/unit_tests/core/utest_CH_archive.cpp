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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - archives for serialization,
// - serialization, with versioning and dynamic creation (class factory)
//
// =============================================================================

#include "gtest/gtest.h"

#include <typeinfo>

#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/solver/ChSolverPSOR.h"

#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

const double ABS_ERR = 1e-5;
enum class ArchiveType { BINARY, JSON, XML };
const std::string val_dir = "TEST_RESULTS/";
const std::string out_dir = val_dir + "ch_archive/";

void assemble_fourbar(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    // Joint coords
    ChFrame<> frameO(ChVector3d(0, 0, 0), QUNIT);
    ChFrame<> frameA(ChVector3d(1, 1, 0), QUNIT);
    ChFrame<> frameB(ChVector3d(3, 1, 0), QUNIT);
    ChFrame<> frameC(ChVector3d(3, 0, 0), QUNIT);

    // Bodies
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    system.Add(floor);

    auto crank = chrono_types::make_shared<ChBody>();
    crank->SetPos((frameO.GetPos() + frameA.GetPos()) / 2.);
    system.Add(crank);

    auto rod = chrono_types::make_shared<ChBody>();
    rod->SetPos((frameA.GetPos() + frameB.GetPos()) / 2.);
    system.Add(rod);

    auto rocker = chrono_types::make_shared<ChBody>();
    rocker->SetPos((frameB.GetPos() + frameC.GetPos()) / 2.);
    system.Add(rocker);

    auto linkO = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    linkO->SetName("linkO");
    linkO->Initialize(crank, floor, ChFrame<>());
    linkO->SetMotorFunction(chrono_types::make_shared<ChFunctionPoly345>(3.14, 1));
    system.Add(linkO);

    auto linkA = chrono_types::make_shared<ChLinkMateRevolute>();
    linkA->Initialize(rod, crank, frameA);
    linkA->SetName("linkA");
    system.Add(linkA);

    auto linkB = chrono_types::make_shared<ChLinkMateRevolute>();
    linkB->Initialize(rocker, rod, frameB);
    linkB->SetName("linkB");
    system.Add(linkB);

    auto linkC = chrono_types::make_shared<ChLinkMateRevolute>();
    linkC->Initialize(rocker, floor, frameC);
    // linkC->Initialize(floor, rocker, frameC);
    linkC->SetName("linkC");
    system.Add(linkC);
}

void assemble_pendulum(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0.0, -9.81, 0.0));

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    floor->SetName("floor");
    floor->SetTag(100);
    system.Add(floor);

    auto moving_body = chrono_types::make_shared<ChBody>();
    moving_body->SetPos(ChVector3d(1.0, -1.0, 1.0));
    moving_body->SetName("moving_body");
    moving_body->SetTag(101);
    system.Add(moving_body);

    auto link = chrono_types::make_shared<ChLinkMateRevolute>();
    link->Initialize(moving_body, floor, ChFrame<>());
    // auto link = chrono_types::make_shared<ChLinkLockRevolute>();
    // link->Initialize(moving_body, floor, ChCoordsys<>());
    system.Add(link);
}

void assemble_gear_and_pulleys(ChSystemNSC& sys) {
    sys.SetGravitationalAcceleration(ChVector3d(0, -10, 0));
    // Create a Chrono physical system

    // Contact material shared among all bodies
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Shared visualization material
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));

    // Create all the rigid bodies.

    double radA = 2;
    double radB = 4;

    // ...the truss
    auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_truss);
    mbody_truss->SetFixed(true);
    mbody_truss->SetPos(ChVector3d(0, 0, 3));

    // ...the first gear
    auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radA, 0.5, 1000, true, false, mat);
    // auto mbody_gearA = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector3d(0, 0, -1));
    mbody_gearA->SetRot(QuatFromAngleAxis(CH_PI / 2, VECT_X));
    mbody_gearA->GetVisualShape(0)->SetMaterial(0, vis_mat);

    // ...impose rotation speed between the first gear and the fixed truss
    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(6));
    sys.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radB, 0.4, 1000, true, false, mat);
    sys.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector3d(interaxis12, 0, -1));
    mbody_gearB->SetRot(QuatFromAngleAxis(CH_PI / 2, VECT_X));
    mbody_gearB->GetVisualShape(0)->SetMaterial(0, vis_mat);

    // ... the second gear is fixed to the rotating bar
    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_truss, ChFrame<>(ChVector3d(interaxis12, 0, 0), QUNIT));  // TEMP
    // link_revolute->Initialize(mbody_gearB, mbody_train, ChCoordsys<>(ChVector3d(interaxis12, 0, 0), QUNIT));
    sys.AddLink(link_revolute);

    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL, chrono::QuatFromAngleX(-CH_PI_2)));
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL, chrono::QuatFromAngleX(-CH_PI_2)));
    link_gearAB->SetTransmissionRatio(radA / radB);
    link_gearAB->SetEnforcePhase(true);
    sys.AddLink(link_gearAB);
}

void assemble_pendulum_visual(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0.0, -9.81, 0.0));

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    floor->SetName("floor");
    floor->SetTag(100);
    system.Add(floor);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    auto moving_body = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
                                                                100,         // density
                                                                true,        // visualization?
                                                                false,       // collision?
                                                                mat);        // contact material
    // auto moving_body = chrono_types::make_shared<ChBody>();
    moving_body->SetPos(ChVector3d(1.0, -1.0, 1.0));
    moving_body->SetName("moving_body");
    moving_body->SetTag(101);
    system.Add(moving_body);

    // auto link = chrono_types::make_shared<ChLinkMateRevolute>();
    // link->Initialize(moving_body, floor, ChFrame<>());
    auto link = chrono_types::make_shared<ChLinkLockRevolute>();
    link->Initialize(moving_body, floor, ChFrame<>());
    system.Add(link);
}

void create_test(std::function<void(ChSystemNSC&)> assembler_fun,
                 ArchiveType outtype,
                 std::string outputfilename = "") {
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    std::string outputfile;
    if (outputfilename.compare("") == 0)
        outputfile = std::string(::testing::UnitTest::GetInstance()->current_test_suite()->name()) + "_" +
                     std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
    else
        outputfile = outputfilename;

    outputfile = out_dir + outputfile;

    std::string extension;
    switch (outtype) {
        case ArchiveType::BINARY:
            extension = ".dat";
            break;
        case ArchiveType::JSON:
            extension = ".json";
            break;
        case ArchiveType::XML:
            extension = ".xml";
            break;
    };

    double timestep = 0.01;
    int step_num = 2000;

    std::shared_ptr<ChState> state_before_archive;
    std::shared_ptr<ChState> state_after_archive;

    {
        ChSystemNSC system;

        assembler_fun(system);

        std::ofstream outstreamfile(outputfile + extension);
        std::shared_ptr<ChArchiveOut> archiveout;

        switch (outtype) {
            case ArchiveType::BINARY:
                archiveout = chrono_types::make_shared<ChArchiveOutBinary>(outstreamfile);
                break;
            case ArchiveType::JSON:
                archiveout = chrono_types::make_shared<ChArchiveOutJSON>(outstreamfile);
                break;
            case ArchiveType::XML:
                archiveout = chrono_types::make_shared<ChArchiveOutXML>(outstreamfile);
                break;
        };

        *archiveout << CHNVP(system);

        for (int step = 0; step < step_num; ++step) {
            system.DoStepDynamics(timestep);
        }

        state_before_archive = chrono_types::make_shared<ChState>(system.GetNumCoordsPosLevel(), &system);
        auto state_delta_dummy = chrono_types::make_shared<ChStateDelta>(system.GetNumCoordsVelLevel(), &system);
        double time_dummy;
        system.StateGather(*state_before_archive, *state_delta_dummy, time_dummy);
    }

    std::ifstream instreamfile(outputfile + extension);
    std::shared_ptr<ChArchiveIn> archivein;
    switch (outtype) {
        case ArchiveType::BINARY:
            archivein = chrono_types::make_shared<ChArchiveInBinary>(instreamfile);
            break;
        case ArchiveType::JSON:
            archivein = chrono_types::make_shared<ChArchiveInJSON>(instreamfile);
            break;
        case ArchiveType::XML:
            archivein = chrono_types::make_shared<ChArchiveInXML>(instreamfile);
            break;
    };

    ChSystemNSC system;
    *archivein >> CHNVP(system);

    // Simulation loop
    for (int step = 0; step < step_num; ++step) {
        system.DoStepDynamics(timestep);
    }

    state_after_archive = chrono_types::make_shared<ChState>(system.GetNumCoordsPosLevel(), &system);
    auto state_delta_dummy = chrono_types::make_shared<ChStateDelta>(system.GetNumCoordsVelLevel(), &system);
    double time_dummy;
    system.StateGather(*state_after_archive, *state_delta_dummy, time_dummy);

    ASSERT_EQ(state_before_archive->size(), state_after_archive->size());

    for (auto i = 0; i < state_before_archive->size(); ++i) {
        ASSERT_NEAR(state_before_archive->data()[i], state_after_archive->data()[i], ABS_ERR);
    }
}

TEST(ChArchiveJSON, Pendulum) {
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    double timestep = 0.01;
    int step_num = 2000;

    std::shared_ptr<ChState> state_before_archive;
    std::shared_ptr<ChState> state_after_archive;

    {
        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, -9.81, 0.0));

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        floor->SetName("floor");
        floor->SetTag(100);
        system.Add(floor);

        auto moving_body = chrono_types::make_shared<ChBody>();
        moving_body->SetPos(ChVector3d(1.0, -1.0, 1.0));
        moving_body->SetName("moving_body");
        moving_body->SetTag(101);
        system.Add(moving_body);

        std::ofstream mfileo(out_dir + "ChArchiveJSON_Pendulum.json");
        ChArchiveOutJSON archive_out(mfileo);
        archive_out << CHNVP(system);

        // Simulation loop
        for (int step = 0; step < step_num; ++step) {
            system.DoStepDynamics(timestep);
        }

        state_before_archive = chrono_types::make_shared<ChState>(system.GetNumCoordsPosLevel(), &system);
        auto state_delta_dummy = chrono_types::make_shared<ChStateDelta>(system.GetNumCoordsVelLevel(), &system);
        double time_dummy;
        system.StateGather(*state_before_archive, *state_delta_dummy, time_dummy);
    }

    std::ifstream mfilei(out_dir + "ChArchiveJSON_Pendulum.json");
    ChArchiveInJSON archive_in(mfilei);
    archive_in.TryTolerateMissingTokens(true);

    ChSystemNSC system;
    archive_in >> CHNVP(system);

    // Simulation loop
    for (int step = 0; step < step_num; ++step) {
        system.DoStepDynamics(timestep);
    }

    state_after_archive = chrono_types::make_shared<ChState>(system.GetNumCoordsPosLevel(), &system);
    auto state_delta_dummy = chrono_types::make_shared<ChStateDelta>(system.GetNumCoordsVelLevel(), &system);
    double time_dummy;
    system.StateGather(*state_after_archive, *state_delta_dummy, time_dummy);

    ASSERT_EQ(state_before_archive->size(), state_after_archive->size());

    for (auto i = 0; i < state_before_archive->size(); ++i) {
        ASSERT_NEAR(state_before_archive->data()[i], state_after_archive->data()[i], ABS_ERR);
    }
}

TEST(ChArchiveJSON, Fourbar) {
    create_test(assemble_fourbar, ArchiveType::JSON);
}

TEST(ChArchiveJSON, PendulumVisual) {
    create_test(assemble_pendulum_visual, ArchiveType::JSON);
}

TEST(ChArchiveJSON, Gears) {
    create_test(assemble_gear_and_pulleys, ArchiveType::JSON);
}

TEST(ChArchiveXML, Fourbar) {
    create_test(assemble_fourbar, ArchiveType::XML);
}

TEST(ChArchiveBinary, Fourbar) {
    create_test(assemble_fourbar, ArchiveType::BINARY);
}

TEST(ChArchiveJSON, Solver) {
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    std::string outputfile = std::string(::testing::UnitTest::GetInstance()->current_test_suite()->name()) + "_" +
                             std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());

    outputfile = out_dir + outputfile;

    {
        ChSolverPSOR* solverPSOR_ptr = new ChSolverPSOR();
        // ChIterativeSolverVI* solverISVI_ptr = solverPSOR_ptr;
        // ChIterativeSolver* solverIS_ptr = solverPSOR_ptr;
        // ChSolverVI* solverVI_ptr = solverPSOR_ptr;
        ChSolver* solverBase_ptr = solverPSOR_ptr;

        solverPSOR_ptr->GetType();
        solverBase_ptr->GetType();

        // std::cout << "solverPSOR_ptr : " << solverPSOR_ptr << std::endl;
        // std::cout << "solverISVI_ptr : " << solverISVI_ptr << std::endl;
        // std::cout << "solverIS_ptr   : " << solverIS_ptr << std::endl;
        // std::cout << "solverVI_ptr   : " << solverVI_ptr << std::endl;
        // std::cout << "solverBase_ptr : " << solverBase_ptr << std::endl;

        std::ofstream mfileo(outputfile + ".json");
        ChArchiveOutJSON archive_out(mfileo);

        archive_out << CHNVP(solverBase_ptr);

        delete solverPSOR_ptr;
    }

    std::ifstream mfilei(outputfile + ".json");
    ChArchiveInJSON archive_in(mfilei);
    ChSolver* solverBase_ptr;
    archive_in >> CHNVP(solverBase_ptr);

    ChSolverPSOR* solverPSOR_ptr = dynamic_cast<ChSolverPSOR*>(solverBase_ptr);

    ASSERT_EQ(solverPSOR_ptr->GetType(), solverBase_ptr->GetType());
}

TEST(ChArchiveJSON, nullpointers) {
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    std::string outputfile = std::string(::testing::UnitTest::GetInstance()->current_test_suite()->name()) + "_" +
                             std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());

    outputfile = out_dir + outputfile;

    {
        std::ofstream mfileo(outputfile + ".json");
        ChArchiveOutJSON archive_out(mfileo);

        ChVector3d* chvector_nullptr = nullptr;
        archive_out << CHNVP(chvector_nullptr);

        ChSolver* chsolver_nullptr = nullptr;
        archive_out << CHNVP(chsolver_nullptr);
    }

    std::ifstream mfilei(outputfile + ".json");
    ChArchiveInJSON archive_in(mfilei);

    ChVector3d* chvector_nullptr;
    ChSolver* chsolver_nullptr;

    archive_in >> CHNVP(chvector_nullptr);
    archive_in >> CHNVP(chsolver_nullptr);

    ASSERT_EQ(chvector_nullptr, nullptr);
    ASSERT_EQ(chsolver_nullptr, nullptr);
}

//
////TEST(ChArchive, shaft_JSON){
// int main(){
//
//
//
//
//     double timestep = 0.01;
//     int step_num = 200;
//
//     double shaft0_pos_before_archive;
//     double shaft1_posdt_before_archive;
//
//     {
//         ChSystemNSC system;
//
//         // Create shaft A, with applied torque
//         auto shaftA = chrono_types::make_shared<ChShaft>();
//         shaftA->SetInertia(0.5);
//         shaftA->SetAppliedLoad(10);
//         system.Add(shaftA);
//
//         // Create shaft B
//         auto shaftB = chrono_types::make_shared<ChShaft>();
//         shaftB->SetInertia(0.5);
//         system.Add(shaftB);
//
//         // Create shaft C, that will be fixed (to be used as truss of epicycloidal reducer)
//         auto shaftC = chrono_types::make_shared<ChShaft>();
//         shaftC->SetFixed(true);
//         system.Add(shaftC);
//
//         // Create a ChShaftsPlanetary, that represents a simplified model
//         // of a planetary gear between THREE ChShaft objects (ex.: a car differential)
//         // An epicycloidal reducer is a special type of planetary gear.
//         auto planetaryBAC = chrono_types::make_shared<ChShaftsPlanetary>();
//         planetaryBAC->Initialize(shaftB, shaftA, shaftC);  // output, carrier, fixed
//
//         // We can set the ratios of the planetary using a simplified formula, for the
//         // so called 'Willis' case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
//         // and leave free the truss C (the outer gear with inner teeth in our reducer); which is
//         // the transmission ratio t0 that we get? It is simply t0=-Za/Zc, with Z = num of teeth of gears.
//         // So just use the following to set all the three ratios automatically:
//         double t0 = -50.0 / 100.0;  // suppose, in the reducer, that pinion A has 50 teeth and truss has 100 inner
//         teeth. planetaryBAC->SetTransmissionRatioOrdinary(t0); system.Add(planetaryBAC);
//
//         // Now, let's make a shaft D, that is fixed, and used for the right side
//         // of a clutch (so the clutch will act as a brake).
//         auto shaftD = chrono_types::make_shared<ChShaft>();
//         shaftD->SetFixed(true);
//         system.Add(shaftD);
//
//         // Make the brake. It is, in fact a clutch between shafts B and D, where
//         // D is fixed as a truss, so the clutch will operate as a brake.
//         auto clutchBD = chrono_types::make_shared<ChShaftsClutch>();
//         clutchBD->Initialize(shaftB, shaftD);
//         clutchBD->SetTorqueLimit(60);
//         system.Add(clutchBD);
//
//         std::ofstream mfileo("ChArchiveJSON_shafts_out.json");
//         ChArchiveOutJSON archive_out(mfileo);
//         archive_out << CHNVP(system);
//
//
//         system.Update();
//         // Simulation loop
//         for (int step = 0; step<step_num; ++step) {
//             system.DoStepDynamics(timestep);
//         }
//
//         shaft0_pos_before_archive = system.GetShafts()[0]->GetPos();
//         shaft1_posdt_before_archive = system.GetShafts()[1]->GetPosDt();
//
//     }
//
//     std::ifstream mfilei("ChArchiveJSON_shafts_out.json");
//     ChArchiveInJSON archive_in(mfilei);
//     ChSystemNSC system;
//     archive_in >> CHNVP(system);
//
//
//     // Simulation loop
//     for (int step = 0; step<step_num; ++step) {
//         system.DoStepDynamics(timestep);
//     }
//
//     double shaft0_pos_after_archive = system.GetShafts()[0]->GetPos();
//     double shaft1_posdt_after_archive = system.GetShafts()[1]->GetPosDt();
//
//     //ASSERT_NEAR(shaft0_pos_before_archive, shaft0_pos_after_archive, ABS_ERR);
//     //ASSERT_NEAR(shaft1_posdt_before_archive, shaft1_posdt_after_archive, ABS_ERR);
//
// }

TEST(ChArchiveJSON, ChVectorDynamicTest) {
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    std::string outputfile = std::string(::testing::UnitTest::GetInstance()->current_test_suite()->name()) + "_" +
                             std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
                             std::string(".json");

    outputfile = out_dir + outputfile;

    ChVectorDynamic<> myVect_before;
    {
        ChVectorDynamic<> myVect;
        myVect.resize(3);
        myVect[0] = 1.0;
        myVect[1] = 2.0;
        myVect[2] = 3.0;
        myVect_before = myVect;

        std::ofstream mfileo(outputfile);
        ChArchiveOutJSON archive_out(mfileo);
        archive_out << CHNVP(myVect);
    }

    std::ifstream mfilei(outputfile);
    ChArchiveInJSON archive_in(mfilei);
    ChVectorDynamic<> myVect;
    archive_in >> CHNVP(myVect);

    ASSERT_DOUBLE_EQ(myVect_before.x(), myVect.x());
    ASSERT_DOUBLE_EQ(myVect_before.y(), myVect.y());
    ASSERT_DOUBLE_EQ(myVect_before.z(), myVect.z());
}
