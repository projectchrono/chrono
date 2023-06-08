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


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/serialization/ChArchive.h"


using namespace chrono;

const double ABS_ERR = 1e-8;


void assemble_fourbar(ChSystem& system){
    system.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Joint coords
    ChFrame<> frameO(ChVector<>(0, 0, 0), QUNIT);
    ChFrame<> frameA(ChVector<>(1, 1, 0), QUNIT);
    ChFrame<> frameB(ChVector<>(3, 1, 0), QUNIT);
    ChFrame<> frameC(ChVector<>(3, 0, 0), QUNIT);

    // Bodies
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
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

    // Links
    //auto linkO = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    //linkO->Initialize(crank, floor, frameO);
    //linkO->SetName("linkO");
    //system.Add(linkO);

    auto linkO = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    linkO->SetName("linkO");
    linkO->Initialize(crank, floor, ChFrame<>());
    linkO->SetMotorFunction(chrono_types::make_shared<ChFunction_Poly345>(3.14, 1));
    system.Add(linkO);

    auto linkA = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    linkA->Initialize(rod, crank, frameA);
    linkA->SetName("linkA");
    system.Add(linkA);

    auto linkB = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    linkB->Initialize(rocker, rod, frameB);
    linkB->SetName("linkB");
    system.Add(linkB);

    auto linkC = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    linkC->Initialize(rocker, floor, frameC);
    //linkC->Initialize(floor, rocker, frameC);
    linkC->SetName("linkC");
    system.Add(linkC);

}



void assemble_pendulum(ChSystem& system){

    system.Set_G_acc(ChVector<>(0.0, -9.81, 0.0));

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    floor->SetName("floor");
    floor->SetIdentifier(100);
    system.Add(floor);


    auto moving_body = chrono_types::make_shared<ChBody>();
    moving_body->SetPos(ChVector<>(1.0, -1.0, 1.0));
    moving_body->SetName("moving_body");
    moving_body->SetIdentifier(101);
    system.Add(moving_body);


    auto link = chrono_types::make_shared<ChLinkMateRevolute>();
    link->Initialize(moving_body, floor, ChFrame<>());
    //auto link = chrono_types::make_shared<ChLinkLockRevolute>();
    //link->Initialize(moving_body, floor, ChCoordsys<>());
    system.Add(link);

}



//int main(){
//    ChSolver* solver_baseptr;
//
//    ChClassFactory::create(std::string("ChSolverPSOR"), &solver_baseptr);
//    solver_baseptr->GetType(); // WRONG CALL
//    //ChSolverPSOR* solver_vptr_psor_correct = reinterpret_cast<ChSolverPSOR*>(solver_baseptr);
//    //solver_vptr_psor_correct->GetType();
//    //ChSolver* solver_vptr2 = static_cast<ChSolver*>(solver_vptr_psor_correct);
//    //solver_vptr2->GetType();
//
//    void* solver_vptr = getVoidPointer<ChSolver>(solver_baseptr); // nothing changes
//
//
//    void* solverbase_vptr_conv = static_cast<void*>(static_cast<ChSolver*>(reinterpret_cast<ChSolverPSOR*>(solver_vptr)));
//    ChSolver* solverbase_ptr_conv = reinterpret_cast<ChSolver*>(solverbase_vptr_conv);
//    solverbase_ptr_conv->GetType();
//}




////TEST(ChArchive, ChArchiveJSON_solver){
//int main(){
//    //{
//    //    ChSolver* solver_baseptr;
//
//    //    ChClassFactory::create(std::string("ChSolverPSOR"), &solver_baseptr);
//    //    solver_baseptr->GetType(); // WRONG CALL
//    //    //ChSolverPSOR* solver_vptr_psor_correct = reinterpret_cast<ChSolverPSOR*>(solver_baseptr);
//    //    //solver_vptr_psor_correct->GetType();
//    //    //ChSolver* solver_vptr2 = static_cast<ChSolver*>(solver_vptr_psor_correct);
//    //    //solver_vptr2->GetType();
//
//    //    void* solver_vptr = getVoidPointer<ChSolver>(solver_baseptr); // nothing changes
//
//
//    //    void* solverbase_vptr_conv = static_cast<void*>(static_cast<ChSolver*>(reinterpret_cast<ChSolverPSOR*>(solver_vptr)));
//    //    ChSolver* solverbase_ptr_conv = reinterpret_cast<ChSolver*>(solverbase_vptr_conv);
//    //    solverbase_ptr_conv->GetType();
//    //    std::cout << "Expected ChSolver*: " << solverbase_ptr_conv << std::endl;
//
//    //    void* solverbase_vptr_convauto = ChCastingMap::Convert("ChSolverPSOR", "ChSolver", solver_vptr);
//    //    ChSolver* solverbase_ptr_convauto = reinterpret_cast<ChSolver*>(solverbase_vptr_convauto);
//    //    solverbase_ptr_convauto->GetType();
//    //    std::cout << "Resulting ChSolver*: " << solverbase_ptr_convauto << std::endl;
//
//    //}
//
//    {
//        ChSolverPSOR* solverPSOR_ptr = new ChSolverPSOR();
//        ChIterativeSolverVI* solverISVI_ptr = solverPSOR_ptr;
//        ChIterativeSolver* solverIS_ptr = solverPSOR_ptr;
//        ChSolverVI* solverVI_ptr = solverPSOR_ptr;
//        ChSolver* solverBase_ptr = solverPSOR_ptr;
//
//
//        solverPSOR_ptr->GetType();
//        solverBase_ptr->GetType();
//
//        std::cout << "solverPSOR_ptr : " << solverPSOR_ptr << std::endl;
//        std::cout << "solverISVI_ptr : " << solverISVI_ptr << std::endl;
//        std::cout << "solverIS_ptr   : " << solverIS_ptr << std::endl;
//        std::cout << "solverVI_ptr   : " << solverVI_ptr << std::endl;
//        std::cout << "solverBase_ptr : " << solverBase_ptr << std::endl;
//
//        std::string jsonfile = "ChArchiveJSON_solver_out.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//
//        marchiveout << CHNVP(solverBase_ptr);
//
//        delete solverPSOR_ptr;
//    }
//
//    std::string jsonfile = "ChArchiveJSON_solver_out.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//    ChSolver* solverBase_ptr;
//    marchivein >> CHNVP(solverBase_ptr);
//
//    ChSolverPSOR* solverPSOR_ptr = dynamic_cast<ChSolverPSOR*>(solverBase_ptr);
//
//    solverPSOR_ptr->GetType();
//    solverBase_ptr->GetType();
//    
//
//    //ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
//    //ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
//    //ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());
//
//}

//TEST(ChArchive, ChArchiveJSON){
int main(){



    double timestep = 0.01;
    int step_num = 200;

    ChVector<> before_archive;

    {
        ChSystemNSC system;
        //assemble_fourbar(system);
        assemble_pendulum(system);

        std::string jsonfile = "ChArchiveJSON_out.json";
        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
        ChArchiveOutJSON marchiveout(mfileo);
        marchiveout << CHNVP(system);


        for (int step = 0; step<step_num; ++step) {
            system.DoStepDynamics(timestep);
        }

        before_archive = system.Get_bodylist()[1]->GetPos();

    }

    std::string jsonfile = "ChArchiveJSON_out.json";
    ChStreamInAsciiFile mfilei(jsonfile.c_str());
    ChArchiveInJSON marchivein(mfilei);
    ChSystemNSC system;
    marchivein >> CHNVP(system);
    
    for (auto& body : system.Get_bodylist()) {
        body->GetBodyFixed();
    }

    for (auto& link : system.Get_linklist()) {
        link->IsActive();
    }


    // Simulation loop
    for (int step = 0; step<step_num; ++step) {
        system.DoStepDynamics(timestep);
    }

    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();

    //ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
    //ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
    //ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());

}

//TEST(ChArchive, ChArchiveXML){
//
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        ChSystemNSC system;
//        assemble_fourbar(system);
//
//        std::string xmlfile = "ChArchiveXML_out.xml";
//        ChStreamOutAsciiFile mfileo(xmlfile.c_str());
//        ChArchiveOutXML marchiveout(mfileo);
//        marchiveout << CHNVP(system);
//
//        for (int step = 0; step<step_num; ++step) {
//            system.DoStepDynamics(timestep);
//        }
//
//        before_archive = system.Get_bodylist()[1]->GetPos();
//
//    }
//
//    std::string xmlfile = "ChArchiveXML_out.xml";
//    ChStreamInAsciiFile mfilei(xmlfile.c_str());
//    ChArchiveInXML marchivein(mfilei);
//    ChSystemNSC system;
//    marchivein >> CHNVP(system);
//    
//
//    // Simulation loop
//    for (int step = 0; step<step_num; ++step) {
//        system.DoStepDynamics(timestep);
//    }
//
//    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();
//
//    ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
//    ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
//    ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());
//
//}
//
//TEST(ChArchive, ChArchiveBinary){
////int main(){
//
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        ChSystemNSC system;
//        assemble_fourbar(system);
//
//        std::string binfile = "ChArchiveBinary_out.dat";
//        ChStreamOutBinaryFile mfileo(binfile.c_str());
//        ChArchiveOutBinary marchiveout(mfileo);
//        marchiveout << CHNVP(system);
//
//        for (int step = 0; step<step_num; ++step) {
//            system.DoStepDynamics(timestep);
//        }
//
//        before_archive = system.Get_bodylist()[1]->GetPos();
//
//    }
//
//    std::string binfile = "ChArchiveBinary_out.dat";
//    ChStreamInBinaryFile mfilei(binfile.c_str());
//    ChArchiveInBinary marchivein(mfilei);
//    ChSystemNSC system;
//    marchivein >> CHNVP(system);
//    
//
//    // Simulation loop
//    for (int step = 0; step<step_num; ++step) {
//        system.DoStepDynamics(timestep);
//    }
//
//    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();
//
//    ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
//    ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
//    ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());
//
//}

//
////TEST(ChArchive, shaft_JSON){
//int main(){
//
//
//    
//    
//    double timestep = 0.01;
//    int step_num = 200;
//
//    double shaft0_pos_before_archive;
//    double shaft1_posdt_before_archive;
//
//    {
//        ChSystemNSC system;
//
//        // Create shaft A, with applied torque
//        auto shaftA = chrono_types::make_shared<ChShaft>();
//        shaftA->SetInertia(0.5);
//        shaftA->SetAppliedTorque(10);
//        system.Add(shaftA);
//
//        // Create shaft B
//        auto shaftB = chrono_types::make_shared<ChShaft>();
//        shaftB->SetInertia(0.5);
//        system.Add(shaftB);
//
//        // Create shaft C, that will be fixed (to be used as truss of epicycloidal reducer)
//        auto shaftC = chrono_types::make_shared<ChShaft>();
//        shaftC->SetShaftFixed(true);
//        system.Add(shaftC);
//
//        // Create a ChShaftsPlanetary, that represents a simplified model
//        // of a planetary gear between THREE ChShaft objects (ex.: a car differential)
//        // An epicycloidal reducer is a special type of planetary gear.
//        auto planetaryBAC = chrono_types::make_shared<ChShaftsPlanetary>();
//        planetaryBAC->Initialize(shaftB, shaftA, shaftC);  // output, carrier, fixed
//
//        // We can set the ratios of the planetary using a simplified formula, for the
//        // so called 'Willis' case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
//        // and leave free the truss C (the outer gear with inner teeth in our reducer); which is
//        // the transmission ratio t0 that we get? It is simply t0=-Za/Zc, with Z = num of teeth of gears.
//        // So just use the following to set all the three ratios automatically:
//        double t0 = -50.0 / 100.0;  // suppose, in the reducer, that pinion A has 50 teeth and truss has 100 inner teeth.
//        planetaryBAC->SetTransmissionRatioOrdinary(t0);
//        system.Add(planetaryBAC);
//
//        // Now, let's make a shaft D, that is fixed, and used for the right side
//        // of a clutch (so the clutch will act as a brake).
//        auto shaftD = chrono_types::make_shared<ChShaft>();
//        shaftD->SetShaftFixed(true);
//        system.Add(shaftD);
//
//        // Make the brake. It is, in fact a clutch between shafts B and D, where
//        // D is fixed as a truss, so the clutch will operate as a brake.
//        auto clutchBD = chrono_types::make_shared<ChShaftsClutch>();
//        clutchBD->Initialize(shaftB, shaftD);
//        clutchBD->SetTorqueLimit(60);
//        system.Add(clutchBD);
//
//        std::string jsonfile = "ChArchiveJSON_shafts_out.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//        marchiveout << CHNVP(system);
//
//
//
//        ChSolver* s_ptr = system.GetSolver().get();
//        ChSolverPSOR* s_der_ptr = dynamic_cast<ChSolverPSOR*>(s_ptr);
//
//        system.Update();
//        // Simulation loop
//        for (int step = 0; step<step_num; ++step) {
//            system.DoStepDynamics(timestep);
//        }
//
//        shaft0_pos_before_archive = system.Get_shaftlist()[0]->GetPos();
//        shaft1_posdt_before_archive = system.Get_shaftlist()[1]->GetPos_dt();
//
//    }
//
//    std::string jsonfile = "ChArchiveJSON_shafts_out.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//    ChSystemNSC system;
//    marchivein >> CHNVP(system);
//    
//
//    // Simulation loop
//    for (int step = 0; step<step_num; ++step) {
//        system.DoStepDynamics(timestep);
//    }
//
//    double shaft0_pos_after_archive = system.Get_shaftlist()[0]->GetPos();
//    double shaft1_posdt_after_archive = system.Get_shaftlist()[1]->GetPos_dt();
//
//    //ASSERT_NEAR(shaft0_pos_before_archive, shaft0_pos_after_archive, ABS_ERR);
//    //ASSERT_NEAR(shaft1_posdt_before_archive, shaft1_posdt_after_archive, ABS_ERR);
//
//}
//


