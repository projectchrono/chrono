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
#include "chrono/serialization/ChArchive.h"

//#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
//const double ABS_ERR = 1e-8;

//
//void assemble_fourbar(ChSystem& system){
//    system.Set_G_acc(ChVector<>(0, -9.81, 0));
//
//    // Joint coords
//    ChFrame<> frameO(ChVector<>(0, 0, 0), QUNIT);
//    ChFrame<> frameA(ChVector<>(1, 1, 0), QUNIT);
//    ChFrame<> frameB(ChVector<>(3, 1, 0), QUNIT);
//    ChFrame<> frameC(ChVector<>(3, 0, 0), QUNIT);
//
//    // Bodies
//    auto floor = chrono_types::make_shared<ChBody>();
//    floor->SetBodyFixed(true);
//    system.Add(floor);
//
//    auto crank = chrono_types::make_shared<ChBody>();
//    crank->SetPos((frameO.GetPos() + frameA.GetPos()) / 2.);
//    system.Add(crank);
//
//    auto rod = chrono_types::make_shared<ChBody>();
//    rod->SetPos((frameA.GetPos() + frameB.GetPos()) / 2.);
//    system.Add(rod);
//
//    auto rocker = chrono_types::make_shared<ChBody>();
//    rocker->SetPos((frameB.GetPos() + frameC.GetPos()) / 2.);
//    system.Add(rocker);
//
//    // Links
//    //auto linkO = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
//    //linkO->Initialize(crank, floor, frameO);
//    //linkO->SetName("linkO");
//    //system.Add(linkO);
//
//    auto linkO = chrono_types::make_shared<ChLinkMotorRotationAngle>();
//    linkO->SetName("linkO");
//    linkO->Initialize(crank, floor, ChFrame<>());
//    linkO->SetMotorFunction(chrono_types::make_shared<ChFunction_Poly345>(3.14, 1));
//    system.Add(linkO);
//
//    auto linkA = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
//    linkA->Initialize(rod, crank, frameA);
//    linkA->SetName("linkA");
//    system.Add(linkA);
//
//    auto linkB = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
//    linkB->Initialize(rocker, rod, frameB);
//    linkB->SetName("linkB");
//    system.Add(linkB);
//
//    auto linkC = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
//    linkC->Initialize(rocker, floor, frameC);
//    //linkC->Initialize(floor, rocker, frameC);
//    linkC->SetName("linkC");
//    system.Add(linkC);
//
//}



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


////TEST(ChArchive, ChArchiveJSON){
//int main(){
//
//
//
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        ChSystemNSC system;
//        //assemble_fourbar(system);
//        assemble_pendulum(system);
//
//        std::string jsonfile = "ChArchiveJSON_out.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//        marchiveout << CHNVP(system);
//
//
//        for (int step = 0; step<step_num; ++step) {
//            system.DoStepDynamics(timestep);
//        }
//
//        before_archive = system.Get_bodylist()[1]->GetPos();
//
//    }
//
//    std::string jsonfile = "ChArchiveJSON_out.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//    ChSystemNSC system;
//    marchivein >> CHNVP(system);
//    
//    for (auto& body : system.Get_bodylist()) {
//        body->GetBodyFixed();
//    }
//
//    for (auto& link : system.Get_linklist()) {
//        link->IsActive();
//    }
//
//
//    // Simulation loop
//    for (int step = 0; step<step_num; ++step) {
//        system.DoStepDynamics(timestep);
//    }
//
//    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();
//
//    //ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
//    //ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
//    //ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());
//
//}

////TEST(ChArchive, ChArchiveJSON){
//int main(){
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        ChSystemNSC system;
//        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
//        mat->SetFriction(0.4f);
//        mat->SetCompliance(0.0);
//        mat->SetComplianceT(0.0);
//        mat->SetDampingF(0.2f);
//
//        auto visual_body = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
//                                                            100,         // density
//                                                            true,        // visualization?
//                                                            false,        // collision?
//                                                            mat);        // contact material
//        visual_body->SetName("ThisIsVisualBody");
//        system.Add(visual_body);
//
//        std::string jsonfile = "ChArchiveJSON_visual.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//        marchiveout << CHNVP(system);
//
//
//        for (int step = 0; step<step_num; ++step) {
//            system.DoStepDynamics(timestep);
//        }
//
//        before_archive = system.Get_bodylist()[0]->GetPos();
//
//    }
//
//    std::string jsonfile = "ChArchiveJSON_visual.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//    ChSystemNSC system;
//    marchivein >> CHNVP(system);
//    
//    // Simulation loop
//    for (int step = 0; step<step_num; ++step) {
//        system.DoStepDynamics(timestep);
//    }
//
//    ChVector<> after_archive = system.Get_bodylist()[0]->GetPos();
//
//    std::cout << "before_archive: " << before_archive << std::endl;
//    std::cout << "after_archive: " << after_archive << std::endl;
//
//    //ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
//    //ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
//    //ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());
//
//}


////TEST(ChArchive, ChArchiveJSON){
//int main(){
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
//        mat->SetFriction(0.4f);
//        mat->SetCompliance(0.0);
//        mat->SetComplianceT(0.0);
//        mat->SetDampingF(0.2f);
//
//        auto visual_body = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
//                                                            100,         // density
//                                                            true,        // visualization?
//                                                            false,        // collision?
//                                                            mat);        // contact material
//        visual_body->SetName("ThisIsVisualBody");
//
//        std::string jsonfile = "ChArchiveJSON_ChBodyEasyBox.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//        marchiveout << CHNVP(visual_body);
//
//    }
//
//    std::string jsonfile = "ChArchiveJSON_ChBodyEasyBox.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//
//    std::shared_ptr<ChBodyEasyBox> visual_body;
//    marchivein >> CHNVP(visual_body);
//}
//
////TEST(ChArchive, ChArchiveJSON){
//int main(){
//    double timestep = 0.01;
//    int step_num = 200;
//
//    ChVector<> before_archive;
//
//    {
//        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
//        mat->SetFriction(0.4f);
//        mat->SetCompliance(0.0);
//        mat->SetComplianceT(0.0);
//        mat->SetDampingF(0.2f);
//
//        //auto visual_body = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
//        //                                                    100,         // density
//        //                                                    true,        // visualization?
//        //                                                    false,        // collision?
//        //                                                    mat);        // contact material
//        //visual_body->SetName("ThisIsVisualBody");
//
//        auto boxshape = chrono_types::make_shared<ChBoxShape>();
//        auto visualmodel = chrono_types::make_shared<ChVisualModel>();
//        visualmodel->AddShape(boxshape);
//
//        std::string jsonfile = "ChArchiveJSON_ChVisualModel.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//
//        marchiveout << CHNVP(visualmodel);
//
//    }
//
//    std::string jsonfile = "ChArchiveJSON_ChVisualModel.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//
//    std::shared_ptr<ChVisualModel> visualmodel;
//    marchivein >> CHNVP(visualmodel);
//    visualmodel->GetShape(0);
//    
//
//}

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

//int main(){
//    {
//        ChVectorDynamic<> myVect;
//        myVect.resize(3);
//        myVect[0] = 1.0;
//        myVect[1] = 2.0;
//        myVect[2] = 3.0;
//
//        std::string jsonfile = "ChArchiveJSON_ChVectorDynamic.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//        ChArchiveOutJSON marchiveout(mfileo);
//        marchiveout << CHNVP(myVect);
//    }
//
//    std::string jsonfile = "ChArchiveJSON_ChVectorDynamic.json";
//    ChStreamInAsciiFile mfilei(jsonfile.c_str());
//    ChArchiveInJSON marchivein(mfilei);
//    ChVectorDynamic<> myVect;
//    marchivein >> CHNVP(myVect);
//}

std::string assemble_gear_and_pulleys(ChSystem& sys){


        sys.Set_G_acc(ChVector<>(0, -10, 0));
        // Create a Chrono physical system

        // Contact material shared among all bodies
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        // Shared visualization material
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png")); // TODO: DARIOM

        // Create all the rigid bodies.

        double radA = 2;
        double radB = 4;

        // ...the truss
        auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
        sys.Add(mbody_truss);
        mbody_truss->SetBodyFixed(true);
        mbody_truss->SetPos(ChVector<>(0, 0, 3));

        //// ...the rotating bar support for the two epicycloidal wheels
        //auto mbody_train = chrono_types::make_shared<ChBodyEasyBox>(8, 1.5, 1.0, 1000, true, false, mat);
        //sys.Add(mbody_train);
        //mbody_train->SetPos(ChVector<>(3, 0, 0));

        //// ...which must rotate respect to truss along Z axis, in 0,0,0,
        //auto link_revoluteTT = chrono_types::make_shared<ChLinkLockRevolute>();
        //link_revoluteTT->Initialize(mbody_truss, mbody_train, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
        //sys.AddLink(link_revoluteTT);

        // ...the first gear
        auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radA, 0.5, 1000, true, false, mat);
        //auto mbody_gearA = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
        sys.Add(mbody_gearA);
        mbody_gearA->SetPos(ChVector<>(0, 0, -1));
        mbody_gearA->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        mbody_gearA->GetVisualShape(0)->SetMaterial(0, vis_mat);

        //// for aesthetic reasons, also add a thin cylinder only as a visualization:
        //auto mshaft_shape = chrono_types::make_shared<ChCylinderShape>(radA * 0.4, 13);
        //mbody_gearA->AddVisualShape(mshaft_shape, ChFrame<>(ChVector<>(0, 3.5, 0), Q_from_AngX(CH_C_PI_2)));

        // ...impose rotation speed between the first gear and the fixed truss
        auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
        link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(6));
        sys.AddLink(link_motor);

        // ...the second gear
        double interaxis12 = radA + radB;
        auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radB, 0.4, 1000, true, false, mat);
        sys.Add(mbody_gearB);
        mbody_gearB->SetPos(ChVector<>(interaxis12, 0, -1));
        mbody_gearB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        mbody_gearB->GetVisualShape(0)->SetMaterial(0, vis_mat);

        // ... the second gear is fixed to the rotating bar
        auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        link_revolute->Initialize(mbody_gearB, mbody_truss, ChCoordsys<>(ChVector<>(interaxis12, 0, 0), QUNIT)); // TEMP
        //link_revolute->Initialize(mbody_gearB, mbody_train, ChCoordsys<>(ChVector<>(interaxis12, 0, 0), QUNIT));
        sys.AddLink(link_revolute);

        // ...the gear constraint between the two wheels A and B.
        //    As transmission ratio (=speed of wheel B / speed of wheel A) to enter in  Set_tau(), we
        //    could use whatever positive value we want: the ChLinkGear will compute the two radii of the
        //    wheels for its 'hidden' computations, given the distance between the two axes. However, since
        //    we already build two '3D cylinders' bodies -just for visualization reasons!- with radA and radB,
        //    we must enter Set_tau(radA/radB).
        //    Also, note that the initial position of the constraint has no importance (simply use CSYSNORM),
        //    but we must set where the two axes are placed in the local coordinates of the two wheels, so
        //    we use Set_local_shaft1() and pass some local ChFrame. Note that, since the Z axis of that frame
        //    will be considered the axis of the wheel, we must rotate the frame 90° with Q_from_AngAxis(), because
        //    we created the wheel with ChBodyEasyCylinder() which created a cylinder with Y as axis.

        auto link_gearAB = chrono_types::make_shared<ChLinkGear>();
        link_gearAB->Initialize(mbody_gearA, mbody_gearB, CSYSNORM);
        link_gearAB->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        link_gearAB->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        link_gearAB->Set_tau(radA / radB);
        link_gearAB->Set_checkphase(true);
        sys.AddLink(link_gearAB);

        //// ...the gear constraint between the second wheel B and a large wheel C with inner teeth, that
        ////    does not necessarily need to be created as a new body because it is the 'fixed' part of the
        ////    epicycloidal reducer, so, as wheel C, we will simply use the ground object 'mbody_truss'.
        //double radC = 2 * radB + radA;
        //auto link_gearBC = chrono_types::make_shared<ChLinkGear>();
        //link_gearBC->Initialize(mbody_gearB, mbody_truss, CSYSNORM);
        //link_gearBC->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        //link_gearBC->Set_local_shaft2(ChFrame<>(ChVector<>(0, 0, -4), QUNIT));
        //link_gearBC->Set_tau(radB / radC);
        //link_gearBC->Set_epicyclic(true);  // <-- this means: use a wheel with internal teeth!
        //sys.AddLink(link_gearBC);

        //// ...the bevel gear at the side,
        //double radD = 5;
        //auto mbody_gearD =
        //    chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radD, 0.8, 1000, true, false, mat);
        //sys.Add(mbody_gearD);
        //mbody_gearD->SetPos(ChVector<>(-10, 0, -9));
        //mbody_gearD->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        //mbody_gearD->GetVisualShape(0)->SetMaterial(0, vis_mat);

        //// ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
        ////     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
        //auto link_revoluteD = chrono_types::make_shared<ChLinkLockRevolute>();
        //link_revoluteD->Initialize(mbody_gearD, mbody_truss,
        //                           ChCoordsys<>(ChVector<>(-10, 0, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        //sys.AddLink(link_revoluteD);

        //// ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear: Chrono does not require
        ////     special info for this case -the position of the two shafts and the transmission ratio are enough-
        //auto link_gearAD = chrono_types::make_shared<ChLinkGear>();
        //link_gearAD->Initialize(mbody_gearA, mbody_gearD, CSYSNORM);
        //link_gearAD->Set_local_shaft1(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        //link_gearAD->Set_local_shaft2(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        //link_gearAD->Set_tau(1);
        //sys.AddLink(link_gearAD);

        //// ...the pulley at the side,
        //double radE = 2;
        //auto mbody_pulleyE =
        //    chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radE, 0.8, 1000, true, false, mat);
        //sys.Add(mbody_pulleyE);
        //mbody_pulleyE->SetPos(ChVector<>(-10, -11, -9));
        //mbody_pulleyE->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        //mbody_pulleyE->GetVisualShape(0)->SetMaterial(0, vis_mat);

        //// ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
        ////     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
        //auto link_revoluteE = chrono_types::make_shared<ChLinkLockRevolute>();
        //link_revoluteE->Initialize(mbody_pulleyE, mbody_truss,
        //                           ChCoordsys<>(ChVector<>(-10, -11, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        //sys.AddLink(link_revoluteE);

        //// ... Let's make a synchro belt constraint between pulley D and pulley E. The user must be
        ////     sure that the two shafts are parallel in absolute space. Also, interaxial distance should not change.
        //auto link_pulleyDE = chrono_types::make_shared<ChLinkPulley>();
        //link_pulleyDE->Initialize(mbody_gearD, mbody_pulleyE, CSYSNORM);
        //link_pulleyDE->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        //link_pulleyDE->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
        //link_pulleyDE->Set_r1(radD);
        //link_pulleyDE->Set_r2(radE);
        //link_pulleyDE->Set_checkphase(
        //    true);  // synchro belts don't tolerate slipping: this avoids it as numerical errors accumulate.
        //sys.AddLink(link_pulleyDE);
        return std::string("ChArchiveJSON_gears.json");
}


std::string assemble_pendulum_visual(ChSystem& system){

    system.Set_G_acc(ChVector<>(0.0, -9.81, 0.0));

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    floor->SetName("floor");
    floor->SetIdentifier(100);
    system.Add(floor);




    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    auto moving_body = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
                                                        100,         // density
                                                        true,        // visualization?
                                                        false,        // collision?
                                                        mat);        // contact material
    //auto moving_body = chrono_types::make_shared<ChBody>();
    moving_body->SetPos(ChVector<>(1.0, -1.0, 1.0));
    moving_body->SetName("moving_body");
    moving_body->SetIdentifier(101);
    system.Add(moving_body);


    //auto link = chrono_types::make_shared<ChLinkMateRevolute>();
    //link->Initialize(moving_body, floor, ChFrame<>());
    auto link = chrono_types::make_shared<ChLinkLockRevolute>();
    link->Initialize(moving_body, floor, ChCoordsys<>());
    system.Add(link);

    return std::string("ChArchiveJSON_visual.json");
}

int main(){

    std::string jsonfile;
    // Create a material that will be shared among all collision shapes
    {

        ChSystemNSC sys;
        
        //assemble_pendulum_visual(sys);
        jsonfile = assemble_gear_and_pulleys(sys);
        

        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
        ChArchiveOutJSON marchiveout(mfileo);
        marchiveout << CHNVP(sys);


        
        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle(jsonfile);
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector<>(12, 15, -20));
        vis->AddTypicalLights();



        double timestep = 0.001;
        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            sys.DoStepDynamics(timestep);

            vis->EndScene();

        }

    }

    ChSystemNSC sys;

    ChStreamInAsciiFile mfilei(jsonfile.c_str());
    ChArchiveInJSON marchivein(mfilei);
    marchivein >> CHNVP(sys);

    {

        ChStreamOutAsciiFile mfileo("ChArchiveJSON_gears2.json");
        ChArchiveOutJSON marchiveout(mfileo);
        marchiveout << CHNVP(sys);
    }


    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle(jsonfile);
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(12, 15, -20));
    vis->AddTypicalLights();



    double timestep = 0.001;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        sys.DoStepDynamics(timestep);

        vis->EndScene();

    }


}