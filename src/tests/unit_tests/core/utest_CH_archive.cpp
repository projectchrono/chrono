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
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/serialization/ChArchiveExplorer.h"

#include "chrono/core/ChGlobal.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChException.h"
#include "chrono/solver/ChConstraintTuple.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_thirdparty/filesystem/path.h"


using namespace chrono;


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

TEST(ChArchive, ChArchiveJSON){

    double timestep = 0.01;
    int step_num = 200;

    ChVector<> before_archive;

    {
        ChSystemNSC system;
        assemble_fourbar(system);

        std::string jsonfile = "foo_archive.json";
        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
        ChArchiveOutJSON marchiveout(mfileo);
        marchiveout << CHNVP(system);

        for (int step = 0; step<step_num; ++step) {
            system.DoStepDynamics(timestep);
        }

        before_archive = system.Get_bodylist()[1]->GetPos();

    }

    std::string jsonfile = "foo_archive.json";
    ChStreamInAsciiFile mfilei(jsonfile.c_str());
    ChArchiveInJSON marchivein(mfilei);
    ChSystemNSC system;
    marchivein >> CHNVP(system);
    

    // Simulation loop
    for (int step = 0; step<step_num; ++step) {
        system.DoStepDynamics(timestep);
    }

    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();

    ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
    ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
    ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());

}

TEST(ChArchive, ChArchiveXML){

    double timestep = 0.01;
    int step_num = 200;

    ChVector<> before_archive;

    {
        ChSystemNSC system;
        assemble_fourbar(system);

        std::string xmlfile = "foo_archive.xml";
        ChStreamOutAsciiFile mfileo(xmlfile.c_str());
        ChArchiveOutXML marchiveout(mfileo);
        marchiveout << CHNVP(system);

        for (int step = 0; step<step_num; ++step) {
            system.DoStepDynamics(timestep);
        }

        before_archive = system.Get_bodylist()[1]->GetPos();

    }

    std::string xmlfile = "foo_archive.xml";
    ChStreamInAsciiFile mfilei(xmlfile.c_str());
    ChArchiveInXML marchivein(mfilei);
    ChSystemNSC system;
    marchivein >> CHNVP(system);
    

    // Simulation loop
    for (int step = 0; step<step_num; ++step) {
        system.DoStepDynamics(timestep);
    }

    ChVector<> after_archive = system.Get_bodylist()[1]->GetPos();

    ASSERT_DOUBLE_EQ(before_archive.x(), after_archive.x());
    ASSERT_DOUBLE_EQ(before_archive.y(), after_archive.y());
    ASSERT_DOUBLE_EQ(before_archive.z(), after_archive.z());

}



//
//
//void AssemblePendulum(){
//    // Example: SERIALIZE TO/FROM JSON:
//    {
//        std::string jsonfile = "foo_archive.json";
//        ChStreamOutAsciiFile mfileo(jsonfile.c_str());
//
//        // Use a JSON archive object to serialize C++ objects into the file
//        ChArchiveOutJSON marchiveout(mfileo);
//
//        ChSystemNSC system;
//
//        system.Set_G_acc(ChVector<>(0.0, -9.81, 0.0));
//
//        auto floor = chrono_types::make_shared<ChBody>();
//        floor->SetBodyFixed(true);
//        floor->SetName("floor");
//        floor->SetIdentifier(100);
//        system.Add(floor);
//
//
//        auto moving_body = chrono_types::make_shared<ChBody>();
//        moving_body->SetPos(ChVector<>(1.0, -1.0, 1.0));
//        moving_body->SetName("moving_body");
//        moving_body->SetIdentifier(101);
//        system.Add(moving_body);
//
//
//        auto link = chrono_types::make_shared<ChLinkMateRevolute>();
//        link->Initialize(moving_body, floor, ChFrame<>());
//        //auto link = chrono_types::make_shared<ChLinkLockRevolute>();
//        //link->Initialize(moving_body, floor, ChCoordsys<>());
//        system.Add(link);
//
//        ChBodyFrame* Body1_test = link->GetBody1();
//        ChBody* b_empty_ptr = nullptr;
//
//        void* vptr = getVoidPointer<ChBodyFrame>(link->GetBody1());
//        ChNameValue<ChBodyFrame*> bVal = make_ChNameValue("Body1_test", Body1_test);
//        ChFunctorArchiveInSpecificPtr<ChBodyFrame> specFuncA(&bVal.value());
//        ChNameValue<ChFunctorArchiveIn> in_ref_arg = ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags());
//
//        const std::type_info& in_ref_arg_ti = typeid(in_ref_arg.value());
//        const std::type_info& vptr_ti = typeid(vptr);
//        const std::type_info& ChBodyFrame_ptr_ti = typeid(*Body1_test);
//
//        std::cout << vptr_ti.name() << std::endl;
//        std::cout << ChClassFactory::GetClassTagName(ChBodyFrame_ptr_ti) << std::endl;
//
//
//
//        marchiveout << CHNVP(system);
//
//        ChBody* moving_body_body_ptr = moving_body.get();
//        ChBodyFrame* moving_body_bodyframe_ptr = moving_body.get();
//        ChBodyFrame* link_body1_bodyframe_ptr = link->GetBody1();
//        
//        std::cout << "ChBody*(ChBody)                         : " << moving_body_body_ptr << "\n";
//        std::cout << "ChBody*(ChBody)  static_cast<void*>     : " << static_cast<void*>(moving_body_body_ptr) << "\n";
//        std::cout << "ChBody*(ChBody) dynamic_cast<void*>     : " << dynamic_cast<void*>(moving_body_body_ptr) << "\n";
//
//        std::cout << "ChBodyFrame*(ChBody)                    : " << moving_body_bodyframe_ptr << "\n";
//        std::cout << "ChBodyFrame*(ChBody)  static_cast<void*>: " << static_cast<void*>(moving_body_bodyframe_ptr) << "\n";
//        std::cout << "ChBodyFrame*(ChBody) dynamic_cast<void*>: " << dynamic_cast<void*>(moving_body_bodyframe_ptr) << "\n";
//        std::cout << "link->GetBody1()                        : " << link_body1_bodyframe_ptr << "\n";
//        std::cout << "\n";
//
//        // Simulation loop
//        while (system.GetChTime() < 2) {
//            system.DoStepDynamics(0.01);
//        }
//
//        GetLog() << "GetPos: " << system.Get_bodylist()[1]->GetPos() << "\n";
//        
//    }
//}
//
