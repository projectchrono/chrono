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

#include "chrono/core/ChClassFactory.h"


using namespace chrono;

std::string out_dir = "D:/workspace/chrono_build/bin";


void AssembleFourbar(){

    ChSystemNSC system;
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

    std::string jsonfile = out_dir + "/foo_archive.json";
    ChStreamOutAsciiFile mfileo(jsonfile.c_str());
    ChArchiveOutJSON marchiveout(mfileo);
    marchiveout << CHNVP(system);

    // Simulation loop
    while (system.GetChTime() < 2) {
        system.DoStepDynamics(0.01);
    }
    
    GetLog() << "Before archive: GetPos: " << system.Get_bodylist()[1]->GetPos() << "\n";
}

void AssemblePendulum(){
    // Example: SERIALIZE TO/FROM JSON:
    {
        std::string jsonfile = out_dir + "/foo_archive.json";
        ChStreamOutAsciiFile mfileo(jsonfile.c_str());

        // Use a JSON archive object to serialize C++ objects into the file
        ChArchiveOutJSON marchiveout(mfileo);

        ChSystemNSC system;

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

        ChBodyFrame* Body1_test = link->GetBody1();
        ChBody* b_empty_ptr = nullptr;

        void* vptr = getVoidPointer<ChBodyFrame>(link->GetBody1());
        ChNameValue<ChBodyFrame*> bVal = make_ChNameValue("Body1_test", Body1_test);
        ChFunctorArchiveInSpecificPtr<ChBodyFrame> specFuncA(&bVal.value());
        ChNameValue<ChFunctorArchiveIn> in_ref_arg = ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags());

        const std::type_info& in_ref_arg_ti = typeid(in_ref_arg.value());
        const std::type_info& vptr_ti = typeid(vptr);
        const std::type_info& ChBodyFrame_ptr_ti = typeid(*Body1_test);

        std::cout << vptr_ti.name() << std::endl;
        std::cout << ChClassFactory::GetClassTagName(ChBodyFrame_ptr_ti) << std::endl;



        marchiveout << CHNVP(system);

        ChBody* moving_body_body_ptr = moving_body.get();
        ChBodyFrame* moving_body_bodyframe_ptr = moving_body.get();
        ChBodyFrame* link_body1_bodyframe_ptr = link->GetBody1();
        
        std::cout << "ChBody*(ChBody)                         : " << moving_body_body_ptr << "\n";
        std::cout << "ChBody*(ChBody)  static_cast<void*>     : " << static_cast<void*>(moving_body_body_ptr) << "\n";
        std::cout << "ChBody*(ChBody) dynamic_cast<void*>     : " << dynamic_cast<void*>(moving_body_body_ptr) << "\n";

        std::cout << "ChBodyFrame*(ChBody)                    : " << moving_body_bodyframe_ptr << "\n";
        std::cout << "ChBodyFrame*(ChBody)  static_cast<void*>: " << static_cast<void*>(moving_body_bodyframe_ptr) << "\n";
        std::cout << "ChBodyFrame*(ChBody) dynamic_cast<void*>: " << dynamic_cast<void*>(moving_body_bodyframe_ptr) << "\n";
        std::cout << "link->GetBody1()                        : " << link_body1_bodyframe_ptr << "\n";
        std::cout << "\n";

        // Simulation loop
        while (system.GetChTime() < 2) {
            system.DoStepDynamics(0.01);
        }

        GetLog() << "GetPos: " << system.Get_bodylist()[1]->GetPos() << "\n";
        
    }
}



int main() {
    AssembleFourbar();


    std::string jsonfile = out_dir + "/foo_archive.json";
    ChStreamInAsciiFile mfilei(jsonfile.c_str());

    // Use a JSON archive object to serialize C++ objects into the file
    ChArchiveInJSON marchivein(mfilei);

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0.0, -9.81, 0.0));


    marchivein >> CHNVP(system);

    //std::shared_ptr<ChBody> floor_ptr = system.Get_bodylist()[0];




    //void* vptr = getVoidPointer<ChBodyFrame>(floor_ptr.get()); // during constructor of body, inside GetRawPtr
    //ChBodyFrame* bf_ptr = static_cast<ChBodyFrame*>(vptr); // during link pointer setup, inside SetRawPtr
    //ChBody* b_ptr = static_cast<ChBody*>(vptr);
    //ChBodyFrame* bf_ptr_converted = static_cast<ChBody*>(vptr); // what it should be done

    //std::cout << "getVoidPointer<ChBodyFrame>(floor_ptr.get())                         : " << vptr << "\n";
    //std::cout << "SetRawPtr: " << bf_ptr << "\n";
    //std::cout << "static_cast<ChBody*>(vptr): " << b_ptr << "\n";
    //std::cout << "SetRawPtr fixed     : " << bf_ptr_converted << "\n";

    //std::shared_ptr<ChBody> moving_body_ptr = system.Get_bodylist()[1];
    //std::shared_ptr<ChLinkMateRevolute> link_ptr = std::dynamic_pointer_cast<ChLinkMateRevolute>(system.Get_linklist()[0]);

    //ChBody* moving_body_body_ptr = moving_body_ptr.get();
    //ChBodyFrame* moving_body_bodyframe_ptr = moving_body_ptr.get();
    //ChBodyFrame* link_body1_bodyframe_ptr = link_ptr->GetBody1();
    //    
    //std::cout << "ChBody*(ChBody)                         : " << moving_body_body_ptr << "\n";
    //std::cout << "ChBody*(ChBody)  static_cast<void*>     : " << static_cast<void*>(moving_body_body_ptr) << "\n";
    //std::cout << "ChBody*(ChBody) dynamic_cast<void*>     : " << dynamic_cast<void*>(moving_body_body_ptr) << "\n";

    //std::cout << "ChBodyFrame*(ChBody)                    : " << moving_body_bodyframe_ptr << "\n";
    //std::cout << "ChBodyFrame*(ChBody)  static_cast<void*>: " << static_cast<void*>(moving_body_bodyframe_ptr) << "\n";
    //std::cout << "ChBodyFrame*(ChBody) dynamic_cast<void*>: " << dynamic_cast<void*>(moving_body_bodyframe_ptr) << "\n";
    //std::cout << "link->GetBody1()                        : " << link_body1_bodyframe_ptr << "\n";
    //std::cout << "\n";

    
    //ChBodyFrame* bodyframe1_ptr = link_ptr->GetBody1();
    ////ChBodyFrame* bodyframe2_ptr = link_ptr->GetBody2();
    //ChBody* body1_ptr = dynamic_cast<ChBody*>(link_ptr->GetBody1());
    ////ChBody* body2_ptr = dynamic_cast<ChBody*>(link_ptr->GetBody2());


    //std::cout << "moving_body | ChBody*(ChBody)                         : " << moving_body_ptr.get() << "\n";
    //std::cout << "moving_body | ChBody*(ChBody)  static_cast<void*>     : " << static_cast<void*>(moving_body_ptr.get()) << "\n";
    //std::cout << "moving_body | ChBody*(ChBody) dynamic_cast<void*>     : " << dynamic_cast<void*>(moving_body_ptr.get()) << "\n";


    //std::cout << "Link::Body1 | ChBody*(ChBody)                         : " << body1_ptr << "\n";
    //std::cout << "Link::Body1 | ChBody*(ChBody)  static_cast<void*>     : " << static_cast<void*>(body1_ptr) << "\n";
    //std::cout << "Link::Body1 | ChBody*(ChBody) dynamic_cast<void*>     : " << dynamic_cast<void*>(body1_ptr) << "\n";

    //std::cout << "Link::Body1 | ChBodyFrame*(ChBody)                    : " << bodyframe1_ptr << "\n";
    //std::cout << "Link::Body1 | ChBodyFrame*(ChBody)  static_cast<void*>: " << static_cast<void*>(bodyframe1_ptr) << "\n";
    //std::cout << "Link::Body1 | ChBodyFrame*(ChBody) dynamic_cast<void*>: " << dynamic_cast<void*>(bodyframe1_ptr) << "\n";

    //ChVariables& (ChBodyFrame::*Variables_bodyframeptr) () = &ChBodyFrame::Variables;
    //ChVariables& (ChBody::*Variables_bodyptr) () = &ChBody::Variables;
    //void (ChBody::*StreamINstate_ptr) (ChStreamInBinary&) = &ChBody::StreamINstate;
    //ChVector<>& (ChBody::*GetPos_bodyptr) () = &ChBody::GetPos;
    //void (ChBody::*SetNoSpeedNoAcceleration_bodyptr) () = &ChBody::SetNoSpeedNoAcceleration;
    //void (ChBody::*ArchiveIN_bodyptr) (ChArchiveIn&) = &ChBody::ArchiveIN;
    //std::cout << "Variables_bodyptr: " << Variables_bodyptr << "\n";
    //std::cout << "StreamINstate_ptr: " << StreamINstate_ptr << "\n";
    //std::cout << "GetPos_bodyptr: " << GetPos_bodyptr << "\n";
    //std::cout << "&ChBody::SetNoSpeedNoAcceleration: " << SetNoSpeedNoAcceleration_bodyptr << "\n";
    //std::cout << "&ChBody::ArchiveIN: " << ArchiveIN_bodyptr << "\n";
    
    //moving_body_ptr.get()->Variables();
    //bodyframe1_ptr->Variables();

    //ChClassFactory* global_factory = GetGlobalClassFactory();

    //system.DoFullAssembly();

    
    // Simulation loop
    while (system.GetChTime() < 2) {
        system.DoStepDynamics(0.01);
    }

    GetLog() << "After archive: GetPos: " << system.Get_bodylist()[1]->GetPos() << "\n";


    auto linkO = std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(system.SearchLink("linkO"));
    std::cout << "function displacement: " << linkO->GetMotorFunction()->Get_y(2) << std::endl;

    return 0;
}
