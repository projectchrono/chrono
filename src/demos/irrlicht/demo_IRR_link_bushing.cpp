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
// Authors: Antonio Recuero, Alessandro Tasora
// =============================================================================
//
// Demonstration of flexible bushing between two bodies
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkBushing.h"
#include "chrono/physics/ChLinkForce.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadContainer.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system
    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the ground body
    auto ground = std::make_shared<ChBodyEasyBox>(0.6, 0.6, 0.15, 10000, false, true);
    system.Add(ground);
    ground->SetBodyFixed(true);
    


    // Create a moving body that will 'bounce' thanks to a flexible bushing.
    // Give it an initial angular velocity and attach also a small sphere 
    // to show the anchoring of the bushing.
    auto body = std::make_shared<ChBodyEasyBox>(0.9, 0.9, 0.15, 1000, false, true);
    system.Add(body);
    body->SetBodyFixed(false);
    body->SetPos(ChVector<>(1.0, 0.0, 0.0));
    body->SetWvel_loc(ChVector<>(1.5, 1.5, -1.5));
    body->SetPos_dt(ChVector<>(1.0, -0.4, 0.2));

    auto symbol_bushing = std::make_shared<ChSphereShape>();
    symbol_bushing->GetSphereGeometry().center = ChVector<>(-1,0,0);
    symbol_bushing->GetSphereGeometry().rad =0.1;
    body->AddAsset(symbol_bushing);

    auto body_col = std::make_shared<ChColorAsset>();
    body_col->SetColor(ChColor(0.6f, 0, 0));
    body->AddAsset(body_col);


    // Now create the bushing. It will connect "ground" and "body".
    // A bushing is like an invisible connection between two bodies; 
    // but differently from constraints, it has some compliance.
    // In the following you will see different examples: just uncomment
    // the my_loadcontainer->Add(..); line to use the associated bushing.

    // Bushings are inherited from ChLoad, so they require a 'load container'

    auto my_loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(my_loadcontainer);


    // EXAMPLE 1: use  ChLoadBodyBodyBushingGeneric
    //
    // This type of bushing requires two 6x6 matrices for generic stiffness and
    // damping, for both translation and rotation. 
    // Optionally, it also supports initial pre-displacement and pre-stress

    ChMatrixNM<double, 6, 6> K_matrix;
    ChMatrixNM<double, 6, 6> R_matrix;

    for (unsigned int ii = 0; ii < 6; ii++) {
        K_matrix(ii, ii) = 95000.0;
        R_matrix(ii, ii) = 1000.0;
    }

    auto my_loadbushingg = std::make_shared<ChLoadBodyBodyBushingGeneric>(
                                    body, // body A
                                    ground, // body B
                                    ChFrame<>(ChVector<>(0.5, 0.0, 0.0)), //initial frame of bushing in abs space
                                    K_matrix,  // the 6x6 (translation+rotation) K matrix in local frame
                                    R_matrix   // the 6x6 (translation+rotation) R matrix in local frame 
                                    ); 
     my_loadbushingg->SetNeutralForce(ChVector<>(100,0,0));
     my_loadbushingg->NeutralDisplacement().SetPos(ChVector<>(0.02,0,0));
     my_loadcontainer->Add(my_loadbushingg);


    // EXAMPLE 2: use  ChLoadBodyBodyBushingMate
    //
    // This type of bushing is like a simplified version of 
    // ChLoadBodyBodyBushingGeneric, it adds compliance to both translation 
    // and rotation, using three x y z and three Rx Ry Rz stiffness values.

    auto my_loadbushing = std::make_shared<ChLoadBodyBodyBushingMate>(
                                    body, // body A
                                    ground, // body B
                                    ChFrame<>(ChVector<>(0.5, 0.0, 0.0)), //initial frame of bushing in abs space
                                    ChVector<>(95000.0),    // K stiffness in local frame  [N/m]
                                    ChVector<>(100.0),      // R damping in local frame  [N/m/s]
                                    ChVector<>(95000.0),    // K rotational stiffness,in local frame [Nm/rad]
                                    ChVector<>(100.0)       // R rotational damping, in local frame [Nm/rad/s]
                                    ); 
    // my_loadcontainer->Add(my_loadbushing);


    // EXAMPLE 3: use  ChLoadBodyBodyBushingPlastic
    //
    // A special type of ChLoadBodyBodyBushingSpherical 
    // that also provides plastic deformation with a plastic yeld. 

     auto my_loadbushingp = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    body, // body A
                                    ground, // body B
                                    ChFrame<>(ChVector<>(0.5, 0.0, 0.0)), //initial frame of bushing in abs space
                                    ChVector<>(95000.0),    // K stiffness in local frame  [N/m]
                                    ChVector<>(100.0),      // R damping in local frame  [N/m/s]
                                    ChVector<>(18000.0)     // plastic yield [N/m]
                                    );  
    //my_loadcontainer->Add(my_loadbushingp);


    // EXAMPLE 4: use  ChLinkBushing
    //
    // Note, the ChLinkBushing is inherited from the ChLink classes. Differently from the
    // previous example, it dies NOT support stiffness matrices, so it should NOT be used
    // for very stiff problems. 
    // This behaves in various ways according to the types in its enums:
    // ChLinkBushing::Spherical: Rotational dofs are free, translational defined by stiffness/damping matrices
    // ChLinkBushing::Revolute: One rotational dof is free, rest of dofs defined by stiffness/damping matrices
    // ChLinkBushing::Mount: All six dofs defined by stiffness/damping matrices

    auto my_linkbushing = std::make_shared<ChLinkBushing>(ChLinkBushing::Mount);
    my_linkbushing->Initialize( body,   // body A 
                                ground, // body B 
                                ChCoordsys<>(ChVector<>(0.5, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0)), //initial frame of bushing in abs space    
                                K_matrix,  // the 6x6 (translation+rotation) K matrix in local frame
                                R_matrix); // the 6x6 (translation+rotation) R matrix in local frame
    //system.Add(my_linkbushing);


    // Finally, add a force that tends to bend the body:

    auto my_loadforce = std::make_shared<ChLoadBodyForce>(
                                    body, 
                                    ChVector<>(0,0, -8000), 
                                    false, 
                                    ChVector<>(1,0,0));
    my_loadcontainer->Add(my_loadforce);


    // Create the Irrlicht application
    ChIrrApp application(&system, L"ChLinkBushing", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(3, 0, 3));

    application.AssetBindAll();
    application.AssetUpdateAll();

    application.SetPlotCOGFrames(true);

    // Change some solver settings;
    application.SetTimestep(0.001);
    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED); 
    //system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN); // does NOT support stiffness matrices, so explicit integration holds for stiff bushings 
    system.SetSolverType(ChSolver::Type::MINRES); // supports stiffness matrices, so implicit integration holds for stiff bushings 
    system.SetMaxItersSolverSpeed(200);
    //system.SetTolForce(1e-12);
    
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
