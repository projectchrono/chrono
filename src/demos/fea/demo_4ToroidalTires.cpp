// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero
// =============================================================================
//
// This demo creates a toroidal geometry that may be used for simple
// tire/ground interaction (user-customizable). The tire is made up of
// ANCF shell elements and may be fully parameterized.
// Both radii of the toroidal geometry and the number of
// ANCF shell elements can be parameterized at the beginning of this file. Boolean
// variables are defined to select the addition of bodies and constraints to the system.
// Position and direction constraints attach the tire tread to the rim. Initial velocity
// and/or forces are used to move the tire forward, which remains in a perpendicular
// plane through a Plane-Plane constraint (ChLinkLockPlanePlane)
// This demo shows two ways of adding pressure to shell elements using ChLoaderPressure
// and ChLoaderUVdistributed. The latter allows for complex, user defined expressions, whereas
// the former imposes constant pressure using normal vectors defined in the element.
// =============================================================================
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"
#include "chrono/core/ChMathematics.h"
#include "chrono_mkl/ChLcpMklSolver.h"
#include "physics/ChLoadContainer.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include <irrlicht.h>
// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;
using namespace chrono::irrlicht;
using namespace irr;
using namespace scene;

bool addConstRim = true;  //
bool addBodies = true;
bool addGroundForces = true;
bool showVisual = true;
bool addSingleLoad = false;
bool addPressureAlessandro = true;
bool addPressure = false;
ChSharedPtr<ChBody> BGround;
ChSharedPtr<ChBody> Hub_1;        // Hub 1
ChSharedPtr<ChBody> Hub_2;        // Hub 2
ChSharedPtr<ChBody> Hub_3;        // Hub 3
ChSharedPtr<ChBody> Hub_4;        // Hub 4
ChSharedPtr<ChBody> SimpChassis;  // Chassis body

ChSharedPtr<ChLinkPointFrame> NodePosRim;
ChSharedPtr<ChLinkDirFrame> NodeDirRim;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode;
ChSharedPtr<ChLinkPointFrame> NodePosRim2;
ChSharedPtr<ChLinkDirFrame> NodeDirRim2;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode2;

// Some model parameters
const double spring_coef = 3e4;  // Springs and dampers for strut
const double damping_coef = 1e3;

const int num_steps = 1500;     // Number of time steps for unit test (range 1 to 4000) 550
double time_step = 0.0005;      //
const double ForVel = 0.0;      // Initial velocity of the tire. Applied to rim and nodes
double plate_lenght_z = 0.007;  // Thickness of the tire

// Mass of the rigid body (dumb brick)
const double RBMass = 2000;
// Location of the wheels w.r.t. rigid body's center of mass
const double Lwz = 1.1;  // Value of longitudinal distance
const double Lwx = 0.7;  // Value of lateral distance
const int NoTires = 4;   // Number of tires. Programmer
// Specification of the mesh
const int numEl_Diameter = 50;  // Number of elements along diameter
const int numEl_Thread = 10;    // Number of elements along thread
const int numEl_z = 1;          // Single element along the thickness
const int N_Diameter = numEl_Diameter;
const int N_Thread = numEl_Thread + 1;
const int N_z = numEl_z + 1;
const double TorusSmallRadius = 0.10;
const double TorusRadius = 0.25;
const double Clearance = -0.0000;  // Initial space between tire and ground
const double GroundLoc =
    -(TorusRadius + TorusSmallRadius + Clearance);          // Vertical position of the ground (for contact)
int TotalNumElements = numEl_Diameter * numEl_Thread;       // Number of elements in the tire
int TotalNumNodes = (numEl_Diameter) * (numEl_Thread + 1);  // Total number of nodes in the tire

double dz = plate_lenght_z / numEl_z;  // We are going to calculate distances through coordinates
double dx = 2 * CH_C_PI * (TorusRadius + TorusSmallRadius) / 2 / N_Diameter;  // Rough estimate of shell dimensions
double dy = CH_C_PI * TorusSmallRadius / numEl_Thread;

// ChLoadCustomMultiple to include basic node-Ground contact interaction
class MyLoadCustomMultiple : public ChLoadCustomMultiple {
  public:
    MyLoadCustomMultiple(std::vector<ChSharedPtr<ChLoadable>>& mloadables) : ChLoadCustomMultiple(mloadables){};
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {
        std::vector<ChSharedPtr<ChLoadable>> NodeList;
        ChVector<> Node1_Pos;
        ChVector<> Node1_Vel;
        ChVector<> Node1_Grad;
        ChVector<> Node1_GradVel;
        // this->load_Q.FillElem(0);
        double KGround = 8e5;
        double CGround = 0.001 * KGround;
        double NormalForceNode = 0;
        double FrictionCoeff = 0.7;

        // Calculation of number of nodes in contact with the Ground
        int NoCNodes = 0;
        for (int iii = 0; iii < 4 * TotalNumNodes; iii++) {
            Node1_Pos = state_x->ClipVector(iii * 6, 0);

            if (Node1_Pos.y < GroundLoc) {
                NoCNodes++;
            }
        }
        if (NoCNodes > 0) {
            KGround = 8e5 / double(NoCNodes);
            CGround = 0.001 * KGround;
        }
        chrono::GetLog() << "  \n"
                         << "Nodes into contact:   " << NoCNodes << " \n";
        if (state_x && state_w) {
            for (int iii = 0; iii < loadables.size(); iii++) {
                Node1_Pos = state_x->ClipVector(iii * 6, 0);
                Node1_Grad = state_x->ClipVector(iii * 6 + 3, 0);
                Node1_Vel = state_w->ClipVector(iii * 6, 0);
                Node1_GradVel = state_w->ClipVector(iii * 6 + 3, 0);
                if (Node1_Pos.y < GroundLoc) {
                    double Penet = abs(Node1_Pos.y - GroundLoc);
                    GetLog() << "Node number:  " << iii << ".  "
                             << "Penetration:  " << Penet << "\n";
                    NormalForceNode = KGround * Penet;            // +CGround * abs(Node1_Vel.y*Penet);
                    this->load_Q(iii * 6 + 1) = NormalForceNode;  // Fy (Vertical)
                    // Friction forces
                    const double VelLimit = 0.1;
                    if (abs(Node1_Vel.x) > VelLimit) {
                        this->load_Q(iii * 6 + 0) =
                            -NormalForceNode * FrictionCoeff *
                            (Node1_Vel.x / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.z, 2))));  // Fx (Plane x)
                    } else {
                        this->load_Q(iii * 6 + 0) =
                            -NormalForceNode * FrictionCoeff * sin(abs(Node1_Vel.x) * CH_C_PI_2 / VelLimit) *
                            (Node1_Vel.x / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.z, 2))));  // Fx (Plane x)
                    }
                    if (abs(Node1_Vel.z) > VelLimit) {
                        this->load_Q(iii * 6 + 2) =
                            -NormalForceNode * FrictionCoeff *
                            (Node1_Vel.z / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.z, 2))));  // Fz (Plane y)
                    } else {
                        this->load_Q(iii * 6 + 2) =
                            -NormalForceNode * FrictionCoeff * sin(abs(Node1_Vel.z) * CH_C_PI_2 / VelLimit) *
                            (Node1_Vel.z / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.z, 2))));  // Fz (Plane y)
                    }
                }
            }
        } else {
            // explicit integrators might call ComputeQ(0,0), null pointers mean
            // that we assume current state, without passing state_x for efficiency
            GetLog() << "\n This should never happen \n";
        }
    }

    // OPTIONAL: if you want to provide an analytical jacobian, just implement the
    // following:
    //   virtual void ComputeJacobian(...)

    // Remember to set this as stiff, to enable the jacobians
    virtual bool IsStiff() { return false; }
};

int main(int argc, char* argv[]) {
    // Definition of the model
    ChSystem my_system;
    std::vector<ChSharedPtr<ChMesh>> TireMeshes(4);

    ChIrrApp application(&my_system, L"ANCF Rolling Tire", core::dimension2d<u32>(1080, 800), false);
    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.5f, 0.5f, 1.15f),   // camera location
                                 core::vector3df(0.65f, 0.0f, 0.0f));  // "look at" location
    application.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 160,
                                 70);
    utils::CSV_writer out("\t");
    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(7);
    // Main loop for the definition of 4 meshes

    for (int TireNo = 0; TireNo < NoTires; TireNo++) {
        TireMeshes[TireNo] = ChSharedPtr<ChMesh>(new ChMesh);
        //  Fixing constraints, initial coordinates and velocities
        double thethaTorus = 0.0;
        double phiTorus = 0.0;
        double loc_x = 0.0;
        double loc_y = 0.0;
        double loc_z = 0.0;

        for (int j = 0; j < N_Diameter; j++) {
            for (int i = 0; i < N_Thread; i++) {
                thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
                thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
                phiTorus = 2 * CH_C_PI * j / N_Diameter;
                switch (TireNo) {
                    case 0:
                        loc_x = TorusSmallRadius * sin(thethaTorus) - Lwx;
                        loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
                        loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus) + Lwz;
                        break;
                    case 1:
                        loc_x = TorusSmallRadius * sin(thethaTorus) + Lwx;
                        loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
                        loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus) + Lwz;
                        break;
                    case 2:
                        loc_x = TorusSmallRadius * sin(thethaTorus) + Lwx;
                        loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
                        loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus) - Lwz;
                        break;
                    case 3:
                        loc_x = TorusSmallRadius * sin(thethaTorus) - Lwx;
                        loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
                        loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus) - Lwz;
                        break;
                }

                double dir_x = sin(thethaTorus);
                double dir_y = cos(thethaTorus) * sin(phiTorus);
                double dir_z = cos(thethaTorus) * cos(phiTorus);

                // Write position and gradient of nodes to a file and plot them in Matlab
                out << i + j * N_Thread << loc_x << loc_y << loc_z << dir_x << dir_y << dir_z << std::endl;
                out.write_to_file("../TorusTire.txt");

                double vel_x = 0;
                double vel_y = 0;
                double vel_z = ForVel;

                // Create the node
                ChSharedPtr<ChNodeFEAxyzD> node(
                    new ChNodeFEAxyzD(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z)));
                // Apply initial velocities
                // node.DynamicCastTo<ChNodeFEAxyz>()->SetPos_dt(ChVector<>(vel_x, vel_y, vel_z));
                node->SetMass(0);
                // Add node to mesh
                TireMeshes[TireNo]->AddNode(node);
            }
        }
        // Create an isotropic material.
        // All elements share the same material.
        ChSharedPtr<ChMaterialShellANCF> mat(new ChMaterialShellANCF(500, 9.0e7, 0.3));
        // Create the elements
        int node0, node1, node2, node3;         // Node numbering
        for (int j = 0; j < N_Diameter; j++) {  // Start node numbering by zero
            for (int i = 0; i < numEl_Thread; i++) {
                if (j == N_Diameter - 1) {
                    node0 = i + j * (numEl_Thread + 1);
                    node1 = i;
                    node2 = i + 1;
                    node3 = i + 1 + j * (numEl_Thread + 1);
                } else {
                    node0 = i + j * (numEl_Thread + 1);
                    node1 = i + (j + 1) * (numEl_Thread + 1);
                    node2 = i + 1 + (j + 1) * (numEl_Thread + 1);
                    node3 = i + 1 + j * (numEl_Thread + 1);
                }
                // Create the element and set its nodes.
                ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
                element->SetNodes(TireMeshes[TireNo]->GetNode(node2).DynamicCastTo<ChNodeFEAxyzD>(),
                                  TireMeshes[TireNo]->GetNode(node1).DynamicCastTo<ChNodeFEAxyzD>(),
                                  TireMeshes[TireNo]->GetNode(node3).DynamicCastTo<ChNodeFEAxyzD>(),
                                  TireMeshes[TireNo]->GetNode(node0).DynamicCastTo<ChNodeFEAxyzD>());
                // Set element dimensions
                element->SetDimensions(dx, dy);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(dz, 0.0 * CH_C_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(0.02);  // Structural damping for this element
                element->SetGravityOn(true);  // gravitational forces

                // Add element to mesh
                TireMeshes[TireNo]->AddElement(element);
            }
        }  // Here the creation of elements ends
        // Switch off mesh class gravity
        TireMeshes[TireNo]->SetAutomaticGravity(false);

        // Add tmy_system.Addhe mesh to the system
        my_system.Add(TireMeshes[TireNo]);

        if (addSingleLoad) {
            ChSharedPtr<ChNodeFEAxyzD> nodetip(
                TireMeshes[TireNo]->GetNode(TotalNumNodes - 1).DynamicCastTo<ChNodeFEAxyzD>());
            nodetip->SetForce(ChVector<>(0, 0, -10));
        }

    }  // Here we finish loop over all tires
    ChSharedPtr<ChNodeFEAxyzD> nodetip(TireMeshes[0]->GetNode(0).DynamicCastTo<ChNodeFEAxyzD>());
    if (addBodies) {
        // Body 1: Ground
        BGround = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(BGround);
        BGround->SetIdentifier(1);
        BGround->SetBodyFixed(true);
        BGround->SetCollide(false);
        BGround->SetMass(1);
        BGround->SetInertiaXX(ChVector<>(1, 1, 0.2));
        BGround->SetPos(ChVector<>(-2, 0, 0));  // Y = -1m
        ChQuaternion<> rot = Q_from_AngX(0.0);
        BGround->SetRot(rot);
        // Defining the Body 2: Rim for TireMeshes[0]
        Hub_1 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Hub_1);
        Hub_1->SetIdentifier(2);
        Hub_1->SetBodyFixed(false);
        Hub_1->SetCollide(false);
        Hub_1->SetMass(10);
        Hub_1->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
        Hub_1->SetPos(ChVector<>(-Lwx, 0, Lwz));  // Y = -1m
        // Defining the Body 3: Rim for TireMeshes[1]
        Hub_2 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Hub_2);
        Hub_2->SetIdentifier(3);
        Hub_2->SetBodyFixed(false);
        Hub_2->SetCollide(false);
        Hub_2->SetMass(10);
        Hub_2->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
        Hub_2->SetPos(ChVector<>(Lwx, 0, Lwz));  // Y = -1m
        // Defining the Body 4: Rim for TireMeshes[2]
        Hub_3 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Hub_3);
        Hub_3->SetIdentifier(4);
        Hub_3->SetBodyFixed(false);
        Hub_3->SetCollide(false);
        Hub_3->SetMass(10);
        Hub_3->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
        Hub_3->SetPos(ChVector<>(Lwx, 0, -Lwz));  // Y = -1m
        // Defining the Body 5: Rim for TireMeshes[3]
        Hub_4 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Hub_4);
        Hub_4->SetIdentifier(5);
        Hub_4->SetBodyFixed(false);
        Hub_4->SetCollide(false);
        Hub_4->SetMass(10);
        Hub_4->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
        Hub_4->SetPos(ChVector<>(-Lwx, 0, -Lwz));  // Y = -1m

        // Struts

        // Defining the Body 2: Rim for TireMeshes[0]
        /*St_2 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(St_2);
        St_2->SetIdentifier(7);
        St_2->SetBodyFixed(false);
        St_2->SetCollide(false);
        St_2->SetMass(10);
        St_2->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
        St_2->SetPos(ChVector<>(-Lwx, 0, Lwz));  // Y = -1m
        // Defining the Body 3: Rim for TireMeshes[1]
        St_3 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(St_3);
        St_3->SetIdentifier(8);
        St_3->SetBodyFixed(false);
        St_3->SetCollide(false);
        St_3->SetMass(10);
        St_3->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
        St_3->SetPos(ChVector<>(Lwx, 0, Lwz));  // Y = -1m
        // Defining the Body 4: Rim for TireMeshes[2]
        St_4 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(St_4);
        St_4->SetIdentifier(9);
        St_4->SetBodyFixed(false);
        St_4->SetCollide(false);
        St_4->SetMass(10);
        St_4->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
        St_4->SetPos(ChVector<>(Lwx, 0, -Lwz));  // Y = -1m
        // Defining the Body 5: Rim for TireMeshes[3]
        St_5 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(St_5);
        St_5->SetIdentifier(10);
        St_5->SetBodyFixed(false);
        St_5->SetCollide(false);
        St_5->SetMass(10);
        St_5->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
        St_5->SetPos(ChVector<>(-Lwx, 0, -Lwz));  // Y = -1m*/
        // Defining the SimpChassis
        // Defining the Body 5: Rim for TireMeshes[3]
        /*ChSharedPtr<ChBodyEasyBox> SimpChassis(new ChBodyEasyBox(0.92, 0.4, 2.5,  // x,y,z size
                                                                 100,            // density
                                                                 false,           // collide enable?
                                                                 true));          // visualization?*/
        SimpChassis = ChSharedPtr<ChBody>(new ChBody);
        SimpChassis->SetPos(ChVector<>(0, 0, 0));
        my_system.AddBody(SimpChassis);
        // optional, attach a texture for better visualization
        ChSharedPtr<ChTexture> mtexturebox(new ChTexture());
        mtexturebox->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));
        SimpChassis->AddAsset(mtexturebox);
        SimpChassis->SetMass(100);
        SimpChassis->SetInertiaXX(ChVector<>(10, 10, 10));
        SimpChassis->SetBodyFixed(false);

        /*SimpChassis = ChSharedPtr<ChBody>(new ChBody); // No visualization
        my_system.AddBody(SimpChassis);
        SimpChassis->SetIdentifier(5);
        SimpChassis->SetBodyFixed(false);
        SimpChassis->SetCollide(false);
        SimpChassis->SetMass(RBMass);
        SimpChassis->SetInertiaXX(ChVector<>(900, 900, 1500));
        SimpChassis->SetPos(ChVector<>(0, 0, 0));  // Y = -1m
        SimpChassis->SetRot(rot);*/

        // Revolute joint between hubs and struts
        /* ChSharedPtr<ChLinkLockRevolute> Rev1(new ChLinkLockRevolute);
         Rev1->Initialize(Hub_1, SimpChassis, ChCoordsys<>(ChVector<>(-Lwx, 0, Lwz), Q_from_AngY(-CH_C_PI_2)));
         my_system.AddLink(Rev1);
         ChSharedPtr<ChLinkLockRevolute> Rev2(new ChLinkLockRevolute);
         Rev2->Initialize(Hub_2, St_3, ChCoordsys<>(ChVector<>(Lwx, 0, Lwz), Q_from_AngY(-CH_C_PI_2)));
         my_system.AddLink(Rev2);
         ChSharedPtr<ChLinkLockRevolute> Rev3(new ChLinkLockRevolute);
         Rev3->Initialize(Hub_3, St_4, ChCoordsys<>(ChVector<>(Lwx, 0, -Lwz), Q_from_AngY(-CH_C_PI_2)));
         my_system.AddLink(Rev3);
         ChSharedPtr<ChLinkLockRevolute> Rev4(new ChLinkLockRevolute);
         Rev4->Initialize(Hub_4, St_5, ChCoordsys<>(ChVector<>(-Lwx, 0, -Lwz), Q_from_AngY(-CH_C_PI_2)));
         my_system.AddLink(Rev4);*/

        ChSharedPtr<ChLinkLockPrismatic> prims_1(new ChLinkLockPrismatic);
        my_system.AddLink(prims_1);
        prims_1->Initialize(Hub_1, SimpChassis, ChCoordsys<>(ChVector<>(-Lwx, 0, Lwz), Q_from_AngX(-CH_C_PI_2)));

        ChSharedPtr<ChLinkLockPrismatic> prims_2(new ChLinkLockPrismatic);
        my_system.AddLink(prims_2);
        prims_2->Initialize(Hub_2, SimpChassis, ChCoordsys<>(ChVector<>(Lwx, 0, Lwz), Q_from_AngX(-CH_C_PI_2)));

        ChSharedPtr<ChLinkLockPrismatic> prims_3(new ChLinkLockPrismatic);
        my_system.AddLink(prims_3);
        prims_3->Initialize(Hub_3, SimpChassis, ChCoordsys<>(ChVector<>(Lwx, 0, -Lwz), Q_from_AngX(-CH_C_PI_2)));

        ChSharedPtr<ChLinkLockPrismatic> prims_4(new ChLinkLockPrismatic);
        my_system.AddLink(prims_4);
        prims_4->Initialize(Hub_4, SimpChassis, ChCoordsys<>(ChVector<>(-Lwx, 0, -Lwz), Q_from_AngX(-CH_C_PI_2)));

        // Spring and damper for secondary suspension: True position vectors are relative
        ChSharedPtr<ChLinkSpring> spring1 = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
        spring1->Initialize(Hub_1, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(-Lwx, 0, Lwz), true);
        spring1->Set_SpringK(spring_coef);
        spring1->Set_SpringR(damping_coef);
        my_system.AddLink(spring1);

        ChSharedPtr<ChLinkSpring> spring2 = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
        spring2->Initialize(Hub_2, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(Lwx, 0, Lwz), true);
        spring2->Set_SpringK(spring_coef);
        spring2->Set_SpringR(damping_coef);
        my_system.AddLink(spring2);

        ChSharedPtr<ChLinkSpring> spring3 = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
        spring3->Initialize(Hub_3, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(Lwx, 0, -Lwz), true);
        spring3->Set_SpringK(spring_coef);
        spring3->Set_SpringR(damping_coef);
        my_system.AddLink(spring3);

        ChSharedPtr<ChLinkSpring> spring4 = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
        spring4->Initialize(Hub_4, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(-Lwx, 0, -Lwz), true);
        spring4->Set_SpringK(spring_coef);
        spring4->Set_SpringR(damping_coef);
        my_system.AddLink(spring4);

        my_system.Set_G_acc(ChVector<>(0, -9.81, 0));
    }

    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    // std::vector<ChSharedPtr<ChLoadContainer> > Mloadcontainer4(4);
    ChSharedPtr<ChLoadContainer> Mloadcontainer(new ChLoadContainer);
    ChSharedPtr<ChLoadContainer> MloadcontainerGround(new ChLoadContainer);
    // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
    if (addPressureAlessandro) {
        for (int TireNo = 0; TireNo < NoTires; TireNo++) {
            for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
                ChSharedPtr<ChLoad<ChLoaderPressure>> faceload(new ChLoad<ChLoaderPressure>(
                    TireMeshes[TireNo]->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
                faceload->loader.SetPressure(-120e3);
                faceload->loader.SetStiff(false);
                faceload->loader.SetIntegrationPoints(2);
                Mloadcontainer->Add(faceload);
            }
        }
    }  // End of defining load containers for all pressure loads

    // can depend arbitrarily on the coordinates
    if (addPressure) {
        class MyPressureLoad : public ChLoaderUVdistributed {
          private:
            ChSharedPtr<ChElementShellANCF> m_element;

          public:
            // Useful: a constructor that also sets ChLoadable
            MyPressureLoad(ChSharedPtr<ChLoadableUV> element) : ChLoaderUVdistributed(element) {
                m_element = element.StaticCastTo<ChElementShellANCF>();
            };
            virtual bool IsStiff() override { return true; }
            virtual int GetIntegrationPointsU() { return 2; }
            virtual int GetIntegrationPointsV() { return 2; }
            // Compute F=F(u)
            // This is the function that you have to implement. It should return the
            // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
            // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
            ChVector<> FPressure;
            virtual void ComputeF(
                const double U,
                const double V,              ///< parametric coordinate in line
                ChVectorDynamic<>& F,        ///< Result F vector here, size must be = n.field coords.of loadable
                ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                ) {
                ChVector<> Position1;
                ChVector<> Gradient1;
                ChVector<> Position2;
                ChVector<> Gradient2;
                ChVector<> Position3;
                ChVector<> Gradient3;
                ChVector<> Position4;
                ChVector<> Gradient4;
                double PressureVal = 220e3;  // Pressure

                if (state_x && state_w) {
                    Position1 = state_x->ClipVector(0, 0);
                    Gradient1 = state_x->ClipVector(3, 0);
                    Position2 = state_x->ClipVector(6, 0);
                    Gradient2 = state_x->ClipVector(9, 0);
                    Position3 = state_x->ClipVector(12, 0);
                    Gradient3 = state_x->ClipVector(15, 0);
                    Position4 = state_x->ClipVector(18, 0);
                    Gradient4 = state_x->ClipVector(21, 0);

                    ChMatrixNM<double, 1, 8> Nx;
                    ChMatrixNM<double, 1, 8> N;
                    ChMatrixNM<double, 1, 8> Ny;
                    ChMatrixNM<double, 1, 8> Nz;
                    ChMatrixNM<double, 3, 8> d;
                    (d).PasteVector(Position1, 0, 0);
                    (d).PasteVector(Gradient1, 0, 1);
                    (d).PasteVector(Position2, 0, 2);
                    (d).PasteVector(Gradient2, 0, 3);
                    (d).PasteVector(Position3, 0, 4);
                    (d).PasteVector(Gradient3, 0, 5);
                    (d).PasteVector(Position4, 0, 6);
                    (d).PasteVector(Gradient4, 0, 7);
                    m_element->ShapeFunctions(N, U, V, 0);
                    m_element->ShapeFunctionsDerivativeX(Nx, U, V, 0);
                    m_element->ShapeFunctionsDerivativeY(Ny, U, V, 0);
                    m_element->ShapeFunctionsDerivativeZ(Nz, U, V, 0);

                    ChMatrixNM<double, 1, 3> Nx_d;
                    Nx_d.MatrMultiplyT(Nx, d);

                    ChMatrixNM<double, 1, 3> Ny_d;
                    Ny_d.MatrMultiplyT(Ny, d);

                    ChMatrixNM<double, 1, 3> Nz_d;
                    Nz_d.MatrMultiplyT(Nz, d);

                    ChMatrixNM<double, 3, 3> rd;
                    rd(0, 0) = Nx_d(0, 0);
                    rd(1, 0) = Nx_d(0, 1);
                    rd(2, 0) = Nx_d(0, 2);
                    rd(0, 1) = Ny_d(0, 0);
                    rd(1, 1) = Ny_d(0, 1);
                    rd(2, 1) = Ny_d(0, 2);
                    rd(0, 2) = Nz_d(0, 0);
                    rd(1, 2) = Nz_d(0, 1);
                    rd(2, 2) = Nz_d(0, 2);

                    ChVector<> G1xG2;
                    G1xG2(0) = rd(1, 0) * rd(2, 1) - rd(2, 0) * rd(1, 1);
                    G1xG2(1) = rd(2, 0) * rd(0, 1) - rd(0, 0) * rd(2, 1);
                    G1xG2(2) = rd(0, 0) * rd(1, 1) - rd(1, 0) * rd(0, 1);
                    G1xG2.Normalize();
                    FPressure = G1xG2 * PressureVal;
                }
                F.PasteVector(FPressure, 0, 0);
            }
        };

        // Create the load (and handle it with a shared pointer).
        // The ChLoad is a 'container' for your ChLoader.
        // It is created using templates, that is instancing a ChLoad<a_loader_class>()
        // initiate for loop for all the elements
        for (int TireNo = 0; TireNo < NoTires; TireNo++) {
            for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
                ChSharedPtr<ChLoad<MyPressureLoad>> PressureElement(new ChLoad<MyPressureLoad>(
                    TireMeshes[TireNo]->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
                Mloadcontainer->Add(PressureElement);  // do not forget to add the load to the load container.
            }
        }
    }
    // Add constraints to the rim: ChLinkPointFrame for position constraints,
    // ChLinkDirFrame for direction constraints
    if (addConstRim) {
        for (int TireNo = 0; TireNo < NoTires; TireNo++) {
            int NoConstNodes = 2 * numEl_Diameter;
            int ConstNodeInx = -1;

            for (int j = 0; j < N_Diameter; j++) {
                for (int i = 0; i < N_Thread; i++) {
                    ConstNodeInx = i + j * N_Thread;  // General node index
                    if ((i + j * N_Thread) % (N_Thread) == 0) {
                        // First node to be constrained (one side of the rim)

                        if (TireNo == 0) {
                            ConstrainedNode = ChSharedPtr<ChNodeFEAxyzD>(
                                TireMeshes[TireNo]->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                            NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim->Initialize(ConstrainedNode, Hub_1);
                            my_system.Add(NodePosRim);
                            GetLog() << "ConstrainedNode Tire 1: " << ConstNodeInx << "\n";

                            NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim->Initialize(ConstrainedNode, Hub_1);
                            NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                            my_system.Add(NodeDirRim);

                            // Second node to be constrained (other side of the rim)
                            ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(TireMeshes[TireNo]
                                                                              ->GetNode(ConstNodeInx + N_Thread - 1)
                                                                              .DynamicCastTo<ChNodeFEAxyzD>());

                            NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim2->Initialize(ConstrainedNode2, Hub_1);
                            my_system.Add(NodePosRim2);
                            // GetLog() << "ConstrainedNode 2: " << ConstNodeInx + N_Thread - 1 << "\n";
                            // system("pause");
                            NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim2->Initialize(ConstrainedNode2, Hub_1);
                            NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                            my_system.Add(NodeDirRim2);
                        }
                        if (TireNo == 1) {
                            ConstrainedNode = ChSharedPtr<ChNodeFEAxyzD>(
                                TireMeshes[TireNo]->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                            NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim->Initialize(ConstrainedNode, Hub_2);
                            my_system.Add(NodePosRim);
                            GetLog() << "ConstrainedNode Tire 1: " << ConstNodeInx << "\n";
                            NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim->Initialize(ConstrainedNode, Hub_2);
                            NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                            my_system.Add(NodeDirRim);

                            // Second node to be constrained (other side of the rim)
                            ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(TireMeshes[TireNo]
                                                                              ->GetNode(ConstNodeInx + N_Thread - 1)
                                                                              .DynamicCastTo<ChNodeFEAxyzD>());

                            NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim2->Initialize(ConstrainedNode2, Hub_2);
                            my_system.Add(NodePosRim2);

                            NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim2->Initialize(ConstrainedNode2, Hub_2);
                            NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                            my_system.Add(NodeDirRim2);
                        }
                        if (TireNo == 2) {
                            ConstrainedNode = ChSharedPtr<ChNodeFEAxyzD>(
                                TireMeshes[TireNo]->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                            NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim->Initialize(ConstrainedNode, Hub_3);
                            my_system.Add(NodePosRim);
                            GetLog() << "ConstrainedNode Tire 1: " << ConstNodeInx << "\n";
                            NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim->Initialize(ConstrainedNode, Hub_3);
                            NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                            my_system.Add(NodeDirRim);

                            // Second node to be constrained (other side of the rim)
                            ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(TireMeshes[TireNo]
                                                                              ->GetNode(ConstNodeInx + N_Thread - 1)
                                                                              .DynamicCastTo<ChNodeFEAxyzD>());

                            NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim2->Initialize(ConstrainedNode2, Hub_3);
                            my_system.Add(NodePosRim2);

                            NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim2->Initialize(ConstrainedNode2, Hub_3);
                            NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                            my_system.Add(NodeDirRim2);
                        }
                        if (TireNo == 3) {
                            ConstrainedNode = ChSharedPtr<ChNodeFEAxyzD>(
                                TireMeshes[TireNo]->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                            NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim->Initialize(ConstrainedNode, Hub_4);
                            my_system.Add(NodePosRim);
                            GetLog() << "ConstrainedNode Tire 1: " << ConstNodeInx << "\n";
                            NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim->Initialize(ConstrainedNode, Hub_4);
                            NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                            my_system.Add(NodeDirRim);

                            // Second node to be constrained (other side of the rim)
                            ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(TireMeshes[TireNo]
                                                                              ->GetNode(ConstNodeInx + N_Thread - 1)
                                                                              .DynamicCastTo<ChNodeFEAxyzD>());

                            NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                            NodePosRim2->Initialize(ConstrainedNode2, Hub_4);
                            my_system.Add(NodePosRim2);

                            NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                            NodeDirRim2->Initialize(ConstrainedNode2, Hub_4);
                            NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                            my_system.Add(NodeDirRim2);
                        }
                    }
                }
            }  // Loop over all 4 tires
        }
    }
    if (addGroundForces) {
        // Select on which nodes we are going to apply a load
        std::vector<ChSharedPtr<ChLoadable>> NodeList1;
        for (int TireNo = 0; TireNo < NoTires; TireNo++) {
            for (int iNode = 0; iNode < TotalNumNodes; iNode++) {
                ChSharedPtr<ChNodeFEAxyzD> NodeLoad1(TireMeshes[TireNo]->GetNode(iNode).DynamicCastTo<ChNodeFEAxyzD>());
                NodeList1.push_back(NodeLoad1);
            }
        }
        // Instance load object. This require a list of ChLoadable objects
        // (these are our two nodes,pay attention to the sequence order), and add to
        // container.
        ChSharedPtr<MyLoadCustomMultiple> Mloadcustommultiple1(new MyLoadCustomMultiple(NodeList1));
        MloadcontainerGround->Add(Mloadcustommultiple1);

    }  // End loop over tires
    my_system.Add(Mloadcontainer);
    my_system.Add(MloadcontainerGround);
    // Mark completion of system construction

    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(false);
    mkl_solver_stab->SetSparsityPatternLock(false);

    // INT_HHT or INT_EULER_IMPLICIT
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    // my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
    mystepper->SetAlpha(-0.3);  // Important for convergence
    mystepper->SetMaxiters(11);
    mystepper->SetTolerance(1e-02);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);  //
    mystepper->SetVerbose(true);
    mystepper->SetRequiredSuccessfulSteps(2);
    mystepper->SetMaxItersSuccess(7);
    ChMatrixNM<double, 3, 1> Cp;
    ChMatrixNM<double, 2, 1> Cd;  // Matrices for storing constraint violations

    // Visualization
    /*ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
    mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    Hub_1->AddAsset(mobjmesh);
    Hub_2->AddAsset(mobjmesh);
    Hub_3->AddAsset(mobjmesh);
    Hub_4->AddAsset(mobjmesh);*/

    double start = std::clock();
    if (showVisual) {
        for (int TireNo = 0; TireNo < NoTires; TireNo++) {  // Add visualization to all meshes
            ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(
                new ChVisualizationFEAmesh(*(TireMeshes[TireNo].get_ptr())));
            mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
            mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
            mvisualizemeshC->SetSymbolsThickness(0.003);
            TireMeshes[TireNo]->AddAsset(mvisualizemeshC);

            ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshwire(
                new ChVisualizationFEAmesh(*(TireMeshes[TireNo].get_ptr())));
            mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
            mvisualizemeshwire->SetWireframe(true);
            TireMeshes[TireNo]->AddAsset(mvisualizemeshwire);

            ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(
                new ChVisualizationFEAmesh(*(TireMeshes[TireNo].get_ptr())));
            mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
            mvisualizemesh->SetColorscaleMinMax(0.0, 30);
            mvisualizemesh->SetSmoothFaces(true);
            TireMeshes[TireNo]->AddAsset(mvisualizemesh);
        }  // End of add visualization to all meshes

        application.AssetBindAll();
        application.AssetUpdateAll();

        video::ITexture* cubeMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("concrete.jpg").c_str());
        video::ITexture* rockMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("rock.jpg").c_str());
        // Create the a plane using body of 'box' type:
        ChBodySceneNode* mrigidBody;
        mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
            &my_system, application.GetSceneManager(), 100.0, ChVector<>(0, GroundLoc, 0), ChQuaternion<>(1, 0, 0, 0),
            ChVector<>(10, 0.000001, 10));
        mrigidBody->GetBody()->SetBodyFixed(true);
        mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0);
        mrigidBody->setMaterialTexture(0, cubeMap);
        mrigidBody->GetBody()->SetCollide(false);
        my_system.SetupInitial();
        my_system.Setup();
        my_system.Update();

        chrono::GetLog()
            << "\n\nREADME\n\n"
            << " - Press SPACE to start dynamic simulation \n - Press F10 for nonlinear statics - Press F11 for "
               "linear statics. \n";

        // at beginning, no analysis is running..
        application.SetPaused(false);
        int AccuNoIterations = 0;
        application.SetStepManage(true);
        application.SetTimestep(time_step);
        application.SetTryRealtime(false);
        double ChTime = 0.0;

        utils::CSV_writer out("\t");
        out.stream().setf(std::ios::scientific | std::ios::showpos);
        out.stream().precision(7);
        const double VerForce = 0;
        const double HorForce = 4000;
        const double tini = 0.026;
        const double tend = 0.2;
        const double interval = tend - tini;
        while (application.GetDevice()->run()) {
            Hub_1->Empty_forces_accumulators();
            Hub_2->Empty_forces_accumulators();
            // Hub_1->Set_Scr_force(const ChVector<>& mf) { Scr_force = mf; }
            if (my_system.GetChTime() >= tini && my_system.GetChTime() <= tend) {
                Hub_1->Set_Scr_torque(ChVector<>(HorForce * (my_system.GetChTime() - tini) / interval, 0, 0));
                Hub_2->Set_Scr_torque(ChVector<>(HorForce * (my_system.GetChTime() - tini) / interval, 0, 0));
            } else if (my_system.GetChTime() > tend) {
                Hub_1->Set_Scr_torque(ChVector<>(HorForce, 0, 0));
                Hub_2->Set_Scr_torque(ChVector<>(HorForce, 0, 0));
            }

            application.BeginScene();
            application.DrawAll();
            application.DoStep();
            application.EndScene();
            if (!application.GetPaused()) {
                std::cout << "Time t = " << my_system.GetChTime() << "s \n";
                // AccuNoIterations += mystepper->GetNumIterations();
                printf(
                    "Vertical position of Tires:      %12.4e       %12.4e       %12.4e       %12.4e  Chassis  %12.4e",
                    Hub_1->GetPos().y, Hub_2->GetPos().y, Hub_3->GetPos().y, Hub_4->GetPos().y,
                    SimpChassis->GetPos().y);
                out << my_system.GetChTime() << Hub_1->GetPos().x << Hub_1->GetPos().y << Hub_1->GetPos().z
                    << Hub_2->GetPos().x << Hub_2->GetPos().y << Hub_2->GetPos().z << Hub_3->GetPos().x
                    << Hub_3->GetPos().y << Hub_3->GetPos().z << std::endl;
                out.write_to_file("../VertPosRim.txt");
            }
        }
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        chrono::GetLog() << "Computation Time: " << duration;
        system("pause");
    } else {
        my_system.Setup();
        my_system.Update();
        my_system.SetupInitial();
        for (unsigned int it = 0; it < num_steps; it++) {
            my_system.DoStepDynamics(time_step);
            std::cout << "Time t = " << my_system.GetChTime() << "s \n";
            // std::cout << "nodetip->pos.z = " << nodetip->pos.z << "\n";
            // std::cout << "mystepper->GetNumIterations()= " << mystepper->GetNumIterations() << "\n";
            if (addConstRim) {
                Cp = NodePosRim->GetC();
                printf("Point constraint violations:      %12.4e  %12.4e  %12.4e\n", Cp.GetElement(0, 0),
                       Cp.GetElement(1, 0), Cp.GetElement(2, 0));
                Cd = NodeDirRim->GetC();
                printf("Direction constraint violations:  %12.4e  %12.4e\n", Cd.GetElement(0, 0), Cd.GetElement(1, 0));

                printf("Vertical position of the rim:  %12.4e m  %12.4e m  %12.4e m  %12.4e m\n", Hub_1->coord.pos.y,
                       Hub_2->coord.pos.y, Hub_3->coord.pos.y, Hub_4->coord.pos.y);
            }
        }
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        chrono::GetLog() << "Computation Time: " << duration;
        system("pause");
    }

    return 0;
}
