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
// This demo creates four wheels objects using the function makeANCFwheel. The 4
// wheels are constrained to the rim, which in turn is linked to the chassis
// through a ChLinkRevoluteTranslational joint. Values for the spring and damping
// coefficients of the secondary suspension may be selected in the parameter
// definition section.
// =============================================================================
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono/assets/ChTexture.h"
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
ChSharedPtr<ChBody> BGround;
ChSharedPtr<ChBody> SimpChassis;  // Chassis body

ChSharedPtr<ChLinkPointFrame> NodePosRim;
ChSharedPtr<ChLinkDirFrame> NodeDirRim;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode;
ChSharedPtr<ChLinkPointFrame> NodePosRim2;
ChSharedPtr<ChLinkDirFrame> NodeDirRim2;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode2;  // Shared pointers for constraining mesh to hub

ChSharedPtr<ChLoadContainer> MloadcontainerGround(new ChLoadContainer);
// Some model parameters
const double spring_coef = 3e4;  // Springs and dampers for strut
const double damping_coef = 1e3;
const int num_steps = 1500;     // Number of time steps for unit test (range 1 to 4000) 550
double time_step = 0.0002;      //
const double ForVelocity = 4;   // Initial velocity of the tire. Applied to hub, nodes, and chassis
double plate_lenght_z = 0.007;  // Thickness of the tire
double TirePressure = 120e3;    // Applied suddently at the beginning of simulation
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
const double Clearance = -0.0003;  // Initial space between tire and ground
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
        double CGround = KGround;
        double NormalForceNode = 0;
        double FrictionCoeff = 0.7;

        // Calculation of number of nodes in contact with the Ground
        int NoCNodes = 0;
        for (int iii = 0; iii < loadables.size(); iii++) {
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
                    // GetLog() << "Node number:  " << iii << ".  "
                    //          << "Penetration:  " << Penet << "\n";
                    NormalForceNode = KGround * Penet;  // +CGround * abs(Node1_Vel.y*Penet);
                    this->load_Q(iii * 6 + 1) =
                        NormalForceNode - CGround * (Node1_Vel.y) * abs(Penet);  // Fy (Vertical)
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
    virtual bool IsStiff() { return false; }
};

void MakeANCFWheel(ChSystem& my_system,
                   const ChVector<> rim_center,
                   ChSharedPtr<ChMaterialShellANCF>& mat,
                   ChSharedPtr<ChBody>& Hub_1,
                   int N_Diameter,
                   int N_Thread,
                   double TorusRadius,
                   double TorusSmallRadius,
                   double TirePressure,
                   double ForVelocity) {
    // Create rim for this mesh
    my_system.AddBody(Hub_1);
    Hub_1->SetIdentifier(2);
    Hub_1->SetBodyFixed(false);
    Hub_1->SetCollide(false);
    Hub_1->SetMass(10);
    Hub_1->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
    Hub_1->SetPos(rim_center);  // Y = -1m
    Hub_1->SetPos_dt(ChVector<>(0, 0, ForVelocity));
    Hub_1->SetWvel_par(ChVector<>(-ForVelocity / (TorusRadius + TorusSmallRadius), 0, 0));

    // Create tire mesh
    ChSharedPtr<ChMesh> TireMesh(new ChMesh);
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
            loc_x = TorusSmallRadius * sin(thethaTorus) + rim_center.x;
            loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
            loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus) + rim_center.z;
            double dir_x = sin(thethaTorus);
            double dir_y = cos(thethaTorus) * sin(phiTorus);
            double dir_z = cos(thethaTorus) * cos(phiTorus);

            double vel_x = 0;
            double vel_y = 0;
            double vel_z = 0;

            // Create the node
            ChSharedPtr<ChNodeFEAxyzD> node(
                new ChNodeFEAxyzD(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z)));
            node->SetMass(0);
            // Add node to mesh
            TireMesh->AddNode(node);
        }
    }

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
            element->SetNodes(TireMesh->GetNode(node2).DynamicCastTo<ChNodeFEAxyzD>(),
                              TireMesh->GetNode(node1).DynamicCastTo<ChNodeFEAxyzD>(),
                              TireMesh->GetNode(node3).DynamicCastTo<ChNodeFEAxyzD>(),
                              TireMesh->GetNode(node0).DynamicCastTo<ChNodeFEAxyzD>());
            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0.0 * CH_C_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(0.02);  // Structural damping for this element
            element->SetGravityOn(true);  // gravitational forces

            // Add element to mesh
            TireMesh->AddElement(element);
        }
    }  // Here the creation of elements ends

    // Add initial velocity to the nodes (for rolling)
    for (unsigned int i = 0; i < TireMesh->GetNnodes(); ++i) {
        ChVector<> node_pos = TireMesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>()->GetPos();
        double tang_vel =
            ForVelocity * (node_pos.y + TorusRadius + TorusSmallRadius) / (TorusRadius + TorusSmallRadius);
        ChVector<> NodeVel(0, 0, tang_vel);
        TireMesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>()->SetPos_dt(NodeVel);
    }

    // Switch off mesh class gravity
    TireMesh->SetAutomaticGravity(false);

    // Add the mesh to the system
    my_system.Add(TireMesh);

    ChSharedPtr<ChLoadContainer> Mloadcontainer(new ChLoadContainer);
    // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
    if (addPressureAlessandro) {
        for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            ChSharedPtr<ChLoad<ChLoaderPressure>> faceload(
                new ChLoad<ChLoaderPressure>(TireMesh->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
            faceload->loader.SetPressure(-TirePressure);
            faceload->loader.SetStiff(false);
            faceload->loader.SetIntegrationPoints(2);
            Mloadcontainer->Add(faceload);
        }
    }
    my_system.Add(Mloadcontainer);
    // Constraints for each mesh rim

    if (addConstRim) {
        int NoConstNodes = 2 * numEl_Diameter;
        int ConstNodeInx = -1;

        for (int j = 0; j < N_Diameter; j++) {
            for (int i = 0; i < N_Thread; i++) {
                ConstNodeInx = i + j * N_Thread;  // General node index
                int indexConst = (i + j * N_Thread) % N_Thread;
                if (indexConst == 0) {
                    // First node to be constrained (one side of the rim)
                    ConstrainedNode =
                        ChSharedPtr<ChNodeFEAxyzD>(TireMesh->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                    NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                    NodePosRim->Initialize(ConstrainedNode, Hub_1);
                    my_system.Add(NodePosRim);
                    NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                    NodeDirRim->Initialize(ConstrainedNode, Hub_1);
                    NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                    my_system.Add(NodeDirRim);

                    // Second node to be constrained (other side of the rim)
                    ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(
                        TireMesh->GetNode(ConstNodeInx + N_Thread - 1).DynamicCastTo<ChNodeFEAxyzD>());

                    NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                    NodePosRim2->Initialize(ConstrainedNode2, Hub_1);
                    my_system.Add(NodePosRim2);
                    NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                    NodeDirRim2->Initialize(ConstrainedNode2, Hub_1);
                    NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                    my_system.Add(NodeDirRim2);
                }
            }
        }
    }
    ChSharedPtr<ChLoadContainer> mloadcontainerGround(new ChLoadContainer);
    if (addGroundForces) {
        // Select on which nodes we are going to apply a load
        std::vector<ChSharedPtr<ChLoadable>> NodeList1;
        for (int iNode = 0; iNode < TotalNumNodes; iNode++) {
            ChSharedPtr<ChNodeFEAxyzD> NodeLoad1(TireMesh->GetNode(iNode).DynamicCastTo<ChNodeFEAxyzD>());
            NodeList1.push_back(NodeLoad1);
        }
        ChSharedPtr<MyLoadCustomMultiple> Mloadcustommultiple1(new MyLoadCustomMultiple(NodeList1));
        mloadcontainerGround->Add(Mloadcustommultiple1);

    }  // End loop over tires
    my_system.Add(mloadcontainerGround);

    if (showVisual) {
        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(new ChVisualizationFEAmesh(*(TireMesh.get_ptr())));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.008);
        TireMesh->AddAsset(mvisualizemeshC);

        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshwire(new ChVisualizationFEAmesh(*(TireMesh.get_ptr())));
        mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshwire->SetWireframe(true);
        TireMesh->AddAsset(mvisualizemeshwire);

        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(new ChVisualizationFEAmesh(*(TireMesh.get_ptr())));
        mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
        mvisualizemesh->SetColorscaleMinMax(0.0, 30);
        mvisualizemesh->SetSmoothFaces(true);
        TireMesh->AddAsset(mvisualizemesh);
    }
}

int main(int argc, char* argv[]) {
    // Definition of the model
    ChSystem my_system;

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

    // Create hubs and tire meshes for 4 wheels
    ChSharedPtr<ChBody> Hub_1(new ChBody);
    ChSharedPtr<ChBody> Hub_2(new ChBody);
    ChSharedPtr<ChBody> Hub_3(new ChBody);
    ChSharedPtr<ChBody> Hub_4(new ChBody);

    ChVector<> rim_center_1(-Lwx, 0, Lwz);
    ChVector<> rim_center_2(Lwx, 0, Lwz);
    ChVector<> rim_center_3(Lwx, 0, -Lwz);
    ChVector<> rim_center_4(-Lwx, 0, -Lwz);

    ChSharedPtr<ChMaterialShellANCF> mat(new ChMaterialShellANCF(500, 9.0e7, 0.3));

    MakeANCFWheel(my_system, rim_center_1, mat, Hub_1, N_Diameter, N_Thread, TorusRadius, TorusSmallRadius,
                  TirePressure, ForVelocity);
    MakeANCFWheel(my_system, rim_center_2, mat, Hub_2, N_Diameter, N_Thread, TorusRadius, TorusSmallRadius,
                  TirePressure, ForVelocity);
    MakeANCFWheel(my_system, rim_center_3, mat, Hub_3, N_Diameter, N_Thread, TorusRadius, TorusSmallRadius,
                  TirePressure, ForVelocity);
    MakeANCFWheel(my_system, rim_center_4, mat, Hub_4, N_Diameter, N_Thread, TorusRadius, TorusSmallRadius,
                  TirePressure, ForVelocity);

    ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
    mmaterial->SetFriction(0.4f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    // SimpChassis = ChSharedPtr<ChBody>(new ChBody);
    ChSharedPtr<ChBodyEasyBox> SimpChassis(new ChBodyEasyBox(1.1, 0.2, 2.4,  // x,y,z size
                                                             100,            // density
                                                             false,          // collide enable?
                                                             true));         // visualization?
    my_system.AddBody(SimpChassis);
    SimpChassis->SetMaterialSurface(mmaterial);  // use shared surface properties
    // optional, attach a texture for better visualization
    ChSharedPtr<ChTexture> mtexturebox(new ChTexture());
    mtexturebox->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));
    SimpChassis->SetPos(ChVector<>(0, 0, 0));
    SimpChassis->SetPos_dt(ChVector<>(0, 0, ForVelocity));
    SimpChassis->AddAsset(mtexturebox);
    SimpChassis->SetBodyFixed(false);

    // Create joints between chassis and hubs
    ChSharedPtr<ChLinkRevoluteTranslational> RevTr_1(new ChLinkRevoluteTranslational);
    my_system.AddLink(RevTr_1);
    RevTr_1->Initialize(Hub_1, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0), ChVector<>(-Lwx, 0.1, Lwz),
                        ChVector<>(0, 0, 1), ChVector<>(0, 1, 0), true);

    ChSharedPtr<ChLinkRevoluteTranslational> RevTr_2(new ChLinkRevoluteTranslational);
    my_system.AddLink(RevTr_2);
    RevTr_2->Initialize(Hub_2, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0), ChVector<>(Lwx, 0.1, Lwz),
                        ChVector<>(0, 0, 1), ChVector<>(0, 1, 0), true);

    ChSharedPtr<ChLinkRevoluteTranslational> RevTr_3(new ChLinkRevoluteTranslational);
    my_system.AddLink(RevTr_3);
    RevTr_3->Initialize(Hub_3, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0), ChVector<>(Lwx, 0.1, -Lwz),
                        ChVector<>(0, 0, 1), ChVector<>(0, 1, 0), true);

    ChSharedPtr<ChLinkRevoluteTranslational> RevTr_4(new ChLinkRevoluteTranslational);
    my_system.AddLink(RevTr_4);
    RevTr_4->Initialize(Hub_4, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0), ChVector<>(-Lwx, 0.1, -Lwz),
                        ChVector<>(0, 0, 1), ChVector<>(0, 1, 0), true);

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

    
    // Create a large cube as a floor.

    ChSharedPtr<ChBodyEasyBox> mrigidBody( new ChBodyEasyBox(10,0.00001, 10, 1000,
                                                        false, // no collide
                                                        true)); // visualize
    my_system.Add(mrigidBody);
    mrigidBody->SetPos(ChVector<>(0, GroundLoc, 0));
    mrigidBody->SetBodyFixed(true);
    mrigidBody->GetMaterialSurface()->SetFriction(0.0);

    ChSharedPtr<ChTexture> mtexture( new ChTexture(GetChronoDataFile("concrete.jpg").c_str()));
    mrigidBody->AddAsset(mtexture);
    


    my_system.Set_G_acc(ChVector<>(0, -9.81, 0));

    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(false);
    mkl_solver_stab->SetSparsityPatternLock(false);

    my_system.SetIntegrationType(ChSystem::INT_HHT);
    // my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
    mystepper->SetAlpha(-0.3);  // Important for convergence
    mystepper->SetMaxiters(11);
    mystepper->SetAbsTolerances(5e-03);
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

    application.AssetBindAll();
    application.AssetUpdateAll();


    chrono::GetLog()
        << "\n\nREADME\n\n"
        << " - Press SPACE to start dynamic simulation \n";

    // at beginning, no analysis is running..
    application.SetPaused(false);
    int AccuNoIterations = 0;
    application.SetStepManage(true);
    application.SetTimestep(time_step);
    application.SetTryRealtime(false);
    double ChTime = 0.0;

    const double VerForce = 0;
    const double HorForce = 80;
    const double tini = 0.1;
    const double tend = 0.2;
    const double interval = tend - tini;
    while (application.GetDevice()->run()) {
        /*Hub_1->Empty_forces_accumulators();
        Hub_2->Empty_forces_accumulators();
        // Hub_1->Set_Scr_force(const ChVector<>& mf) { Scr_force = mf; }
        if (my_system.GetChTime() >= tini && my_system.GetChTime() <= tend) {
            Hub_1->Set_Scr_torque(ChVector<>(HorForce * (my_system.GetChTime() - tini) / interval, 0, 0));
            Hub_2->Set_Scr_torque(ChVector<>(HorForce * (my_system.GetChTime() - tini) / interval, 0, 0));
        } else if (my_system.GetChTime() > tend) {
            Hub_1->Set_Scr_torque(ChVector<>(HorForce, 0, 0));
            Hub_2->Set_Scr_torque(ChVector<>(HorForce, 0, 0));
        }*/

        application.BeginScene();
        application.DrawAll();
        // If time is larger than 0.015s, doapplicationsetstep(smallertimestep)
        application.DoStep();
        application.EndScene();
        if (!application.GetPaused()) {
            std::cout << "Time t = " << my_system.GetChTime() << "s \n";
            // AccuNoIterations += mystepper->GetNumIterations();
            printf("Vertical position of Tires:      %12.4e       %12.4e       %12.4e       %12.4e  Chassis   \n",
                   Hub_1->GetPos().y, Hub_2->GetPos().y, Hub_3->GetPos().y, Hub_4->GetPos().y);

            printf("Longitudinal position of Tires:      %12.4e       %12.4e       %12.4e       %12.4e  Chassis  ",
                   Hub_1->GetPos().z, Hub_2->GetPos().z, Hub_3->GetPos().z, Hub_4->GetPos().z);
            out << my_system.GetChTime() << Hub_1->GetPos().x << Hub_1->GetPos().y << Hub_1->GetPos().z
                << Hub_2->GetPos().x << Hub_2->GetPos().y << Hub_2->GetPos().z << Hub_3->GetPos().x << Hub_3->GetPos().y
                << Hub_3->GetPos().z << std::endl;
            out.write_to_file("../VertPosRim.txt");
        }
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    chrono::GetLog() << "Computation Time: " << duration;
    system("pause");

    /*my_system.Setup();
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
    system("pause");*/
    return 0;
}
