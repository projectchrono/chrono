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
std::shared_ptr<ChBody> BGround;
std::shared_ptr<ChBodyEasyBox> SimpChassis;  // Chassis body
std::shared_ptr<ChLinkPointFrame> constraint;  // Create shared pointers for rim-mesh constraints
std::shared_ptr<ChLinkDirFrame> constraintD;
std::shared_ptr<ChNodeFEAxyzD> ConstrainedNode;
std::shared_ptr<ChLinkLockPlanePlane> constraintRim;

auto MloadcontainerGround = std::make_shared<ChLoadContainer>();
// Some model parameters
const double spring_coef = 3e4;  // Springs and dampers for strut
const double damping_coef = 1e3;
const int num_steps = 1500;     // Number of time steps for unit test (range 1 to 4000) 550
double time_step = 0.00015;      //
const double ForVelocity = 2;   // Initial velocity of the tire. Applied to hub, nodes, and chassis
double TirePressure = -120e3;    // Applied suddently at the beginning of simulation// Was 120e3 // I CHANGED THE SIGN 1/25/2016 9:00PM
const double HumveeVertPos = 0.4673;   // Vertical (z axis) center of rim
// Location of the wheels w.r.t. rigid body's center of mass
const double Lwx = 1.1;  // Value of longitudinal distance
const double Lwy = 0.7;  // Value of lateral distance
const int NoTires = 4;   // Number of tires. Programmer
const double GroundLoc = 0.0000; // Redefined?

// Read input file for the comprehensive Humvee tire
void ReadInputFile(ChMatrixNM<double, 3000, 6> &COORDFlex, ChMatrixNM<double, 3000, 6> &VELCYFlex, ChMatrixNM<int, 2880, 4> &NodesPerElement, int &TotalNumElements, int &NumElements_x, int &NumElements_y, int &TotalNumNodes, ChMatrixNM<int, 2880, 1> &SectionID, ChMatrixNM<double, 15, 2> &LayerPROP, ChMatrixNM<int, 15, 7> &MatID, ChMatrixNM<double, 7, 12> &MPROP, ChMatrixNM<double, 2880, 2> &ElementLength, ChMatrixNM<int, 3, 1> &NumLayPerSect)
{
    FILE *inputfile;
    char str1[100];
    int numFlexBody = 0;
    int dummy;
    int count;

    //double ACCELFlex[4000][6];
    //double ACCELRigid[2][7];
    //double LuGreZStart[25][40];
    //double LuGreZStart_dt[25][40];
    //double LuGreZStart_dtdt[25][40];

    int NDR[4000][6];
    int NumLayer[10];
    int MaxSectionNumber = 0;
    int MaxMatID = 0;
    int MTYPE = 0;

    int MAXCOUNT = 100;
    inputfile = fopen(GetChronoDataFile("fea/ANCFtire/IndataBiLinearShell_Tire(HMMWV50x24).INP").c_str(), "r");
    printf("Open IndataBiLinearShell_Tire(HMMWV50x24).INP \n");
    if (inputfile == NULL){
        printf("Input data file not found!!\n");
        system("pause");
        exit(1);
    }

    TotalNumElements = 0;
    NumElements_x = 0;
    NumElements_y = 0;
    TotalNumNodes = 0;

    //!--------------------------------------!
    //!-- Elememt data            -----------!
    //!--------------------------------------!

    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    fscanf(inputfile, "%d\n", &numFlexBody);

    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    fscanf(inputfile, "%d %d %d %d\n", &TotalNumElements, &NumElements_x, &NumElements_y, &TotalNumNodes);
    fgets(str1, MAXCOUNT, inputfile);

    printf("%s\n", str1);
    for (int i = 0; i<TotalNumElements; i++)
    {
        fscanf(inputfile, "%d %d %d %d %d %d %d\n", &count, &dummy, &SectionID(i, 0), &NodesPerElement(i, 0), &NodesPerElement(i, 1), &NodesPerElement(i, 2), &NodesPerElement(i, 3));
        printf("SectionID[i] %d\n  ", SectionID(i, 0));

        fscanf(inputfile, " %lf %lf\n", &ElementLength(i, 0), &ElementLength(i, 1));
        if (MaxSectionNumber<SectionID(i, 0))
        {
            MaxSectionNumber = SectionID(i, 0);
        }
        //if(TotalNumNodes<max(NumNodes[i][0],max(NumNodes[i][1],max(NumNodes[i][2],NumNodes[i][3]))))
        //{TotalNumNodes=max(NumNodes[i][0],max(NumNodes[i][1],max(NumNodes[i][2],NumNodes[i][3])));}

        //printf("MaxSectionNumber %lf, %lf \n ", ElemLengthXY[i][0],ElemLengthXY[i][1]);
    }

    //!--------------------------------------!
    //!-- NDR,COORDFlex,VELCYFlex -----------!
    //!--------------------------------------!
    //fscanf(inputfile,"%s\n",str1);
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    for (int i = 0; i<TotalNumNodes; i++)
    {
        fscanf(inputfile, "%d %d %d %d %d %d %d\n", &count, &NDR[i][0], &NDR[i][1], &NDR[i][2], &NDR[i][3], &NDR[i][4], &NDR[i][5]);
        fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &COORDFlex(i, 0), &COORDFlex(i, 1), &COORDFlex(i, 2), &COORDFlex(i, 3), &COORDFlex(i, 4), &COORDFlex(i, 5));
        fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &VELCYFlex(i, 0), &VELCYFlex(i, 1), &VELCYFlex(i, 2), &VELCYFlex(i, 3), &VELCYFlex(i, 4), &VELCYFlex(i, 5));
        //printf("NumNodes %d %d %d %d %d %d\n",NDR[i][0],NDR[i][1],NDR[i][2],NDR[i][3],NDR[i][4],NDR[i][5]);
        //printf("NumNodes %lf %lf %lf %lf %lf %lf\n",COORDFlex[i][0],COORDFlex[i][1],COORDFlex[i][2],COORDFlex[i][3],COORDFlex[i][4],COORDFlex[i][5]);
        //system("pause");
    }

    //!--------------------------------------!
    //!--- Read Layer Data ------------------!
    //!--------------------------------------!
    //fscanf(inputfile,"%s\n",str1);
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    int counted = 0;
    for (int i = 0; i<MaxSectionNumber; i++)
    {
        fscanf(inputfile, "%d %d\n", &count, &NumLayer[i]);
        for (int j = 0; j<NumLayer[i]; j++)
        {
            fscanf(inputfile, "%lf %lf %d\n", &LayerPROP(counted + j, 0), &LayerPROP(counted + j, 1), &MatID(i, j));
            if (MaxMatID<MatID(i, j))
            {
                MaxMatID = MatID(i, j);
            }
            NumLayPerSect(i) = NumLayer[i];
            //printf("%lf %lf %d\n%d\n", LayerPROP(counted + j, 0), LayerPROP(counted + j, 1), MatID(i, j), counted + j);
        }
        counted += NumLayPerSect(i);
        //system("pause");
    }

    //!--------------------------------------!
    //!--- Read Material Data ---------------!
    //!--------------------------------------!
    //fscanf(inputfile,"%s\n",str1);
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    for (int i = 0; i<MaxMatID; i++)
    {
        fscanf(inputfile, "%d %d\n", &count, &MTYPE);
        if (MTYPE == 1)
        {
            fscanf(inputfile, "%lf %lf %lf %lf\n", &MPROP(i, 0), &MPROP(i, 1), &MPROP(i, 2), &MPROP(i, 3));
        }
        if (MTYPE == 2)
        {
            fscanf(inputfile, "%lf %lf %lf %lf\n", &MPROP(i, 0), &MPROP(i, 1), &MPROP(i, 2), &MPROP(i, 3));
            fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &MPROP(i, 4), &MPROP(i, 5), &MPROP(i, 6), &MPROP(i, 7), &MPROP(i, 8), &MPROP(i, 9));

        }
        //printf("%lf %lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2],MPROP[i][3]);
    }

};

// ChLoadCustomMultiple to include basic node-Ground contact interaction
class MyLoadCustomMultiple : public ChLoadCustomMultiple {
  public:
    MyLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& mloadables) : ChLoadCustomMultiple(mloadables){};
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {

        std::vector<std::shared_ptr<ChLoadable> > NodeList;
        
        ChVector<> Node1_Pos;
        ChVector<> Node1_Vel;
        ChVector<> Node1_Grad;
        ChVector<> Node1_GradVel;
        // this->load_Q.FillElem(0);
        double KGround = 9e5;
        double CGround = KGround;
        double NormalForceNode = 0;
        double FrictionCoeff = 0.7;

        // Calculation of number of nodes in contact with the Ground
        int NoCNodes = 0;
        for (int iii = 0; iii < loadables.size(); iii++) {
            Node1_Pos = state_x->ClipVector(iii * 6, 0);
            if (Node1_Pos.z < GroundLoc) {
             //  chrono::GetLog() << " \n Node1_Pos.z: " << Node1_Pos.z << "\n GroundLoc: " << GroundLoc << "  Number: " << iii;
                NoCNodes++;
            }
        }
        if (NoCNodes > 0) {
            KGround = 9e5 / double(NoCNodes);
            CGround = 0.001 * KGround;
        }
       // chrono::GetLog() << "  \n"
       //                  << "Nodes into contact:   " << NoCNodes << " \n";
        if (state_x && state_w) {
            for (int iii = 0; iii < loadables.size(); iii++) {
                Node1_Pos = state_x->ClipVector(iii * 6, 0);
                Node1_Grad = state_x->ClipVector(iii * 6 + 3, 0);
                Node1_Vel = state_w->ClipVector(iii * 6, 0);
                Node1_GradVel = state_w->ClipVector(iii * 6 + 3, 0);
                if (Node1_Pos.z < GroundLoc) {
                    double Penet = abs(Node1_Pos.z - GroundLoc);
                    // GetLog() << "Node number:  " << iii << ".  "
                    //          << "Penetration:  " << Penet << "\n";
                    NormalForceNode = KGround * Penet;  // +CGround * abs(Node1_Vel.y*Penet);
                    this->load_Q(iii * 6 + 2) =
                        NormalForceNode - CGround * (Node1_Vel.z) * abs(Penet);  // Fy (Vertical)
                    // Friction forces
                    const double VelLimit = 0.1;
                    if (abs(Node1_Vel.x) > VelLimit) {
                        this->load_Q(iii * 6 + 0) =
                            -NormalForceNode * FrictionCoeff *
                            (Node1_Vel.x / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.y, 2))));  // Fx (Plane x)
                    } else {
                        this->load_Q(iii * 6 + 0) =
                            -NormalForceNode * FrictionCoeff * sin(abs(Node1_Vel.x) * CH_C_PI_2 / VelLimit) *
                            (Node1_Vel.x / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.y, 2))));  // Fx (Plane x)
                    }
                    if (abs(Node1_Vel.y) > VelLimit) {
                        this->load_Q(iii * 6 + 1) =
                            -NormalForceNode * FrictionCoeff *
                            (Node1_Vel.y / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.y, 2))));  // Fz (Plane y)
                    } else {
                        this->load_Q(iii * 6 + 1) =
                            -NormalForceNode * FrictionCoeff * sin(abs(Node1_Vel.z) * CH_C_PI_2 / VelLimit) *
                            (Node1_Vel.y / sqrt((pow(Node1_Vel.x, 2) + pow(Node1_Vel.y, 2))));  // Fz (Plane y)
                    }
                }
            }
        } else {
            // explicit integrators might call ComputeQ(0,0), null pointers mean
            // that we assume current state, without passing state_x for efficiency
            GetLog() << "\n This should never happen \n";
        }
    }
    virtual bool IsStiff() { return true; }
};

void MakeANCFHumveeWheel(ChSystem& my_system,
                         const ChVector<> rim_center,
                         std::shared_ptr<ChBody>& Hub_1,
                         double TirePressure,
                         double ForVelocity, int Ident) {
    // Create rim for this mesh
    my_system.AddBody(Hub_1);
    Hub_1->SetIdentifier(Ident);
    Hub_1->SetBodyFixed(false);
    Hub_1->SetCollide(false);
    Hub_1->SetMass(10);
    Hub_1->SetInertiaXX(ChVector<>(0.3, 0.3, 0.3));
    Hub_1->SetPos(rim_center);  // Y = -1m
    Hub_1->SetPos_dt(ChVector<>(ForVelocity, 0, 0));
    Hub_1->SetWvel_par(ChVector<>(0, ForVelocity / (HumveeVertPos), 0)); //0.3 to be substituted by an actual measure of the average radius.

    // Create tire mesh
    auto TireMesh = std::make_shared<ChMesh>();

    //  Fixing constraints, initial coordinates and velocities
    // READ INPUT DATA AND CREATE ARRAYS

    // Creating arrays for inputting data
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Tire (Fixed),  implicit integration \n\n";

    // Boolean variables to determine which output files are written
    bool output = true;
    bool output1 = true;
    bool output2 = true;
    bool output3 = true;
    bool output4 = true;

    int TotalNumNodes;
    // Matricies to hold the state informatino for the nodes and rigid bodies
    ChMatrixNM<double, 3000, 6> COORDFlex;
    ChMatrixNM<double, 3000, 6> VELCYFlex;
    ChMatrixNM<double, 3000, 6> ACCELFlex;
    ChMatrixNM<double, 2, 7> COORDRigid;
    ChMatrixNM<double, 2, 7> VELCYRigid;
    ChMatrixNM<double, 2, 7> ACCELRigid;

    ChMatrixNM<int, 2880, 4> NodesPerElement;  // Defines the connectivity between the elements and nodes
    ChMatrixNM<double, 2880, 2> ElemLength;    // X and Y dimensions of the shell elements
    int TotalNumElements;
    int NumElements_x;
    int NumElements_y;
    ChMatrixNM<int, 2880, 1> SectionID;  // Catagorizes which tire section the elements are a part of
    ChMatrixNM<double, 15, 2> LayPROP;   // Thickness and ply angles of the layered elements
    ChMatrixNM<int, 15, 7> MatID;  // Catagorizes the material of each layer
    ChMatrixNM<double, 7, 12> MPROP;  // Material properties
    ChMatrixNM<int, 3, 1> NumLayPerSection;
    double ContactZ = 0.0;  // Vertical location of the flat ground
    ChVector<> NetContact;  // Net contact forces

    // End of declaration of arrays for inputting data

    // Read actual data file
    ReadInputFile(COORDFlex, VELCYFlex, NodesPerElement, TotalNumElements, NumElements_x, NumElements_y, TotalNumNodes,
                  SectionID, LayPROP, MatID, MPROP, ElemLength, NumLayPerSection);
    ///////////////////////////////////////////////////////////////////////////
    // Assign the humvee mesh properties to our ChMesh
    //// Material List (for HMMWV)
    //// i=0: Carcass
    //// i=1: Steel belt in rubber matrix
    //// i=2: Rubber
    ///////////////////////////////////////////////////////////////////////////

    std::vector<std::shared_ptr<ChMaterialShellANCF> > MaterialList(MPROP.GetRows());
    for (int i = 0; i < MPROP.GetRows(); i++) {
        double rho = MPROP(i, 0);
        ChVector<double> E(MPROP(i, 1), MPROP(i, 2), MPROP(i, 3));
        ChVector<double> nu(MPROP(i, 4), MPROP(i, 5), MPROP(i, 6));
        ChVector<double> G(MPROP(i, 7), MPROP(i, 8), MPROP(i, 9));
        MaterialList[i] = std::make_shared<ChMaterialShellANCF>(rho, E, nu, G);
    }

    // Create a set of nodes for the tire based on the input data
    for (int i = 0; i < TotalNumNodes; i++) {

        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(COORDFlex(i, 0) + rim_center.x, COORDFlex(i, 1) +rim_center.y, COORDFlex(i, 2)),
                              ChVector<>(COORDFlex(i, 3), COORDFlex(i, 4), COORDFlex(i, 5)));
        node->SetPos_dt(ChVector<>(VELCYFlex(i, 0), VELCYFlex(i, 1), VELCYFlex(i, 2)));
        node->SetD_dt(ChVector<>(VELCYFlex(i, 3), VELCYFlex(i, 4), VELCYFlex(i, 5)));
        node->SetPos_dtdt(ChVector<>(ACCELFlex(i, 0), ACCELFlex(i, 1), ACCELFlex(i, 2)));
        node->SetD_dtdt(ChVector<>(ACCELFlex(i, 3), ACCELFlex(i, 4), ACCELFlex(i, 5)));
        node->SetMass(0.0);

        TireMesh->AddNode(node);  // Add nodes to the system
    }
    // Check position of the bottom node
    GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode((TotalNumElements / 2)));
    GetLog() << "X : " << nodetip->GetPos().x << " Y : " << nodetip->GetPos().y << " Z : " << nodetip->GetPos().z
             << "\n\n";
    GetLog() << "dX : " << nodetip->GetD().x << " dY : " << nodetip->GetD().y << " dZ : " << nodetip->GetD().z
             << "\n\n";

    int LayerHist = 0;  // Number of layers in the previous tire sections

    // Create all elements of the tire
    for (int i = 0; i < TotalNumElements; i++) {
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(NodesPerElement(i, 0) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(NodesPerElement(i, 1) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(NodesPerElement(i, 2) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(NodesPerElement(i, 3) - 1)));
        element->SetDimensions(ElemLength(i, 0), ElemLength(i, 1));

        // Determine the section in which the current element resides
        switch (SectionID(i)) {
            // Bead section
            case 1: {
                LayerHist = 0;
                break;
            }
            // Sidewall section
            case 2: {
                LayerHist = NumLayPerSection(0);
                break;
            }
            // Tread section
            case 3: {
                LayerHist = NumLayPerSection(0) + NumLayPerSection(1);
                break;
            }
        }  // End of switch

        // Give material properties to elements as a construction of layers
        for (int j = 0; j < NumLayPerSection(SectionID(i) - 1); j++) {
            element->AddLayer(LayPROP(LayerHist + j, 0), LayPROP(LayerHist + j, 1) * CH_C_DEG_TO_RAD,
                              MaterialList[MatID(SectionID(i) - 1, j) - 1]);
            // GetLog() << "Thickness: " << LayPROP(LayerHist + j, 0) << "  Ply: " << LayPROP(LayerHist + j, 1) << "
            // Mat: " << MatID(SectionID(i) - 1, j) << "\n";
            // GetLog() << "Index: " << LayerHist + j << "   PRev: " << LayerHist << "\n";
        }
        element->SetAlphaDamp(0.01); //0.005
        element->SetGravityOn(true);
        TireMesh->AddElement(element);
    }
    // End of assigning properties to TireMesh (ChMesh)
    // Create constraints for the tire and rim
    // Constrain the flexible tire to the rigid rim body.
    if (addConstRim) {
        for (int i = 0; i < TotalNumNodes; i++) {
            if (i < NumElements_x ||
                i >= TotalNumNodes - NumElements_x) {  // Only constrain the nodes at the ends of the bead section
                ConstrainedNode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(i));
                // Add position constraints
                constraint = std::make_shared<ChLinkPointFrame>();
                constraint->Initialize(ConstrainedNode, Hub_1);
                my_system.Add(constraint);

                // Add rotation constraints
                constraintD = std::make_shared<ChLinkDirFrame>();
                constraintD->Initialize(ConstrainedNode, Hub_1);
                constraintD->SetDirectionInAbsoluteCoords(ConstrainedNode->GetD());
                my_system.Add(constraintD);
            }
        }
    }

    // END OF INPUT DATA AND CREATE ARRAYS

    // Add initial velocity to the nodes (for rolling)
    for (unsigned int i = 0; i < TireMesh->GetNnodes(); ++i) {
        ChVector<> node_pos = std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(i))->GetPos();
        double tang_vel =
            ForVelocity * (node_pos.z) / (HumveeVertPos);
        ChVector<> NodeVel(tang_vel, 0, 0.0);
        std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(i))->SetPos_dt(NodeVel);
    }

    // Switch off mesh class gravity
    TireMesh->SetAutomaticGravity(false);

    // Add the mesh to the system
    my_system.Add(TireMesh);
    auto Mloadcontainer = std::make_shared<ChLoadContainer>();
    // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
    if (addPressureAlessandro) {
        for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            auto faceload = std::make_shared<ChLoad<ChLoaderPressure > >(std::static_pointer_cast<ChElementShellANCF>(TireMesh->GetElement(NoElmPre)));
            faceload->loader.SetPressure(-TirePressure);
            faceload->loader.SetStiff(false);
            faceload->loader.SetIntegrationPoints(2);
            Mloadcontainer->Add(faceload);
        }
    }
    my_system.Add(Mloadcontainer);
    // Constraints for each mesh rim
    auto mloadcontainerGround = std::make_shared<ChLoadContainer>();

    if (addGroundForces) {
        // Select on which nodes we are going to apply a load

        for (int iNode = 0; iNode < TotalNumNodes; iNode++) {
            std::vector<std::shared_ptr<ChLoadable> > NodeList1;
            auto NodeLoad1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(TireMesh->GetNode(iNode));
            NodeList1.push_back(NodeLoad1);
            auto Mloadcustommultiple1 = std::make_shared<MyLoadCustomMultiple>(NodeList1);
            mloadcontainerGround->Add(Mloadcustommultiple1);
        }

    }  // End loop over tires
    my_system.Add(mloadcontainerGround);

    if (showVisual) {
        auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(TireMesh.get()));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.005);
        TireMesh->AddAsset(mvisualizemeshC);

        auto mvisualizemeshwire = std::make_shared<ChVisualizationFEAmesh>(*(TireMesh.get()));
        mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshwire->SetWireframe(true);
        TireMesh->AddAsset(mvisualizemeshwire);

        auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(TireMesh.get()));
        //mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
        mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRAIN_VONMISES);
        mvisualizemesh->SetColorscaleMinMax(-0.05, 0.05);
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
    BGround = std::make_shared<ChBody>();
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
    auto Hub_1 = std::make_shared<ChBody>();
    auto Hub_2 = std::make_shared<ChBody>();
    auto Hub_3 = std::make_shared<ChBody>();
    auto Hub_4 = std::make_shared<ChBody>();

    ChVector<> rim_center_1(Lwx, -Lwy, HumveeVertPos); //
    ChVector<> rim_center_2(Lwx, Lwy, HumveeVertPos);
    ChVector<> rim_center_3(-Lwx, Lwy, HumveeVertPos);
    ChVector<> rim_center_4(-Lwx, -Lwy, HumveeVertPos);

    MakeANCFHumveeWheel(my_system, rim_center_1, Hub_1,
                  TirePressure, ForVelocity, 2);
    MakeANCFHumveeWheel(my_system, rim_center_2, Hub_2,
        TirePressure, ForVelocity, 3);
    MakeANCFHumveeWheel(my_system, rim_center_3, Hub_3,
        TirePressure, ForVelocity, 4);
    MakeANCFHumveeWheel(my_system, rim_center_4, Hub_4,
        TirePressure, ForVelocity, 5);

    auto mmaterial = std::make_shared<ChMaterialSurface>();
    mmaterial->SetFriction(0.4f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    SimpChassis = std::make_shared<ChBodyEasyBox>(2.4, 1.1, 0.2,  // x,y,z size
                                                  2800,           // density
                                                  false,          // collide enable?
                                                  true);          // visualization?
    my_system.AddBody(SimpChassis);
    SimpChassis->SetMaterialSurface(mmaterial);  // use shared surface properties
    // optional, attach a texture for better visualization
    auto mtexturebox = std::make_shared<ChTexture>();
    mtexturebox->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));
    SimpChassis->SetPos(ChVector<>(0, 0, HumveeVertPos));
    SimpChassis->SetPos_dt(ChVector<>(ForVelocity, 0, 0));
    SimpChassis->AddAsset(mtexturebox);
    SimpChassis->SetBodyFixed(false);
    // */
    // Create joints between chassis and hubs
    auto RevTr_1 = std::make_shared<ChLinkRevoluteTranslational>();
    my_system.AddLink(RevTr_1);
    RevTr_1->Initialize(Hub_1, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), ChVector<>(Lwx, -Lwy, 0.1),
        ChVector<>(0, 0, 1), ChVector<>(1, 0, 0), true);

    auto RevTr_2 = std::make_shared<ChLinkRevoluteTranslational>();
    my_system.AddLink(RevTr_2);
    RevTr_2->Initialize(Hub_2, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), ChVector<>(Lwx, Lwy, 0.1),
        ChVector<>(0, 0, 1), ChVector<>(1, 0, 0), true);

    auto RevTr_3 = std::make_shared<ChLinkRevoluteTranslational>();
    my_system.AddLink(RevTr_3);
    RevTr_3->Initialize(Hub_3, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), ChVector<>(-Lwx, Lwy, 0.1),
        ChVector<>(0, 0, 1), ChVector<>(1, 0, 0), true);

    auto RevTr_4 = std::make_shared<ChLinkRevoluteTranslational>();
    my_system.AddLink(RevTr_4);
    RevTr_4->Initialize(Hub_4, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), ChVector<>(-Lwx, -Lwy, 0.1),
        ChVector<>(0, 0, 1), ChVector<>(1, 0, 0), true);

    // Spring and damper for secondary suspension: True position vectors are relative
    auto spring1 = std::make_shared<ChLinkSpring>();
    spring1->Initialize(Hub_1, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(Lwx, -Lwy, 0), true);
    spring1->Set_SpringK(spring_coef);
    spring1->Set_SpringR(damping_coef);
    my_system.AddLink(spring1);

    auto spring2 = std::make_shared<ChLinkSpring>();
    spring2->Initialize(Hub_2, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(-Lwx, -Lwy, 0), true);
    spring2->Set_SpringK(spring_coef);
    spring2->Set_SpringR(damping_coef);
    my_system.AddLink(spring2);

    auto spring3 = std::make_shared<ChLinkSpring>();
    spring3->Initialize(Hub_3, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(-Lwx, Lwy, 0), true);
    spring3->Set_SpringK(spring_coef);
    spring3->Set_SpringR(damping_coef);
    my_system.AddLink(spring3);

    auto spring4 = std::make_shared<ChLinkSpring>();
    spring4->Initialize(Hub_4, SimpChassis, true, ChVector<>(0, 0, 0), ChVector<>(Lwx, Lwy, 0), true);
    spring4->Set_SpringK(spring_coef);
    spring4->Set_SpringR(damping_coef);
    my_system.AddLink(spring4);


    // Create a large cube as a floor.

    auto mrigidBody = std::make_shared<ChBodyEasyBox>(10,10,0.00001, 1000,
                                                        false, // no collide
                                                        true); // visualize
    my_system.Add(mrigidBody);
    mrigidBody->SetPos(ChVector<>(0, 0, GroundLoc));
    mrigidBody->SetBodyFixed(true);
    mrigidBody->GetMaterialSurface()->SetFriction(0.0);
    auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg").c_str());
    mrigidBody->AddAsset(mtexture);
    

    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));
    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);

    my_system.SetIntegrationType(ChSystem::INT_HHT);
    // my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.3);  // Important for convergence
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(6e-03, 2.5);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);  //
    mystepper->SetVerbose(true);
    mystepper->SetRequiredSuccessfulSteps(2);
    mystepper->SetMaxItersSuccess(7);
    ChMatrixNM<double, 3, 1> Cp;
    ChMatrixNM<double, 2, 1> Cd;  // Matrices for storing constraint violations

    // Visualization
    /*
    auto mobjmesh = std::make_shared<ChObjShapeFile>();
    mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    Hub_1->AddAsset(mobjmesh);
    Hub_2->AddAsset(mobjmesh);
    Hub_3->AddAsset(mobjmesh);
    Hub_4->AddAsset(mobjmesh);
    */

    double start = std::clock();

    application.AssetBindAll();
    application.AssetUpdateAll();


    my_system.SetupInitial();
    my_system.Setup();
    my_system.Update();

    chrono::GetLog()
        << "\n\nREADME\n\n"
        << " - Press SPACE to start dynamic simulation \n - Press F10 for nonlinear statics - Press F11 for "
           "linear statics. \n";

    // at beginning, no analysis is running..
    application.SetPaused(true);
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

        application.BeginScene();
        application.DrawAll();
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
