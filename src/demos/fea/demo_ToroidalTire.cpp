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
using namespace irr;
using namespace scene;

bool addConstRim = true;
bool addBodies = true;
bool addGroundForces = true;
bool showVisual = true;
bool addSingleLoad = false;
bool addPressure = false;
bool addPressureAlessandro = true;

ChSharedPtr<ChBody> BGround;
ChSharedPtr<ChBody> Body_2;  // Rim (unsuspended)
ChSharedPtr<ChBody> Body_3;  // Hub
ChSharedPtr<ChBody> Body_4;  // Suspended
ChSharedPtr<ChLinkPointFrame> NodePosRim;
ChSharedPtr<ChLinkDirFrame> NodeDirRim;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode;
ChSharedPtr<ChLinkPointFrame> NodePosRim2;
ChSharedPtr<ChLinkDirFrame> NodeDirRim2;
ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode2;

// Some model parameters

const int num_steps = 550;  // Number of time steps for unit test (range 1 to 4000) 550
double time_step = 0.001;   // Time step: 0.001 It works like that, but rotates at the beginning

const double ForVel = 0.0;      // Initial velocity of the tire. Applied to rim and nodes
double plate_lenght_z = 0.014;  // Thickness of the tire

// Specification of the mesh
const int numEl_Diameter = 120;  // Number of elements along diameter
const int numEl_Thread = 24;     // Number of elements along thread
const int numEl_z = 1;           // Single element along the thickness
const int N_Diameter = numEl_Diameter;
const int N_Thread = numEl_Thread + 1;
const int N_z = numEl_z + 1;
const double TorusSmallRadius = 0.195;
const double TorusRadius = 0.35;
const double Clearance = 0.00;  // Initial space between tire and ground
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

    // Compute Q=Q(x,v)
    // This is the function that you have to implement. It should return the
    // generalized Q load
    // (i.e.the force in generalized lagrangian coordinates).
    // Since here we have multiple connected ChLoadable objects (the two nodes),
    // the rule is that
    // all the vectors (load_Q, state_x, state_w) are split in the same order that
    // the loadable objects
    // are added to MyLoadCustomMultiple; in this case for instance
    // Q={Efx,Efy,Efz,Ffx,Ffy,Ffz}.
    // As this is a stiff force field, dependency from state_x and state_y must be
    // considered.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {
        std::vector<ChSharedPtr<ChLoadable>> NodeList;
        ChVector<> Node1_Pos;
        ChVector<> Node1_Vel;
        ChVector<> Node1_Grad;
        ChVector<> Node1_GradVel;
        this->load_Q.FillElem(0);
        const double KGround = 3e5;
        const double CGround = 8e2;
        double NormalForceNode = 0;
        double FrictionCoeff = 2.0;  // F1-like
        if (state_x && state_w) {
            for (unsigned int iii = 0; iii < TotalNumNodes; iii++) {
                Node1_Pos = state_x->ClipVector(iii * 6, 0);
                Node1_Grad = state_x->ClipVector(iii * 6 + 3, 0);
                Node1_Vel = state_w->ClipVector(iii * 6, 0);
                Node1_GradVel = state_w->ClipVector(iii * 6 + 3, 0);

                if (Node1_Pos.y < GroundLoc) {
                    NormalForceNode = KGround * abs(Node1_Pos.y - GroundLoc) - CGround * (Node1_Vel.y);
                    this->load_Q(iii * 6 + 1) = NormalForceNode;  // Fy (Vertical)
                    const double VelLimit = 0.0000;
                    // Calculation of friction forces
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
                Node1_Pos.SetNull();
            }
        } else {
            // explicit integrators might call ComputeQ(0,0), null pointers mean
            // that we assume current state, without passing state_x for efficiency
        }
    }

    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK,         ///< result dQ/dx
                                 ChMatrix<>& mR,         ///< result dQ/dv
                                 ChMatrix<>& mM)         ///< result dQ/da
    {
        double Delta = 1e-8;

        int mrows_w = this->LoadGet_ndof_w();

        // compute Q at current speed & position, x_0, v_0
        ChVectorDynamic<> Q0(mrows_w);
        this->ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
        Q0 = this->load_Q;

        ChVectorDynamic<> Q1(mrows_w);
        ChVectorDynamic<> Jcolumn(mrows_w);

        // Compute K=-dQ(x,v)/dx by backward differentiation
        for (int i = 0; i < mrows_w; ++i) {
            (*state_x)(i) += Delta;            //***TODO*** use NodeIntStateIncrement
            this->ComputeQ(state_x, state_w);  // Q1 = Q(x+Dx, v)
            Q1 = this->load_Q;
            (*state_x)(i) -= Delta;                //***TODO*** use NodeIntStateIncrement
            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
            this->jacobians->K.PasteMatrix(&Jcolumn, 0, i);
        }
        // Compute R=-dQ(x,v)/dv by backward differentiation
        for (int i = 0; i < mrows_w; ++i) {
            (*state_w)(i) += Delta;
            this->ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
            Q1 = this->load_Q;
            (*state_w)(i) -= Delta;

            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
            this->jacobians->R.PasteMatrix(&Jcolumn, 0, i);
        }
    };

    // OPTIONAL: if you want to provide an analytical jacobian, just implement the
    // following:
    //   virtual void ComputeJacobian(...)

    // Remember to set this as stiff, to enable the jacobians
    virtual bool IsStiff() { return false; }
};

int main(int argc, char* argv[]) {
    // Definition of the model
    ChSystem my_system;
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);
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

    int MaxMNUM = 0;
    int MTYPE = 0;
    int MaxLayNum = 0;
    ChMatrixDynamic<double> COORDFlex(TotalNumNodes, 6);
    ChMatrixDynamic<double> VELCYFlex(TotalNumNodes, 6);
    ChMatrixDynamic<int> NumNodes(TotalNumElements, 4);
    ChMatrixDynamic<int> LayNum(TotalNumElements, 1);
    ChMatrixDynamic<int> NDR(TotalNumNodes, 6);
    ChMatrixDynamic<double> ElemLengthXY(TotalNumElements, 2);
    ChMatrixNM<double, 10, 12> MPROP;
    ChMatrixNM<int, 10, 7> MNUM;
    ChMatrixNM<int, 10, 1> NumLayer;
    double LayPROP[10][7][2];

    //--------------- Element data--------------------
    for (int j = 0; j < N_Diameter; j++) {  // Start node numbering by zero
        for (int i = 0; i < numEl_Thread; i++) {
            LayNum(i + j * numEl_Thread, 0) = 1;  // Each element has one layer
            if (j == N_Diameter - 1) {
                NumNodes(i + j * numEl_Thread, 0) = i + j * (numEl_Thread + 1);
                NumNodes(i + j * numEl_Thread, 1) = i;
                NumNodes(i + j * numEl_Thread, 2) = i + 1;
                NumNodes(i + j * numEl_Thread, 3) = i + 1 + j * (numEl_Thread + 1);
            } else {
                NumNodes(i + j * numEl_Thread, 0) = i + j * (numEl_Thread + 1);
                NumNodes(i + j * numEl_Thread, 1) = i + (j + 1) * (numEl_Thread + 1);
                NumNodes(i + j * numEl_Thread, 2) = i + 1 + (j + 1) * (numEl_Thread + 1);
                NumNodes(i + j * numEl_Thread, 3) = i + 1 + j * (numEl_Thread + 1);
            }

            // Add shell dimensions based on coordinates
            ElemLengthXY(i + j * numEl_Thread, 0) = dx;  // dx
            ElemLengthXY(i + j * numEl_Thread, 1) = dy;  // dy

            if (MaxLayNum < LayNum(i, 0)) {
                MaxLayNum = LayNum(i, 0);
            }
        }
    }
    //  Fixing constraints, initial coordinates and velocities
    double thethaTorus = 0.0;
    double phiTorus = 0.0;

    for (int j = 0; j < N_Diameter; j++) {
        for (int i = 0; i < N_Thread; i++) {
            thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
            thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
            phiTorus = 2 * CH_C_PI * j / N_Diameter;

            COORDFlex(i + j * N_Thread, 2) = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus);
            COORDFlex(i + j * N_Thread, 1) = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
            COORDFlex(i + j * N_Thread, 0) = TorusSmallRadius * sin(thethaTorus);

            COORDFlex(i + j * N_Thread, 5) = cos(thethaTorus) * cos(phiTorus);
            COORDFlex(i + j * N_Thread, 4) = cos(thethaTorus) * sin(phiTorus);
            COORDFlex(i + j * N_Thread, 3) = sin(thethaTorus);

            // Write position and gradient of nodes to a file and plot them in Matlab
            out << i + j * N_Thread << COORDFlex(i + j * N_Thread, 0) << COORDFlex(i + j * N_Thread, 1)
                << COORDFlex(i + j * N_Thread, 2) << COORDFlex(i + j * N_Thread, 3) << COORDFlex(i + j * N_Thread, 4)
                << COORDFlex(i + j * N_Thread, 5) << std::endl;
            out.write_to_file("../TorusTire.txt");

            VELCYFlex(i + j * N_Thread, 0) = 0;
            VELCYFlex(i + j * N_Thread, 1) = 0;
            VELCYFlex(i + j * N_Thread, 2) = ForVel;
            VELCYFlex(i + j * N_Thread, 3) = 0;
            VELCYFlex(i + j * N_Thread, 4) = 0;
            VELCYFlex(i + j * N_Thread, 5) = 0;
        }
    }
    //------------- Read Layer Data-------------------
    for (int i = 0; i < MaxLayNum; i++) {
        NumLayer(i, 0) = i + 1;

        for (int j = 0; j < NumLayer(i, 0); j++) {
            LayPROP[i][j][0] = dz;  // Height of each layer
            if (j == 0) {
                LayPROP[i][j][1] = 0;  // For first layer, fiber angle 20 degrees
            }                          // Fiber angle of each ply
            else {
                LayPROP[i][j][1] = 0;  // For second layer, fiber angle -20 degrees
            }
            MNUM[i][j] = 1;  // Material_ID
            // In this example one single material ID (same material properties for the two layers)
            if (MaxMNUM < MNUM(i, j))
                MaxMNUM = MNUM(i, j);
        }
    }
    //------------ Input Material Data----------------
    for (int i = 0; i < MaxMNUM; i++) {
        double nu_coef = 0.3;
        MTYPE = 2;  // The user must use orthotropic input (MTYPE=2)
        if (MTYPE == 2) {
            MPROP(i, 0) = 500;                                    // Density [kg/m3]
            MPROP(i, 1) = 9.0E+07;                                // Ex//
            MPROP(i, 2) = 9.0E+07;                                // Ey
            MPROP(i, 3) = 9.0E+07;                                // Ez
            MPROP(i, 4) = 0.3;                                    // nuxy
            MPROP(i, 5) = 0.3;                                    // nuxz
            MPROP(i, 6) = 0.3;                                    // nuyz
            MPROP(i, 7) = MPROP(i, 1) / 2.0 / (1 + MPROP(i, 6));  // Gxy
            MPROP(i, 8) = MPROP(i, 2) / 2.0 / (1 + MPROP(i, 6));  // Gxz
            MPROP(i, 9) = MPROP(i, 3) / 2.0 / (1 + MPROP(i, 6));  // Gyz
        }
    }

    // Adding the nodes to the mesh
    int i = 0;

    while (i < TotalNumNodes) {
        ChSharedPtr<ChNodeFEAxyzD> node(
            new ChNodeFEAxyzD(ChVector<>(COORDFlex(i, 0), COORDFlex(i, 1), COORDFlex(i, 2)),
                              ChVector<>(COORDFlex(i, 3), COORDFlex(i, 4), COORDFlex(i, 5))));
        node.StaticCastTo<ChNodeFEAxyz>()->SetPos_dt(ChVector<>(VELCYFlex(i, 0), VELCYFlex(i, 1), VELCYFlex(i, 2)));
        node->SetMass(0.0);
        my_mesh->AddNode(node);
        i++;
    }

    int elemcount = 0;
    while (elemcount < TotalNumElements) {
        ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
        // Save material data into InertFlexVec(98x1) at each layer
        ChMatrixNM<double, 98, 1> InertFlexVec;
        InertFlexVec.Reset();
        double TotalThickness;  // Element thickness: Summation of all layers' thickness
        TotalThickness = 0.0;
        int i = elemcount;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {  // For each element, define material properties
            int ij = 14 * j;
            InertFlexVec(ij) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][0];  // Density
            InertFlexVec(ij + 1) = ElemLengthXY(i, 0);                   // EL
            InertFlexVec(ij + 2) = ElemLengthXY(i, 1);                   // EW
            InertFlexVec(ij + 3) = LayPROP[LayNum(i, 0) - 1][j][0];      // Thickness per layer
            TotalThickness += InertFlexVec(ij + 3);
            InertFlexVec(ij + 4) = LayPROP[LayNum(i, 0) - 1][j][1];           // Fiber angle
            InertFlexVec(ij + 5) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][1];   // Ex
            InertFlexVec(ij + 6) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][2];   // Ey
            InertFlexVec(ij + 7) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][3];   // Ez
            InertFlexVec(ij + 8) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][4];   // nuxy
            InertFlexVec(ij + 9) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][5];   // nuxz
            InertFlexVec(ij + 10) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][6];  // nuyz
            InertFlexVec(ij + 11) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][7];  // Gxy
            InertFlexVec(ij + 12) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][8];  // Gxz
            InertFlexVec(ij + 13) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][9];  // Gyz
        }
        // Automatically selects the Gauss range to integrate inertia/internal forces over the thickness
        // Each layer's thickness will be integrated using 2 Gauss integration points
        // LimLow and LimHigh represent the thickness range for a given layer
        ChMatrixNM<double, 7, 2> GaussZRange;
        GaussZRange.Reset();
        double CurrentHeight = 0.0;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {
            double LimLow = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            CurrentHeight += LayPROP[LayNum(i, 0) - 1][j][0];
            double LimHigh = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            GaussZRange(j, 0) = LimLow;
            GaussZRange(j, 1) = LimHigh;
        }
        // Now we give some parameters element by element
        element->SetInertFlexVec(InertFlexVec);
        element->SetGaussZRange(GaussZRange);
        element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][1]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][3]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][2]).DynamicCastTo<ChNodeFEAxyzD>());
        element->SetNumLayers(NumLayer(LayNum(i, 0) - 1, 0));
        element->SetThickness(TotalThickness);
        element->SetAlphaDamp(0.15);               // Structural damping for this
        element->Setdt(time_step);                 // dt to calculate DampingCoefficient
        element->SetGravityOn(true);               // turn gravity on/off
        ChMatrixNM<double, 35, 1> StockAlpha_EAS;  // StockAlpha(5*7,1): Max #Layer is 7
        StockAlpha_EAS.Reset();
        element->SetStockAlpha(StockAlpha_EAS);
        my_mesh->AddElement(element);
        elemcount++;
    }
    if (addSingleLoad) {
        ChSharedPtr<ChNodeFEAxyzD> nodetip(my_mesh->GetNode(TotalNumNodes - 1).DynamicCastTo<ChNodeFEAxyzD>());
        nodetip->SetForce(ChVector<>(0, 0, -10));
    }

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
        // Defining the Body 2: Rim
        Body_2 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Body_2);
        Body_2->SetIdentifier(2);
        Body_2->SetBodyFixed(false);
        Body_2->SetCollide(false);
        Body_2->SetMass(100);
        Body_2->SetInertiaXX(ChVector<>(1, 0.3, 0.3));
        Body_2->SetPos(ChVector<>(0, 0, 0));  // Y = -1m
        Body_2->SetRot(rot);
        // Create constrained rim: Will remain in a perpendicular plane to the ground
        // Defining the Body 3: Hub
        Body_3 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Body_3);
        Body_3->SetIdentifier(3);
        Body_3->SetBodyFixed(false);
        Body_3->SetCollide(false);
        Body_3->SetMass(0.5);
        Body_3->SetInertiaXX(ChVector<>(0.01, 0.01, 0.01));
        Body_3->SetPos(ChVector<>(0, 0.0, 0));  // Y = -1m
        Body_3->SetRot(rot);
        // Defining the Body 4: Suspended mass
        Body_4 = ChSharedPtr<ChBody>(new ChBody);
        my_system.AddBody(Body_4);
        Body_4->SetIdentifier(4);
        Body_4->SetBodyFixed(false);
        Body_4->SetCollide(false);
        Body_4->SetMass(1500);
        Body_4->SetInertiaXX(ChVector<>(10, 3, 3));
        Body_4->SetPos(ChVector<>(0, 0.4, 0));  // Y = -1m
        Body_4->SetRot(rot);

        // Create revolute joint between
        ChSharedPtr<ChLinkLockOldham> Body4Plane(new ChLinkLockOldham);
        my_system.AddLink(Body4Plane);
        Body4Plane->Initialize(Body_4, BGround, ChCoordsys<>(ChVector<>(0, 0.4, 0), Q_from_AngY(-CH_C_PI_2)));

        // Prismatic joint between hub and suspended mass
        ChSharedPtr<ChLinkLockPrismatic> prims(new ChLinkLockPrismatic);
        my_system.AddLink(prims);
        prims->Initialize(Body_3, Body_4, ChCoordsys<>(ChVector<>(0, 0.4, 0), Q_from_AngX(-CH_C_PI_2)));
        my_system.Set_G_acc(ChVector<>(0, -9.81, 0));  // Hey! 4G!

        // Revolute joint between hub and rim
        ChSharedPtr<ChLinkLockRevolute> Rev(new ChLinkLockRevolute);
        my_system.AddLink(Rev);
        Rev->Initialize(Body_2, Body_3, ChCoordsys<>(ChVector<>(0, 0.0, 0), Q_from_AngY(-CH_C_PI_2)));

        my_system.Set_G_acc(ChVector<>(0, -9.81, 0));  // Hey! 4G!
        const double spring_coef = 1e5;
        const double damping_coef = 1e3;

        // Spring and damper for secondary suspension
        ChSharedPtr<ChLinkSpring> spring = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
        spring->Initialize(Body_3, Body_4, false, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), true);
        spring->Set_SpringK(spring_coef);
        spring->Set_SpringR(damping_coef);
        my_system.AddLink(spring);
    }
    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    ChSharedPtr<ChLoadContainer> Mloadcontainer(new ChLoadContainer);
    // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
    if (addPressureAlessandro) {
        for (unsigned int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            ChSharedPtr<ChLoad<ChLoaderPressure>> faceload(
                new ChLoad<ChLoaderPressure>(my_mesh->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
            faceload->loader.SetPressure(320e3);
            faceload->loader.SetStiff(false);
            faceload->loader.SetIntegrationPoints(2);
            Mloadcontainer->Add(faceload);
        }
    }
    // MyPressureLoad allows adding user-defined pressure functions, which
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
        for (unsigned int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            ChSharedPtr<ChLoad<MyPressureLoad>> PressureElement(
                new ChLoad<MyPressureLoad>(my_mesh->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
            Mloadcontainer->Add(PressureElement);  // do not forget to add the load to the load container.
        }
    }
    // Add constraints to the rim: ChLinkPointFrame for position constraints,
    // ChLinkDirFrame for direction constraints
    if (addConstRim) {
        int NoConstNodes = 2 * numEl_Diameter;
        int ConstNodeInx = -1;

        for (int j = 0; j < N_Diameter; j++) {
            for (int i = 0; i < N_Thread; i++) {
                ConstNodeInx = i + j * N_Thread;  // General node index

                if ((i + j * N_Thread) % (N_Thread) == 0) {
                    // First node to be constrained (one side of the rim)
                    ConstrainedNode =
                        ChSharedPtr<ChNodeFEAxyzD>(my_mesh->GetNode(ConstNodeInx).DynamicCastTo<ChNodeFEAxyzD>());
                    NodePosRim = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                    NodePosRim->Initialize(ConstrainedNode, Body_2);
                    my_system.Add(NodePosRim);

                    NodeDirRim = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                    NodeDirRim->Initialize(ConstrainedNode, Body_2);
                    NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                    my_system.Add(NodeDirRim);

                    // Second node to be constrained (other side of the rim)

                    ConstrainedNode2 = ChSharedPtr<ChNodeFEAxyzD>(
                        my_mesh->GetNode(ConstNodeInx + N_Thread - 1).DynamicCastTo<ChNodeFEAxyzD>());

                    NodePosRim2 = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
                    NodePosRim2->Initialize(ConstrainedNode2, Body_2);
                    my_system.Add(NodePosRim2);

                    NodeDirRim2 = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
                    NodeDirRim2->Initialize(ConstrainedNode2, Body_2);
                    NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                    my_system.Add(NodeDirRim2);
                }
            }
        }

        if (addGroundForces) {
            // Select on which nodes we are going to apply a load
            // ChSharedPtr<ChNodeFEAxyzD> NodeLoad6(my_mesh->GetNode(6).DynamicCastTo<ChNodeFEAxyzD>());
            // ChSharedPtr<ChNodeFEAxyzD> NodeLoad7(my_mesh->GetNode(7).DynamicCastTo<ChNodeFEAxyzD>());
            std::vector<ChSharedPtr<ChLoadable>> NodeList;
            for (int iNode = 0; iNode < TotalNumNodes; iNode++) {
                ChSharedPtr<ChNodeFEAxyzD> NodeLoad(my_mesh->GetNode(iNode).DynamicCastTo<ChNodeFEAxyzD>());
                NodeList.push_back(NodeLoad);
            }
            // Instance load object. This require a list of ChLoadable objects
            // (these are our two nodes,pay attention to the sequence order), and add to
            // container.
            ChSharedPtr<MyLoadCustomMultiple> Mloadcustommultiple(new MyLoadCustomMultiple(NodeList));
            Mloadcontainer->Add(Mloadcustommultiple);
        }
    }
    my_system.Add(Mloadcontainer);
    // Switch off mesh class gravity: my_mesh still does not implement this element's gravity forces
    my_mesh->SetAutomaticGravity(false);
    // Remember to add the mesh to the system
    my_system.Add(my_mesh);
    // This is mandatory
    my_system.SetupInitial();
    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);

    // INT_HHT or INT_EULER_IMPLICIT
    my_system.SetIntegrationType(ChSystem::INT_HHT);

    ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
    mystepper->SetAlpha(-0.2);  // Important for convergence
    mystepper->SetMaxiters(5);
    mystepper->SetTolerance(5e-05);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);  //
    mystepper->SetVerbose(true);

    ChMatrixNM<double, 3, 1> Cp;
    ChMatrixNM<double, 2, 1> Cd;  // Matrices for storing constraint violations

    // Visualization
    Body_2->SetPos_dt(ChVector<>(0, 0, ForVel));
    ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
    mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    Body_2->AddAsset(mobjmesh);
    double start = std::clock();
    if (showVisual) {
        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.003);
        my_mesh->AddAsset(mvisualizemeshC);

        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshwire(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
        mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshwire->SetWireframe(true);
        my_mesh->AddAsset(mvisualizemeshwire);

        ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
        mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
        mvisualizemesh->SetColorscaleMinMax(0.0, 30);
        mvisualizemesh->SetSmoothFaces(true);
        my_mesh->AddAsset(mvisualizemesh);
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
        mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.5);
        mrigidBody->setMaterialTexture(0, cubeMap);

        my_system.Setup();
        my_system.Update();

        GetLog() << "\n\nREADME\n\n"
                 << " - Press SPACE to start dynamic simulation \n - Press F10 for nonlinear statics - Press F11 for "
                    "linear statics. \n";

        // at beginning, no analysis is running..
        application.SetPaused(true);
        int AccuNoIterations = 0;
        application.SetStepManage(true);
        application.SetTimestep(time_step);
        application.SetTryRealtime(true);
        double ChTime = 0.0;

        utils::CSV_writer out("\t");
        out.stream().setf(std::ios::scientific | std::ios::showpos);
        out.stream().precision(7);
        const double VerForce = 0;
        const double HorForce = 4000;
        const double tini = 0.1;
        const double tend = 0.2;
        const double interval = tend - tini;
        while (application.GetDevice()->run()) {
            Body_2->Empty_forces_accumulators();
            // Body_2->Set_Scr_force(const ChVector<>& mf) { Scr_force = mf; }
            if (my_system.GetChTime() >= tini && my_system.GetChTime() <= tend) {
                Body_2->Set_Scr_torque(ChVector<>(HorForce * (my_system.GetChTime() - tini) / interval, 0, 0));
            } else if (my_system.GetChTime() > tend) {
                Body_2->Set_Scr_torque(ChVector<>(HorForce, 0, 0));
            }
            application.BeginScene();
            application.DrawAll();
            application.DoStep();
            application.EndScene();
            if (!application.GetPaused()) {
                std::cout << "Time t = " << my_system.GetChTime() << "s \n";
                AccuNoIterations += mystepper->GetNumIterations();
                printf("Forward position of rim X:      %12.4e ", Body_2->coord.pos.x);
                out << my_system.GetChTime() << Body_2->GetPos().x << Body_2->GetPos().y << Body_2->GetPos().z
                    << Body_3->GetPos().x << Body_3->GetPos().y << Body_3->GetPos().z << Body_4->GetPos().x
                    << Body_4->GetPos().y << Body_4->GetPos().z << std::endl;
                out.write_to_file("../VertPosRim.txt");
            }
        }
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        chrono::GetLog() << "Computation Time: " << duration;
        system("pause");
    } else {
        for (unsigned int it = 0; it < num_steps; it++) {
            my_system.DoStepDynamics(time_step);
            std::cout << "Time t = " << my_system.GetChTime() << "s \n";
            // std::cout << "nodetip->pos.z = " << nodetip->pos.z << "\n";
            std::cout << "mystepper->GetNumIterations()= " << mystepper->GetNumIterations() << "\n";
            if (addConstRim) {
                Cp = NodePosRim->GetC();
                printf("Point constraint violations:      %12.4e  %12.4e  %12.4e\n", Cp.GetElement(0, 0),
                       Cp.GetElement(1, 0), Cp.GetElement(2, 0));
                Cd = NodeDirRim->GetC();
                printf("Direction constraint violations:  %12.4e  %12.4e\n", Cd.GetElement(0, 0), Cd.GetElement(1, 0));

                printf("Vertical position of the rim:  %12.4e m\n", Body_2->coord.pos.y);
            }
        }
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        chrono::GetLog() << "Computation Time: " << duration;
        system("pause");
    }

    return 0;
}
