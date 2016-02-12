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
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::scene;

bool addConstRim = true;
bool addBodies = true;
bool addGroundForces = true;
bool showVisual = true;
bool addSingleLoad = false;
bool addPressure = false;
bool addPressureAlessandro = true;

std::shared_ptr<ChBody> BGround;
std::shared_ptr<ChBody> Body_2;  // Rim (unsuspended)
std::shared_ptr<ChBody> Body_3;  // Hub
std::shared_ptr<ChBody> Body_4;  // Suspended
std::shared_ptr<ChLinkPointFrame> NodePosRim;
std::shared_ptr<ChLinkDirFrame> NodeDirRim;
std::shared_ptr<ChNodeFEAxyzD> ConstrainedNode;
std::shared_ptr<ChLinkPointFrame> NodePosRim2;
std::shared_ptr<ChLinkDirFrame> NodeDirRim2;
std::shared_ptr<ChNodeFEAxyzD> ConstrainedNode2;

// Some model parameters

const int num_steps = 550;  // Number of time steps for unit test (range 1 to 4000) 550
double time_step = 0.001;   // Time step: 0.001 It works like that, but rotates at the beginning

const double ForVel = 0.0;      // Initial velocity of the tire. Applied to rim and nodes
double plate_lenght_z = 0.014;  // Thickness of the tire

// Specification of the mesh
const int numEl_Diameter = 60;  // Number of elements along diameter
const int numEl_Thread = 12;     // Number of elements along thread
const int numEl_z = 1;          // Single element along the thickness
const int N_Diameter = numEl_Diameter;
const int N_Thread = numEl_Thread + 1;
const int N_z = numEl_z + 1;
const double TorusSmallRadius = 0.195;
const double TorusRadius = 0.35;
const double Clearance = 0.0002;  // Initial space between tire and ground
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
    MyLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable> >& mloadables) : ChLoadCustomMultiple(mloadables){};

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
        std::vector<std::shared_ptr<ChLoadable> > NodeList;
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
            for (int iii = 0; iii < TotalNumNodes; iii++) {
                Node1_Pos = state_x->ClipVector(iii * 6, 0);
                Node1_Grad = state_x->ClipVector(iii * 6 + 3, 0);
                Node1_Vel = state_w->ClipVector(iii * 6, 0);
                Node1_GradVel = state_w->ClipVector(iii * 6 + 3, 0);

                if (Node1_Pos.y < GroundLoc) {
                    NormalForceNode = KGround * abs(Node1_Pos.y - GroundLoc);
                    this->load_Q(iii * 6 + 1) = NormalForceNode;  // Fy (Vertical)
                    const double VelLimit = 0.5;
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
    auto my_mesh = std::make_shared<ChMesh>();
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

    //  Fixing constraints, initial coordinates and velocities
    double thethaTorus = 0.0;
    double phiTorus = 0.0;

    for (int j = 0; j < N_Diameter; j++) {
        for (int i = 0; i < N_Thread; i++) {
            thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
            thethaTorus = -CH_C_PI / 2 + CH_C_PI * i / (N_Thread - 1);
            phiTorus = 2 * CH_C_PI * j / N_Diameter;

            double loc_x = TorusSmallRadius * sin(thethaTorus);
            double loc_y = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * sin(phiTorus);
            double loc_z = (TorusRadius + TorusSmallRadius * cos(thethaTorus)) * cos(phiTorus);

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
            auto node =
                std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));
            // Apply initial velocities
            std::static_pointer_cast<ChNodeFEAxyz>(node)->SetPos_dt(ChVector<>(vel_x, vel_y, vel_z));
            node->SetMass(0);
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }
    // Create an isotropic material.
    // All elements share the same material.
    auto mat = std::make_shared<ChMaterialShellANCF>(500, 9.0e7, 0.3);
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
            auto element = std::make_shared<ChElementShellANCF>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)));
            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(0.15);  // Structural damping for this element
            element->SetGravityOn(true);  // no gravitational forces

            // Add element to mesh
            my_mesh->AddElement(element);
        }
    }

    // Switch off mesh class gravity
    my_mesh->SetAutomaticGravity(false);

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // Mark completion of system construction
    my_system.SetupInitial();

    if (addSingleLoad) {
        auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));
        nodetip->SetForce(ChVector<>(0, 0, -10));
    }

    if (addBodies) {
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
        // Defining the Body 2: Rim
        Body_2 = std::make_shared<ChBody>();
        my_system.AddBody(Body_2);
        Body_2->SetIdentifier(2);
        Body_2->SetBodyFixed(false);
        Body_2->SetCollide(false);
        Body_2->SetMass(100);
        Body_2->SetInertiaXX(ChVector<>(1, 0.3, 0.3));
        Body_2->SetPos(ChVector<>(0, 0, 0));  // Y = -1m
        Body_2->SetRot(rot);
        Body_2->SetPos_dt(ChVector<>(0, 0, ForVel));
        // Create constrained rim: Will remain in a perpendicular plane to the ground
        // Defining the Body 3: Hub
        Body_3 = std::make_shared<ChBody>();
        my_system.AddBody(Body_3);
        Body_3->SetIdentifier(3);
        Body_3->SetBodyFixed(false);
        Body_3->SetCollide(false);
        Body_3->SetMass(0.5);
        Body_3->SetInertiaXX(ChVector<>(0.01, 0.01, 0.01));
        Body_3->SetPos(ChVector<>(0, 0.0, 0));  // Y = -1m
        Body_3->SetRot(rot);
        Body_3->SetPos_dt(ChVector<>(0, 0, ForVel));
        // Defining the Body 4: Suspended mass
        Body_4 = std::make_shared<ChBody>();
        my_system.AddBody(Body_4);
        Body_4->SetIdentifier(4);
        Body_4->SetBodyFixed(false);
        Body_4->SetCollide(false);
        Body_4->SetMass(1500);
        Body_4->SetInertiaXX(ChVector<>(10, 3, 3));
        Body_4->SetPos(ChVector<>(0, 0.4, 0));  // Y = -1m
        Body_4->SetRot(rot);
        Body_4->SetPos_dt(ChVector<>(0, 0, ForVel));
        // Create revolute joint between
        auto Body4Plane = std::make_shared<ChLinkLockOldham>();
        my_system.AddLink(Body4Plane);
        Body4Plane->Initialize(Body_4, BGround, ChCoordsys<>(ChVector<>(0, 0.4, 0), Q_from_AngY(-CH_C_PI_2)));

        // Prismatic joint between hub and suspended mass
        auto prims = std::make_shared<ChLinkLockPrismatic>();
        my_system.AddLink(prims);
        prims->Initialize(Body_3, Body_4, ChCoordsys<>(ChVector<>(0, 0.4, 0), Q_from_AngX(-CH_C_PI_2)));
        my_system.Set_G_acc(ChVector<>(0, -9.81, 0));  // Hey! 4G!

        // Revolute joint between hub and rim
        auto Rev = std::make_shared<ChLinkLockRevolute>();
        my_system.AddLink(Rev);
        Rev->Initialize(Body_2, Body_3, ChCoordsys<>(ChVector<>(0, 0.0, 0), Q_from_AngY(-CH_C_PI_2)));

        my_system.Set_G_acc(ChVector<>(0, -9.81, 0));  // Hey! 4G!
        const double spring_coef = 1e5;
        const double damping_coef = 1e3;

        // Spring and damper for secondary suspension
        auto spring = std::make_shared<ChLinkSpring>();
        spring->Initialize(Body_3, Body_4, false, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), true);
        spring->Set_SpringK(spring_coef);
        spring->Set_SpringR(damping_coef);
        my_system.AddLink(spring);
    }
    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    auto Mloadcontainer = std::make_shared<ChLoadContainer>();
    // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
    if (addPressureAlessandro) {
        for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            auto faceload = std::make_shared<ChLoad<ChLoaderPressure>>(
                std::static_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(NoElmPre)));
            ;
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
            std::shared_ptr<ChElementShellANCF> m_element;

          public:
            // Useful: a constructor that also sets ChLoadable

            MyPressureLoad(std::shared_ptr<ChLoadableUV> element) : ChLoaderUVdistributed(element) {
                m_element = std::static_pointer_cast<ChElementShellANCF>(element);
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
        for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            auto PressureElement = std::make_shared<ChLoad<MyPressureLoad>>(
                std::static_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(NoElmPre)));
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
                    ConstrainedNode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ConstNodeInx));
                    NodePosRim = std::make_shared<ChLinkPointFrame>();
                    NodePosRim->Initialize(ConstrainedNode, Body_2);
                    my_system.Add(NodePosRim);

                    NodeDirRim = std::make_shared<ChLinkDirFrame>();
                    NodeDirRim->Initialize(ConstrainedNode, Body_2);
                    NodeDirRim->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
                    my_system.Add(NodeDirRim);

                    // Second node to be constrained (other side of the rim)
                    ConstrainedNode2 =
                        std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ConstNodeInx + N_Thread - 1));
                    NodePosRim2 = std::make_shared<ChLinkPointFrame>();
                    NodePosRim2->Initialize(ConstrainedNode2, Body_2);
                    my_system.Add(NodePosRim2);

                    NodeDirRim2 = std::make_shared<ChLinkDirFrame>();
                    NodeDirRim2->Initialize(ConstrainedNode2, Body_2);
                    NodeDirRim2->SetDirectionInAbsoluteCoords(ConstrainedNode2->D);
                    my_system.Add(NodeDirRim2);
                }
            }
        }

        if (addGroundForces) {
            std::vector<std::shared_ptr<ChLoadable>> NodeList;
            for (int iNode = 0; iNode < TotalNumNodes; iNode++) {
                auto NodeLoad =
                    std::shared_ptr<ChNodeFEAxyzD>(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(iNode)));
                NodeList.push_back(NodeLoad);
            }
            // Instance load object. This require a list of ChLoadable objects
            // (these are our two nodes,pay attention to the sequence order), and add to
            // container.
            auto Mloadcustommultiple = std::make_shared<MyLoadCustomMultiple>(NodeList);
            Mloadcontainer->Add(Mloadcustommultiple);
        }
    }
    my_system.Add(Mloadcontainer);
    // Switch off mesh class gravity: my_mesh still does not implement this element's gravity forces
    my_mesh->SetAutomaticGravity(false);

    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);

    // INT_HHT or INT_EULER_IMPLICIT
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);  // Important for convergence
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(5e-05, 5e-03);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);  //
    mystepper->SetVerbose(true);

    ChMatrixNM<double, 3, 1> Cp;
    ChMatrixNM<double, 2, 1> Cd;  // Matrices for storing constraint violations

    double start = std::clock();
    if (showVisual) {


        // Visualization
        auto mobjmesh = std::make_shared<ChObjShapeFile>();
        mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj").c_str());
        Body_2->AddAsset(mobjmesh);

        // Create a floor body
        auto mrigidBody = std::make_shared<ChBodyEasyBox>(10, 0.2, 10, 1000, true, true);
        my_system.Add(mrigidBody);
        mrigidBody->SetBodyFixed(true);
        mrigidBody->SetPos(ChVector<>(0, GroundLoc - 0.1, 0));
        mrigidBody->GetMaterialSurface()->SetFriction(0.5);
        auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg").c_str());
        mrigidBody->AddAsset(mtexture);


        auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.003);
        my_mesh->AddAsset(mvisualizemeshC);

        auto mvisualizemeshwire = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshwire->SetWireframe(true);
        my_mesh->AddAsset(mvisualizemeshwire);

        auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
        mvisualizemesh->SetColorscaleMinMax(0.0, 30);
        mvisualizemesh->SetSmoothFaces(true);
        my_mesh->AddAsset(mvisualizemesh);
        application.AssetBindAll();
        application.AssetUpdateAll();
    }

    my_system.Setup();
    my_system.Update();

    // This is mandatory
    my_system.SetupInitial();

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

    const double VerForce = 0;
    const double HorForce = 4000;
    const double tini = 0.1;
    const double tend = 0.2;
    const double interval = tend - tini;
    double duration = 0.0;
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
            printf("Vertical position of rim:      %12.4e \n", Body_2->GetPos().y);
            out << my_system.GetChTime() << Body_2->GetPos().x << Body_2->GetPos().y << Body_2->GetPos().z
                << Body_3->GetPos().x << Body_3->GetPos().y << Body_3->GetPos().z << Body_4->GetPos().x
                << Body_4->GetPos().y << Body_4->GetPos().z << std::endl;
            out.write_to_file("../VertPosRim.txt");
        }
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    }
    chrono::GetLog() << "Computation Time: " << duration << "\n";
    return 0;
}
