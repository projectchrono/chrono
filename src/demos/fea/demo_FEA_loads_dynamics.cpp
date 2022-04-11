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
// Authors: Radu Serban
// =============================================================================
//
// FEA demo on applying loads to beams
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChLoadsBeam.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace fea;
using namespace chrono::irrlicht;

const std::string out_dir = GetChronoOutputPath() + "FEA_LOADS";  // Output directory

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the physical system
    ChSystemSMC sys;

    // Create a mesh
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Create some nodes (with default mass 0)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
    auto nodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(2, 0, 0)));
    nodeA->SetMass(0.0);
    nodeB->SetMass(0.0);
    mesh->AddNode(nodeA);
    mesh->AddNode(nodeB);

    // Create beam section & material
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    double beam_wy = 0.1;
    double beam_wz = 0.2;
    msection->SetAsRectangularSection(beam_wy, beam_wz);
    msection->SetYoungModulus(0.01e9);
    msection->SetGshearModulus(0.01e9 * 0.3);
    msection->SetBeamRaleyghDamping(0.200);
    msection->SetDensity(1500);

    // Create an Eulero-Bernoulli beam with a single element
    auto elementA = chrono_types::make_shared<ChElementBeamEuler>();
    elementA->SetNodes(nodeA, nodeB);
    elementA->SetSection(msection);
    mesh->AddElement(elementA);

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    sys.Add(ground);

    // Create a constraint at the end of the beam
    auto constrA = chrono_types::make_shared<ChLinkMateGeneric>();
    constrA->Initialize(nodeA, ground, false, nodeA->Frame(), nodeA->Frame());
    sys.Add(constrA);
    constrA->SetConstrainedCoords(true, true, true,   // x, y, z
                                  true, true, true);  // Rx, Ry, Rz

    // -----------------------------------------------------------------
    // Apply loads

    // Create the load container
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(loadcontainer);

    // Select applied loads:
    //    LOAD_1  : Vertical load on end node
    //    LOAD_2  : Distributed load along beam element
    //    LOAD_3  : Distributed volumetric load (gravity)
    //    LOAD_4  : Custom distributed load along beam element
    //    LOAD_5  : Custom load with stiff force, acting on a single node (VER 1)
    //    LOAD_6  : Custom load with stiff force, acting on a single node (VER 2)
////#define LOAD_1
////#define LOAD_2
////#define LOAD_3
////#define LOAD_4
#define LOAD_5
////#define LOAD_6

    GetLog() << "Appled loads: \n";

#ifdef LOAD_1
    // Example 1:
    // Add a vertical load to the end of the beam element

    GetLog() << "   Vertical load acting on end node.\n";

    auto mwrench = chrono_types::make_shared<ChLoadBeamWrench>(elementA);
    mwrench->loader.SetApplication(1.0);  // in -1..+1 range, -1: end A, 0: mid, +1: end B
    mwrench->loader.SetForce(ChVector<>(0, 200, 0));
    loadcontainer->Add(mwrench);  // do not forget to add the load to the load container.
#endif

#ifdef LOAD_2
    // Example 2:
    // Add a distributed load along the beam element:

    GetLog() << "   Distributed load along beam element.\n";

    auto mwrenchdis = chrono_types::make_shared<ChLoadBeamWrenchDistributed>(elementA);
    mwrenchdis->loader.SetForcePerUnit(ChVector<>(0, 400.0, 0));  // load per unit length
    loadcontainer->Add(mwrenchdis);
#endif

#ifdef LOAD_3
    // Example 3:
    // Add gravity (constant volumetric load)

    GetLog() << "   Gravitty load (constant volumetric load).\n";

    auto mgravity = chrono_types::make_shared<ChLoad<ChLoaderGravity>>(elementA);
    loadcontainer->Add(mgravity);

    // note that by default all solid elements in the mesh will already
    // get gravitational force, if you want to bypass this automatic gravity, do:
    mesh->SetAutomaticGravity(false);
#endif

#ifdef LOAD_4
    // Example 4:
    // Now, create a custom load for the beam element.
    // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h  headers,
    // from which you can inherit. Here we inherit from
    // For example, let's make a distributed triangular load. A load on the beam is a
    // wrench, i.e. force+load per unit lenght applied at a certain abscissa U, that is a six-dimensional load.
    // By the way, a triangular load will work as a constant one because a single Euler beam
    // cannot feel more than this.

    GetLog() << "   Custom distributed load along beam element.\n";

    class MyLoaderTriangular : public ChLoaderUdistributed {
      public:
        // Useful: a constructor that also sets ChLoadable
        MyLoaderTriangular(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUdistributed(mloadable){};

        // Compute F=F(u)
        // This is the function that you have to implement. It should return the
        // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(
            const double U,              ///< parametric coordinate in line
            ChVectorDynamic<>& F,        ///< Result F vector here, size must be = n.field coords.of loadable
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
        ) {
            double Fy_max = 1000;
            F.segment(0, 3) = ChVector<>(0, ((1 + U) / 2) * Fy_max, 0).eigen();  // load, force part
            F.segment(3, 3).setZero();                                           // load, torque part
        }

        // Needed because inheriting ChLoaderUdistributed. Use 1 because linear load fx.
        virtual int GetIntegrationPointsU() { return 1; }
    };

    // Create the load (and handle it with a shared pointer).
    // The ChLoad is a 'container' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<a_loader_class>()

    std::shared_ptr<ChLoad<MyLoaderTriangular>> mloadtri(new ChLoad<MyLoaderTriangular>(elementA));
    loadcontainer->Add(mloadtri);  // do not forget to add the load to the load container.
#endif

#ifdef LOAD_5
    // Example 5:
    // Create a custom load with stiff force, acting on a single node.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    GetLog() << "   Custom load with stiff force, acting on a single node (VER 1).\n";

    auto nodeC = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    mesh->AddNode(nodeC);

    class MyLoaderPointStiff : public ChLoaderUVWatomic {
      public:
        MyLoaderPointStiff(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoaderUVWatomic(mloadable, 0, 0, 0) {}

        // Compute F=F(u)
        // This is the function that you have to implement. It should return the F load at U,V,W.
        // For ChNodeFEAxyz, loads are expected as 3-rows vectors, containing F absolute force.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeF(
            const double U,
            const double V,
            const double W,
            ChVectorDynamic<>& F,        ///< Result F vector here, size must be = n.field coords.of loadable
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
        ) {
            ChVector<> node_pos;
            ChVector<> node_vel;
            if (state_x) {
                node_pos = state_x->segment(0, 3);
                node_vel = state_w->segment(0, 3);
            } else {
                node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos();
                node_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos_dt();
            }
            // Just implement a simple force+spring+damper in xy plane,
            // for spring&damper connected to absolute reference:
            double Kx = 100;
            double Ky = 400;
            double Dx = 0.6;
            double Dy = 0.9;
            double x_offset = 2;
            double y_offset = 5;
            double x_force = 50;
            double y_force = 0;

            // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
            F(0) = x_force - Kx * (node_pos.x() - x_offset) - Dx * node_vel.x();  // Fx component of force
            F(1) = y_force - Ky * (node_pos.y() - y_offset) - Dy * node_vel.y();  // Fy component of force
            F(2) = 0;                                                             // Fz component of force
        }

        // Remember to set this as stiff, to enable the jacobians
        virtual bool IsStiff() { return true; }
    };

    // Instance a ChLoad object, applying to a node, and passing a ChLoader as a template
    // (in this case the ChLoader-inherited class is our MyLoaderPointStiff), and add to container:
    std::shared_ptr<ChLoad<MyLoaderPointStiff>> mloadstiff(new ChLoad<MyLoaderPointStiff>(nodeC));
    loadcontainer->Add(mloadstiff);
#endif

#ifdef LOAD_6
    // Example 6:
    // As before, create a custom load with stiff force, acting on a single node, but
    // this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
    // This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    GetLog() << "   Custom load with stiff force, acting on a single node (VER 2).\n";

    auto nodeD = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    mesh->AddNode(nodeD);

    class MyLoadCustom : public ChLoadCustom {
      public:
        MyLoadCustom(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoadCustom(mloadable){};

        /// "Virtual" copy constructor (covariant return type).
        virtual MyLoadCustom* Clone() const override { return new MyLoadCustom(*this); }

        // Compute Q=Q(x,v)
        // This is the function that you have to implement. It should return the generalized Q load
        // (i.e.the force in generalized lagrangian coordinates).
        // For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                              ChStateDelta* state_w  ///< state speed to evaluate Q
                              ) override {
            ChVector<> node_pos;
            ChVector<> node_vel;
            if (state_x && state_w) {
                node_pos = state_x->segment(0, 3);
                node_vel = state_w->segment(0, 3);
            } else {
                node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos();
                node_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos_dt();
            }
            // Just implement a simple force+spring+damper in xy plane,
            // for spring&damper connected to absolute reference:
            double Kx = 100;
            double Ky = 400;
            double Dx = 0.6;
            double Dy = 0.9;
            double x_offset = 2;
            double y_offset = 5;
            double x_force = 50;
            double y_force = 0;

            // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
            this->load_Q(0) = x_force - Kx * (node_pos.x() - x_offset) - Dx * node_vel.x();
            this->load_Q(1) = y_force - Ky * (node_pos.y() - y_offset) - Dy * node_vel.y();
            this->load_Q(2) = 0;
        }

        // OPTIONAL: if you want to provide an analytical jacobian, that might avoid the lengthy and approximate
        // default numerical jacobian, just implement the following:
        virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                     ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                     ChMatrixRef mK,         ///< result dQ/dx
                                     ChMatrixRef mR,         ///< result dQ/dv
                                     ChMatrixRef mM          ///< result dQ/da
                                     ) override {
            mK(0, 0) = 100;
            mK(1, 1) = 400;
            mR(0, 0) = 0.6;
            mR(1, 1) = 0.9;
        }

        // Remember to set this as stiff, to enable the jacobians
        virtual bool IsStiff() override { return true; }
    };

    // Instance load object, applying to a node, as in previous example, and add to container:
    auto mloadcustom = chrono_types::make_shared<MyLoadCustom>(nodeD);
    loadcontainer->Add(mloadcustom);
#endif

#ifdef LOAD_7
    // Example 7:
    // As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
    // This time we will need the ChLoadCustomMultiple as base class.
    // Those nodes (ie.e ChLoadable objects) can be added in mesh in whatever order,
    // not necessarily contiguous, because the bookkeeping is automated.
    // Being a stiff load, a jacobian will be automatically generated
    // by default using numerical differentiation; but if you want you
    // can override ComputeJacobian() and compute mK, mR analytically - see prev.example.

    ////GetLog() << "   Custom load with stiff force, acting on multiple nodes.\n";

    auto nodeE = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    mesh->AddNode(nodeE);
    auto nodeF = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(2, 11, 3));
    mesh->AddNode(nodeF);

    class MyLoadCustomMultiple : public ChLoadCustomMultiple {
      public:
        MyLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& mloadables) : ChLoadCustomMultiple(mloadables){};

        /// "Virtual" copy constructor (covariant return type).
        virtual MyLoadCustomMultiple* Clone() const override { return new MyLoadCustomMultiple(*this); }

        // Compute Q=Q(x,v)
        // This is the function that you have to implement. It should return the generalized Q load
        // (i.e.the force in generalized lagrangian coordinates).
        // Since here we have multiple connected ChLoadable objects (the two nodes), the rule is that
        // all the vectors (load_Q, state_x, state_w) are split in the same order that the loadable objects
        // are added to MyLoadCustomMultiple; in this case for instance Q={Efx,Efy,Efz,Ffx,Ffy,Ffz}.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                              ChStateDelta* state_w  ///< state speed to evaluate Q
                              ) override {
            ChVector<> Enode_pos;
            ChVector<> Enode_vel;
            ChVector<> Fnode_pos;
            ChVector<> Fnode_vel;
            if (state_x && state_w) {
                Enode_pos = state_x->segment(0, 3);
                Enode_vel = state_w->segment(0, 3);
                Fnode_pos = state_x->segment(3, 3);
                Fnode_vel = state_w->segment(3, 3);
            } else {
                // explicit integrators might call ComputeQ(0,0), null pointers mean
                // that we assume current state, without passing state_x for efficiency
                Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPos();
                Enode_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPos_dt();
                Fnode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPos();
                Fnode_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPos_dt();
            }
            // Just implement two simple force+spring+dampers in xy plane:
            // ... from node E to ground,
            double Kx1 = 60;
            double Ky1 = 50;
            double Dx1 = 0.3;
            double Dy1 = 0.2;
            double E_x_offset = 2;
            double E_y_offset = 10;
            ChVector<> spring1(-Kx1 * (Enode_pos.x() - E_x_offset) - Dx1 * Enode_vel.x(),
                               -Ky1 * (Enode_pos.y() - E_y_offset) - Dy1 * Enode_vel.y(), 0);
            // ... from node F to node E,
            double Ky2 = 10;
            double Dy2 = 0.2;
            double EF_dist = 0.75;
            ChVector<> spring2(
                0, -Ky2 * (Fnode_pos.y() - Enode_pos.y() - EF_dist) - Dy2 * (Enode_vel.y() - Fnode_vel.y()), 0);
            double Fforcey = 2;

            // store generalized forces as a contiguous vector in this->load_Q, with same order of state_w
            this->load_Q(0) = spring1.x() - spring2.x();  // Fx component of force on 1st node
            this->load_Q(1) = spring1.y() - spring2.y();  // Fy component of force on 1st node
            this->load_Q(2) = spring1.z() - spring2.z();  // Fz component of force on 1st node
            this->load_Q(3) = spring2.x();                // Fx component of force on 2nd node
            this->load_Q(4) = spring2.y() + Fforcey;      // Fy component of force on 2nd node
            this->load_Q(5) = spring2.z();                // Fz component of force on 2nd node
        }

        // OPTIONAL: if you want to provide an analytical jacobian, just implement the following:
        //   virtual void ComputeJacobian(...)

        // Remember to set this as stiff, to enable the jacobians
        virtual bool IsStiff() override { return true; }
    };

    // Instance load object. This require a list of ChLoadable objects
    // (these are our two nodes, pay attention to the sequence order), and add to container.
    std::vector<std::shared_ptr<ChLoadable>> mnodelist;
    mnodelist.push_back(nodeE);
    mnodelist.push_back(nodeF);
    auto mloadcustommultiple = chrono_types::make_shared<MyLoadCustomMultiple>(mnodelist);
    ////loadcontainer->Add(mloadcustommultiple);
#endif

    // -----------------------------------------------------------------
    // Set visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-400, 200);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Loads on beams");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0.5, 0.0, -3.0), ChVector<>(0.5, 0.0, 0.0));
    sys.SetVisualSystem(vis);

    // -----------------------------------------------------------------

    // Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-15);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    sys.SetSolverForceTolerance(1e-13);

    // Set integrator type
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Change integrator type
    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
    ////    mystepper->SetAlpha(-0.2);
    ////    mystepper->SetMaxiters(6);
    ////    mystepper->SetAbsTolerances(1e-12);
    ////    mystepper->SetVerbose(false);
    ////    mystepper->SetStepControl(false);
    ////}

    // Simulation loop
    ChFunction_Recorder rec;

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        sys.DoStepDynamics(1e-3);

        // double time = sys.GetChTime();
        auto& posB = nodeB->GetPos();
        rec.AddPoint(posB.x(), posB.y());

        /*
        std::cout << "TIME=" << time << "  " << posB.x() << "  " << posB.y() << "  ";
#ifdef LOAD_5
        auto& posC = nodeC->GetPos();
        std::cout << posC.x() << "  " << posC.y() << "  ";
#endif
#ifdef LOAD_6
        auto& posD = nodeD->GetPos();
        std::cout << posD.x() << "  " << posD.y() << "  ";
#endif
#ifdef LOAD_7
        auto& posE = nodeE->GetPos();
        auto& posF = nodeF->GetPos();
        std::cout << posE.x() << "  " << posE.y() << "  " << posF.x() << "  " << posF.y() << "  ";
#endif
        std::cout << std::endl;
        */
    }

    std::string gplfilename = out_dir + "/beam_loads.gpl";
    postprocess::ChGnuPlot mplot(gplfilename.c_str());
    mplot.SetGrid(false, 1, ChColor(0.8f, 0.8f, 0.8f));
    mplot.SetLabelX("X");
    mplot.SetLabelY("Y");
    mplot.SetCommand("set title 'Free node trajectory'");
    mplot.Plot(rec, "", " with lines lt -1 lc rgb'#00AAEE' ");

    return 0;
}
