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
// FEA demo on applying loads to beams, shells, volumes, etc.
//
// =============================================================================

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementHexaANCF_3813.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLoadsBeam.h"
#include "chrono/fea/ChMesh.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono belong to this namespace and its children...

using namespace chrono;
using namespace fea;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: load applied to a beam\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh:
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Create some nodes.
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, 0)));
    auto nodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(2, 0, 0)));

    // Default mass for FEM nodes is zero
    nodeA->SetMass(0.0);
    nodeB->SetMass(0.0);

    mesh->AddNode(nodeA);
    mesh->AddNode(nodeB);

    // Create beam section & material
    auto beam_section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    double beam_wy = 0.1;
    double beam_wz = 0.2;
    beam_section->SetAsRectangularSection(beam_wy, beam_wz);
    beam_section->SetYoungModulus(0.01e9);
    beam_section->SetShearModulus(0.01e9 * 0.3);
    beam_section->SetRayleighDamping(0.200);
    beam_section->SetDensity(1500);

    // Create a beam of Euler-Bernoulli type:
    auto elementA = chrono_types::make_shared<ChElementBeamEuler>();
    elementA->SetNodes(nodeA, nodeB);
    elementA->SetSection(beam_section);
    mesh->AddElement(elementA);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint at the end of the beam
    auto constraintA = chrono_types::make_shared<ChLinkMateFix>();
    constraintA->Initialize(nodeA, truss, false, nodeA->Frame(), nodeA->Frame());
    sys.Add(constraintA);

    // APPLY SOME LOADS!

    // First: loads must be added to "load containers",
    // and load containers must be added to your system
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    // Example 1:

    // Add a vertical load to the end of the beam element:
    auto wrench_load = chrono_types::make_shared<ChLoadBeamWrench>(elementA);
    wrench_load->GetLoader()->SetApplication(1.0);  // in -1..+1 range, -1: end A, 0: mid, +1: end B
    wrench_load->GetLoader()->SetForce(ChVector3d(0, -0.2, 0));
    load_container->Add(wrench_load);  // do not forget to add the load to the load container.

    // Example 2:

    // Add a distributed load along the beam element:
    auto distr_wrench_load = chrono_types::make_shared<ChLoadBeamWrenchDistributed>(elementA);
    distr_wrench_load->GetLoader()->SetForcePerUnit(ChVector3d(0, -0.1, 0));  // load per unit length
    load_container->Add(distr_wrench_load);

    // Example 3:

    // Add gravity (constant volumetric load)
    auto gravity_loader = chrono_types::make_shared<ChLoaderGravity>(elementA);
    auto gravity_load = chrono_types::make_shared<ChLoad>(gravity_loader);
    load_container->Add(gravity_load);

    // note that by default all solid elements in the mesh will already
    // get gravitational force, if you want to bypass this automatic gravity, do:
    mesh->SetAutomaticGravity(false);

    // Example 4:

    // Now, create a custom load for the beam element.
    // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h  headers,
    // from which you can inherit. Here we inherit from
    // For example, let's make a distributed triangular load. A load on the beam is a
    // wrench, i.e. force+load per unit lenght applied at a certain abscissa U, that is a six-dimensional load.
    // By the way, a triangular load will work as a constant one because a single Euler beam
    // cannot feel more than this.

    class MyLoaderTriangular : public ChLoaderUdistributed {
      public:
        // Useful: a constructor that also sets ChLoadable
        MyLoaderTriangular(std::shared_ptr<ChLoadableU> loadable) : ChLoaderUdistributed(loadable) {}

        // Compute F=F(u)
        // This is the function that you have to implement. It should return the
        // load at U. For Euler beams, loads are expected as 6-rows vectors, containing
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(double U,                    // parametric coordinate in line
                              ChVectorDynamic<>& F,        // result vector, size = field dim of loadable
                              ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate F
                              ChVectorDynamic<>* state_w  // if != 0, update state (speed part) to this, then evaluate F
        ) {
            double Fy_max = 0.005;
            F.segment(0, 3) = ChVector3d(0, ((1 + U) / 2) * Fy_max, 0).eigen();  // load, force part
            F.segment(3, 3).setZero();                                           // load, torque part
        }

        // Needed because inheriting ChLoaderUdistributed. Use 1 because linear load fx.
        virtual int GetIntegrationPointsU() { return 1; }
    };

    // Create the load (and handle it with a shared pointer).
    // The ChLoad is a 'container' for your ChLoader.

    auto tri_loader = chrono_types::make_shared<MyLoaderTriangular>(elementA);
    auto tri_load = chrono_types::make_shared<ChLoad>(tri_loader);
    load_container->Add(tri_load);  // do not forget to add the load to the load container.

    // Example 5:

    // Create a custom load with stiff force, acting on a single node.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    auto nodeC = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 10, 3));
    mesh->AddNode(nodeC);

    class MyLoaderPointStiff : public ChLoaderUVWatomic {
      public:
        MyLoaderPointStiff(std::shared_ptr<ChLoadableUVW> loadable) : ChLoaderUVWatomic(loadable, 0, 0, 0) {}

        // Compute F=F(u)
        // This is the function that you have to implement. It should return the F load at U,V,W.
        // For ChNodeFEAxyz, loads are expected as 3-rows vectors, containing F absolute force.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeF(
             double U,
             double V,
             double W,
            ChVectorDynamic<>& F,        // result vector, size = field dim of loadable
            ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate F
        ) {
            ChVector3d node_pos;
            ChVector3d node_vel;
            if (state_x) {
                node_pos = state_x->segment(0, 3);
                node_vel = state_w->segment(0, 3);
            } else {
                node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos();
                node_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPosDt();
            }
            // Just implement a simple force+spring+damper in xy plane,
            // for spring&damper connected to absolute reference:
            double Kx = 100;
            double Ky = 400;
            double Dx = 0.6;
            double Dy = 0.9;
            double x_offset = 2;
            double y_offset = 10;
            double x_force = 50;
            double y_force = 0;

            // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
            F(0) = x_force - Kx * (node_pos.x() - x_offset) - Dx * node_vel.x();  // Fx component of force
            F(1) = y_force - Ky * (node_pos.y() - y_offset) - Dy * node_vel.y();  // Fy component of force
            F(2) = 0;                                                             // Fz component of force
        }

        // Set this as stiff, to enable the Jacobians
        virtual bool IsStiff() { return true; }
    };

    // Instance a ChLoad object, applying to a node, and passing a ChLoader as a template
    // (in this case the ChLoader-inherited class is our MyLoaderPointStiff), and add to container:
    auto stiff_loader = chrono_types::make_shared<MyLoaderPointStiff>(nodeC);
    auto stiff_load = chrono_types::make_shared<ChLoad>(stiff_loader);
    load_container->Add(stiff_load);

    // Example 6:

    // As before, create a custom load with stiff force, acting on a single node, but
    // this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
    // This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    auto nodeD = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 10, 3));
    mesh->AddNode(nodeD);

    class MyLoadCustom : public ChLoadCustom {
      public:
        MyLoadCustom(std::shared_ptr<ChLoadableUVW> loadable) : ChLoadCustom(loadable) {}

        // "Virtual" copy constructor (covariant return type).
        virtual MyLoadCustom* Clone() const override { return new MyLoadCustom(*this); }

        // Compute Q=Q(x,v)
        // This is the function that you have to implement. It should return the generalized Q load
        // (i.e.the force in generalized lagrangian coordinates).
        // For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeQ(ChState* state_x,      // state position to evaluate Q
                              ChStateDelta* state_w  // state speed to evaluate Q
                              ) override {
            ChVector3d node_pos;
            ChVector3d node_vel;
            if (state_x && state_w) {
                node_pos = state_x->segment(0, 3);
                node_vel = state_w->segment(0, 3);
            } else {
                node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPos();
                node_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadable)->GetPosDt();
            }
            // Just implement a simple force+spring+damper in xy plane,
            // for spring&damper connected to absolute reference:
            double Kx = 100;
            double Ky = 400;
            double Dx = 0.6;
            double Dy = 0.9;
            double x_offset = 2;
            double y_offset = 10;
            double x_force = 50;
            double y_force = 0;

            // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
            this->load_Q(0) = x_force - Kx * (node_pos.x() - x_offset) - Dx * node_vel.x();
            this->load_Q(1) = y_force - Ky * (node_pos.y() - y_offset) - Dy * node_vel.y();
            this->load_Q(2) = 0;
        }

        // OPTIONAL: if you want to provide an analytical jacobian, that might avoid the lengthy and approximate
        // default numerical jacobian, just implement the following:
        virtual void ComputeJacobian(ChState* state_x,      // state position to evaluate jacobians
                                     ChStateDelta* state_w  // state speed to evaluate jacobians
                                     ) override {
            m_jacobians->K(0, 0) = 100;
            m_jacobians->K(1, 1) = 400;
            m_jacobians->R(0, 0) = 0.6;
            m_jacobians->R(1, 1) = 0.9;
        }

        // Set this as stiff, to enable the Jacobians
        virtual bool IsStiff() override { return true; }
    };

    // Instance load object, applying to a node, as in previous example, and add to container:
    auto custom_load = chrono_types::make_shared<MyLoadCustom>(nodeD);
    load_container->Add(custom_load);

    // Example 7:

    // As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
    // This time we will need the ChLoadCustomMultiple as base class.
    // Those nodes (ie.e ChLoadable objects) can be added in mesh in whatever order,
    // not necessarily contiguous, because the bookkeeping is automated.
    // Being a stiff load, a jacobian will be automatically generated
    // by default using numerical differentiation; but if you want you
    // can override ComputeJacobian() and compute mK, mR analytically - see prev.example.

    auto nodeE = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 10, 3));
    mesh->AddNode(nodeE);
    auto nodeF = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 11, 3));
    mesh->AddNode(nodeF);

    class MyLoadCustomMultiple : public ChLoadCustomMultiple {
      public:
        MyLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& loadables) : ChLoadCustomMultiple(loadables) {}

        // "Virtual" copy constructor (covariant return type).
        virtual MyLoadCustomMultiple* Clone() const override { return new MyLoadCustomMultiple(*this); }

        // Compute Q=Q(x,v)
        // This is the function that you have to implement. It should return the generalized Q load
        // (i.e.the force in generalized lagrangian coordinates).
        // Since here we have multiple connected ChLoadable objects (the two nodes), the rule is that
        // all the vectors (load_Q, state_x, state_w) are split in the same order that the loadable objects
        // are added to MyLoadCustomMultiple; in this case for instance Q={Efx,Efy,Efz,Ffx,Ffy,Ffz}.
        // As this is a stiff force field, dependency from state_x and state_y must be considered.
        virtual void ComputeQ(ChState* state_x,      // state position to evaluate Q
                              ChStateDelta* state_w  // state speed to evaluate Q
                              ) override {
            ChVector3d Enode_pos;
            ChVector3d Enode_vel;
            ChVector3d Fnode_pos;
            ChVector3d Fnode_vel;
            if (state_x && state_w) {
                Enode_pos = state_x->segment(0, 3);
                Enode_vel = state_w->segment(0, 3);
                Fnode_pos = state_x->segment(3, 3);
                Fnode_vel = state_w->segment(3, 3);
            } else {
                // explicit integrators might call ComputeQ(0,0), null pointers mean
                // that we assume current state, without passing state_x for efficiency
                Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPos();
                Enode_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPosDt();
                Fnode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPos();
                Fnode_vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPosDt();
            }
            // Just implement two simple force+spring+dampers in xy plane:
            // ... from node E to ground,
            double Kx1 = 60;
            double Ky1 = 50;
            double Dx1 = 0.3;
            double Dy1 = 0.2;
            double E_x_offset = 2;
            double E_y_offset = 10;
            ChVector3d spring1(-Kx1 * (Enode_pos.x() - E_x_offset) - Dx1 * Enode_vel.x(),
                               -Ky1 * (Enode_pos.y() - E_y_offset) - Dy1 * Enode_vel.y(), 0);
            // ... from node F to node E,
            double Ky2 = 10;
            double Dy2 = 0.2;
            double EF_dist = 1;
            ChVector3d spring2(
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

        // Set this as stiff, to enable the Jacobians
        virtual bool IsStiff() override { return true; }
    };

    // Instance load object. This require a list of ChLoadable objects
    // (these are our two nodes,pay attention to the sequence order), and add to container.
    std::vector<std::shared_ptr<ChLoadable>> node_list;
    node_list.push_back(nodeE);
    node_list.push_back(nodeF);

    auto custom_multi_load = chrono_types::make_shared<MyLoadCustomMultiple>(node_list);
    load_container->Add(custom_multi_load);

    // ----------------------------------

    // Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    // Perform a static analysis:
    sys.DoStaticLinear();

    auto reaction = constraintA->GetReaction2();

    std::cout << " constraintA reaction force  F= " << reaction.force << std::endl;
    std::cout << " constraintA reaction torque T= " << reaction.torque << std::endl;

    std::cout << " nodeC position = " << nodeC->GetPos() << std::endl;
    std::cout << " stiff_load K jacobian=" << stiff_load->GetJacobians()->K << std::endl;

    std::cout << " nodeD position = " << nodeD->GetPos() << std::endl;
    std::cout << " custom_load K jacobian=" << custom_load->GetJacobians()->K << std::endl;

    std::cout << " nodeE position = " << nodeE->GetPos() << std::endl;
    std::cout << " nodeF position = " << nodeF->GetPos() << std::endl;
    std::cout << " custom_multi_load K jacobian=" << custom_multi_load->GetJacobians()->K << std::endl;

    return 0;
}
