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
#include "chrono/utils/ChUtils.h"

#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChLoadsBeam.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------------------------------------------------------
    // Select load type
    // -----------------------------------------------------------------

    std::cout << "Options:" << std::endl;
    std::cout << "1  : Vertical load on end node" << std::endl;
    std::cout << "2  : Distributed load along beam element" << std::endl;
    std::cout << "3  : Distributed volumetric load (gravity)" << std::endl;
    std::cout << "4  : Custom distributed load along beam element" << std::endl;
    std::cout << "5  : Custom load with stiff force, acting on a single node (VER 1)" << std::endl;
    std::cout << "6  : Custom load with stiff force, acting on a single node (VER 2)" << std::endl;
    std::cout << "7  : Custom load with stiff force, acting on multiple nodes" << std::endl;
    std::cout << "\nSelect option (1-7): ";

    int load_option = 1;
    std::cin >> load_option;
    std::cout << std::endl;
    ChClampValue(load_option, 1, 7);

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "FEA_LOADS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // -----------------------------------------------------------------
    // Create the physical system
    // -----------------------------------------------------------------

    ChSystemSMC sys;

    // Create a mesh
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Create some nodes (with default mass 0)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, 0)));
    auto nodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(2, 0, 0)));
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

    // Create an Euler-Bernoulli beam with a single element
    auto elementA = chrono_types::make_shared<ChElementBeamEuler>();
    elementA->SetNodes(nodeA, nodeB);
    elementA->SetSection(beam_section);
    mesh->AddElement(elementA);

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Create a constraint at the end of the beam
    auto constrA = chrono_types::make_shared<ChLinkMateFix>();
    constrA->Initialize(nodeA, ground, false, nodeA->Frame(), nodeA->Frame());
    sys.Add(constrA);

    // -----------------------------------------------------------------
    // Apply loads
    // -----------------------------------------------------------------

    // Create the load container
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    switch (load_option) {
        case 1: {
            // Add a vertical load to the end of the beam element
            std::cout << "   Vertical load acting on end node.\n" << std::endl;

            auto wrench_load = chrono_types::make_shared<ChLoadBeamWrench>(elementA);
            wrench_load->GetLoader()->SetApplication(1.0);  // in -1..+1 range, -1: end A, 0: mid, +1: end B
            wrench_load->GetLoader()->SetForce(ChVector3d(0, 200, 0));
            load_container->Add(wrench_load);

            break;
        }  // end option 1

        case 2: {
            // Add a distributed load along the beam element:
            std::cout << "   Distributed load along beam element.\n" << std::endl;

            auto distr_wrench_load = chrono_types::make_shared<ChLoadBeamWrenchDistributed>(elementA);
            distr_wrench_load->GetLoader()->SetForcePerUnit(ChVector3d(0, 400.0, 0));  // load per unit length
            load_container->Add(distr_wrench_load);

            break;
        }  // end option 2

        case 3: {
            // Add gravity (constant volumetric load)
            std::cout << "   Gravity load (constant volumetric load).\n" << std::endl;

            auto gravity_loader = chrono_types::make_shared<ChLoaderGravity>(elementA);
            auto gravity_load = chrono_types::make_shared<ChLoad>(gravity_loader);
            load_container->Add(gravity_load);

            // By default, all solid elements in the mesh will already get gravitational force.
            // To bypass this automatic gravity, set:
            mesh->SetAutomaticGravity(false);

            break;
        }  // end option 3

        case 4: {
            // Now, create a custom load for the beam element.
            // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h  headers,
            // from which you can inherit. Here we inherit from
            // For example, let's make a distributed triangular load. A load on the beam is a
            // wrench, i.e. force+load per unit lenght applied at a certain abscissa U, that is a six-dimensional load.
            // By the way, a triangular load will work as a constant one because a single Euler beam
            // cannot feel more than this.
            std::cout << "   Custom distributed load along beam element.\n" << std::endl;

            class MyLoaderTriangular : public ChLoaderUdistributed {
              public:
                // Useful: a constructor that also sets ChLoadable
                MyLoaderTriangular(std::shared_ptr<ChLoadableU> loadable) : ChLoaderUdistributed(loadable){};

                // Compute F=F(u)
                // This is the function that you have to implement. It should return the
                // load at U. For Euler beams, loads are expected as 6-rows vectors, containing
                // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
                virtual void ComputeF(
                    double U,                    // parametric coordinate in line
                    ChVectorDynamic<>& F,        // result vector, size = field dim of loadable
                    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate F
                    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate F
                ) {
                    double Fy_max = 1000;
                    F.segment(0, 3) = ChVector3d(0, ((1 + U) / 2) * Fy_max, 0).eigen();  // load, force part
                    F.segment(3, 3).setZero();                                           // load, torque part
                }

                // Needed because inheriting ChLoaderUdistributed. Use 1 because linear load fx.
                virtual int GetIntegrationPointsU() { return 1; }
            };

            // Create the load
            auto tri_loader = chrono_types::make_shared<MyLoaderTriangular>(elementA);
            auto tri_load = chrono_types::make_shared<ChLoad>(tri_loader);
            load_container->Add(tri_load);

            break;
        }  // end option 4

        case 5: {
            // Create a custom load with stiff force, acting on a single node.
            // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
            // that will be used in statics, implicit integrators, etc.
            std::cout << "Custom load with stiff force, acting on a single node (VER 1).\n" << std::endl;

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
                    double y_offset = 5;
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

            break;
        }  // end option 5

        case 6: {
            // As before, create a custom load with stiff force, acting on a single node, but
            // this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
            // This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
            // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
            // that will be used in statics, implicit integrators, etc.
            std::cout << "Custom load with stiff force, acting on a single node (VER 2).\n" << std::endl;

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
                    // for spring & damper connected to absolute reference
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

                // To provide an analytical Jacobian, implement the following:
                virtual void ComputeJacobian(ChState* state_x, ChStateDelta* state_w) override {
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

            break;
        }  // end option 6

        case 7: {
            // As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
            // This time we will need the ChLoadCustomMultiple as base class.
            // Those nodes (ie.e ChLoadable objects) can be added in mesh in whatever order,
            // not necessarily contiguous, because the bookkeeping is automated.
            // Being a stiff load, a jacobian will be automatically generated
            // by default using numerical differentiation; but if you want you
            // can override ComputeJacobian() and compute mK, mR analytically - see prev.example.
            std::cout << "   Custom load with stiff force, acting on multiple nodes.\n" << std::endl;

            auto nodeE = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 10, 3));
            mesh->AddNode(nodeE);
            auto nodeF = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(2, 11, 3));
            mesh->AddNode(nodeF);

            class MyLoadCustomMultiple : public ChLoadCustomMultiple {
              public:
                MyLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& loadables)
                    : ChLoadCustomMultiple(loadables) {}

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
                    double EF_dist = 0.75;
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
            // (these are our two nodes, pay attention to the sequence order), and add to container.
            std::vector<std::shared_ptr<ChLoadable>> node_list;
            node_list.push_back(nodeE);
            node_list.push_back(nodeF);

            auto custom_load = chrono_types::make_shared<MyLoadCustomMultiple>(node_list);
            ////load_container->Add(custom_load);

            break;
        }  // end option 7

    }  // end switch(load_option)

    // -----------------------------------------------------------------
    // Set visualization of the FEM mesh.
    // -----------------------------------------------------------------

    auto beam_visA = chrono_types::make_shared<ChVisualShapeFEA>();
    beam_visA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    beam_visA->SetColormapRange(-400, 200);
    beam_visA->SetSmoothFaces(true);
    beam_visA->SetWireframe(false);
    mesh->AddVisualShapeFEA(beam_visA);

    auto beam_visB = chrono_types::make_shared<ChVisualShapeFEA>();
    beam_visB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    beam_visB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    beam_visB->SetSymbolsThickness(0.006);
    beam_visB->SetSymbolsScale(0.01);
    beam_visB->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(beam_visB);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Loads on beams",
                                         ChVector3d(0.5, 0.0, -3.0), ChVector3d(0.5, 0.0, 0.0));

    // -----------------------------------------------------------------

    // Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-15);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    // Set integrator type
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Change integrator type
    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
    ////    mystepper->SetAlpha(-0.2);
    ////    mystepper->SetMaxIters(6);
    ////    mystepper->SetAbsTolerances(1e-12);
    ////    mystepper->SetVerbose(false);
    ////    mystepper->SetStepControl(false);
    ////}

    // Simulation loop
    ChFunctionInterp rec;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
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
    postprocess::ChGnuPlot gnu_plot(gplfilename);
    gnu_plot.SetGrid(false, 1, ChColor(0.8f, 0.8f, 0.8f));
    gnu_plot.SetLabelX("X");
    gnu_plot.SetLabelY("Y");
    gnu_plot.SetCommand("set title 'Free node trajectory'");
    gnu_plot.Plot(rec, "", " with lines lt -1 lc rgb'#00AAEE' ");

    return 0;
}
