//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//   Demos code about
//
//     - how to apply loads to beams, shells, volumes, etc. 
//

// Include some headers used by this tutorial...

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementBar.h"
#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChElementBrick.h"
#include "chrono_fea/ChElementHexa_20.h"
#include "chrono_fea/ChElementHexa_8.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementTetra_10.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLoadsBeam.h"
#include "chrono_fea/ChMesh.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

void test_1() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: load applied to a beam                       \n\n";

    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh:
    auto my_mesh = std::make_shared<ChMesh>();
    my_system.Add(my_mesh);

    // Create some nodes.
    auto mnodeA = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
    auto mnodeB = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(2, 0, 0)));

    // Default mass for FEM nodes is zero
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);

    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create beam section & material
    auto msection = std::make_shared<ChBeamSectionAdvanced>();
	double beam_wy = 0.1;
	double beam_wz = 0.2;
	msection->SetAsRectangularSection(beam_wy, beam_wz);
	msection->SetYoungModulus (0.01e9);
	msection->SetGshearModulus(0.01e9 * 0.3);
	msection->SetBeamRaleyghDamping(0.200);
    msection->SetDensity(1500);

    // Create a beam of Eulero-Bernoulli type:
    auto melementA = std::make_shared<ChElementBeamEuler>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSection(msection);
    my_mesh->AddElement(melementA);


    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint at the end of the beam
    auto constr_a = std::make_shared<ChLinkMateGeneric>();
	constr_a->Initialize(  mnodeA,
							truss,
							false, 
							mnodeA->Frame(),
							mnodeA->Frame() 
							 );
	my_system.Add(constr_a);
	constr_a->SetConstrainedCoords( true, true, true,	  // x, y, z
									true, true, true);   // Rx, Ry, Rz

    // APPLY SOME LOADS!

    // First: loads must be added to "load containers", 
    // and load containers must be added to your ChSystem 
    auto mloadcontainer = std::make_shared<ChLoadContainer>();
    my_system.Add(mloadcontainer);


    // Example 1:
    
    // Add a vertical load to the end of the beam element:
    auto mwrench = std::make_shared<ChLoadBeamWrench>(melementA);
    mwrench->loader.SetApplication(1.0); // in -1..+1 range, -1: end A, 0: mid, +1: end B
    mwrench->loader.SetForce(ChVector<>(0,-0.2,0));
    mloadcontainer->Add(mwrench);  // do not forget to add the load to the load container.


    // Example 2:

    // Add a distributed load along the beam element:
    auto mwrenchdis = std::make_shared<ChLoadBeamWrenchDistributed>(melementA);
    mwrenchdis->loader.SetForcePerUnit(ChVector<>(0,-0.1,0)); // load per unit length
    mloadcontainer->Add(mwrenchdis);  


    // Example 3:

    // Add gravity (constant volumetric load)
    auto mgravity = std::make_shared<ChLoad<ChLoaderGravity>>(melementA);
    mloadcontainer->Add(mgravity);  

    // note that by default all solid elements in the mesh will already 
    // get gravitational force, if you want to bypass this automatic gravity, do:
    my_mesh->SetAutomaticGravity(false);


    // Example 4:

    // Now, create a custom load for the beam element.
    // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h  headers,
    // from which you can inherit. Here we inherit from 
    // For example, let's make a distributed triangular load. A load on the beam is a 
    // wrench, i.e. force+load per unit lenght aplied at a certain abscyssa U, that is a six-dimensional load. 
    // By the way, a triangular load will work as a constant one because a single Euler beam
    // cannot feel more than this. 

    class MyLoaderTriangular : public ChLoaderUdistributed {
    public:
            // Useful: a constructor that also sets ChLoadable    
            MyLoaderTriangular(std::shared_ptr<ChLoadableU> mloadable) :  ChLoaderUdistributed(mloadable) {};

            // Compute F=F(u)
            // This is the function that you have to implement. It should return the 
            // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
            // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ. 
            virtual void ComputeF(const double U,     ///< parametric coordinate in line
                          ChVectorDynamic<>& F,       ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
                double Fy_max = 0.005;
                F.PasteVector( ChVector<>(0, (((1+U)/2)*Fy_max),0) ,0,0); // load, force part; hardwired for brevity
                F.PasteVector( ChVector<>(0,0,0) ,3,0);   // load, torque part; hardwired for brevity
            }

            // Needed because inheriting ChLoaderUdistributed. Use 1 because linear load fx.
            virtual int GetIntegrationPointsU() {return 1;}
    };

    // Create the load (and handle it with a shared pointer).
    // The ChLoad is a 'container' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<a_loader_class>()

    std::shared_ptr< ChLoad<MyLoaderTriangular> > mloadtri (new ChLoad<MyLoaderTriangular>(melementA) );
    mloadcontainer->Add(mloadtri);  // do not forget to add the load to the load container.



    // Example 5:

    // Create a custom load with stiff force, acting on a single node.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    auto mnodeC = std::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    my_mesh->AddNode(mnodeC);

    class MyLoaderPointStiff : public ChLoaderUVWatomic {
    public:   
            MyLoaderPointStiff(std::shared_ptr<ChLoadableUVW> mloadable) :  ChLoaderUVWatomic(mloadable,0,0,0) {};

            // Compute F=F(u)
            // This is the function that you have to implement. It should return the F load at U,V,W. 
            // For ChNodeFEAxyz, loads are expected as 3-rows vectors, containing F absolute force.
            // As this is a stiff force field, dependency from state_x and state_y must be considered.
            virtual void ComputeF(const double U, const double V, const double W, 
                          ChVectorDynamic<>& F,       ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
                ChVector<> node_pos;
                ChVector<> node_vel;
                if (state_x) {
                    node_pos = state_x->ClipVector(0,0);
                    node_vel = state_w->ClipVector(0,0);
                }
                else {
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
                double y_offset = 10;
                double x_force = 50;
                double y_force = 0;
                
                // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
                F(0) = x_force -Kx*(node_pos.x() - x_offset) -Dx*node_vel.x(); // Fx component of force
                F(1) = y_force -Ky*(node_pos.y() - y_offset) -Dy*node_vel.y(); // Fy component of force
                F(2) = 0; // Fz component of force
            }

            // Remember to set this as stiff, to enable the jacobians
            virtual bool IsStiff() {return true;}
    };

    // Instance a ChLoad object, applying to a node, and passing a ChLoader as a template 
    // (in this case the ChLoader-inherited class is our MyLoaderPointStiff), and add to container:
    std::shared_ptr< ChLoad<MyLoaderPointStiff> > mloadstiff (new ChLoad<MyLoaderPointStiff>(mnodeC) );
    mloadcontainer->Add(mloadstiff);  


    
    // Example 6:

    // As before, create a custom load with stiff force, acting on a single node, but
    // this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
    // This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
    // As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
    // that will be used in statics, implicit integrators, etc.

    auto mnodeD = std::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    my_mesh->AddNode(mnodeD);

    class MyLoadCustom : public ChLoadCustom {
    public:   
            MyLoadCustom(std::shared_ptr<ChLoadableUVW> mloadable) :  ChLoadCustom(mloadable) {};

            // Compute Q=Q(x,v)
            // This is the function that you have to implement. It should return the generalized Q load 
            // (i.e.the force in generalized lagrangian coordinates).
            // For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
            // As this is a stiff force field, dependency from state_x and state_y must be considered.
            virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                                  ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) { 
                ChVector<> node_pos;
                ChVector<> node_vel;
                if (state_x && state_w) {
                    node_pos = state_x->ClipVector(0,0);
                    node_vel = state_w->ClipVector(0,0);
                }
                else {
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
                double y_offset = 10;
                double x_force = 50;
                double y_force = 0;
                
                // Store the computed generalized forces in this->load_Q, same x,y,z order as in state_w
                this->load_Q(0) = x_force -Kx*(node_pos.x() - x_offset) -Dx*node_vel.x(); 
                this->load_Q(1) = y_force -Ky*(node_pos.y() - y_offset) -Dy*node_vel.y(); 
                this->load_Q(2) = 0; 
            }

            // OPTIONAL: if you want to provide an analytical jacobian, that might avoid the lengthy and approximate
            // default numerical jacobian, just implement the following:
            virtual void ComputeJacobian(ChState*      state_x, ///< state position to evaluate jacobians
                                 ChStateDelta* state_w, ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK, ///< result dQ/dx
                                 ChMatrix<>& mR, ///< result dQ/dv
                                 ChMatrix<>& mM ///< result dQ/da   
                                               ) {
                mK(0,0)=100;
                mK(1,1)=400;
                mR(0,0)=0.6;
                mR(1,1)=0.9;
            }

            // Remember to set this as stiff, to enable the jacobians
            virtual bool IsStiff() {return true;}
    };

    // Instance load object, applying to a node, as in previous example, and add to container:
    auto mloadcustom = std::make_shared<MyLoadCustom>(mnodeD);
    mloadcontainer->Add(mloadcustom);  

  
    // Example 7:

    // As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
    // This time we will need the ChLoadCustomMultiple as base class.
    // Those nodes (ie.e ChLoadable objects) can be added in my_mesh in whatever order, 
    // not necessarily contiguous, because the bookkeeping is automated.
    // Being a stiff load, a jacobian will be automatically generated
    // by default using numerical differentiation; but if you want you 
    // can override ComputeJacobian() and compute mK, mR analytically - see prev.example.

    auto mnodeE = std::make_shared<ChNodeFEAxyz>(ChVector<>(2, 10, 3));
    my_mesh->AddNode(mnodeE);
    auto mnodeF = std::make_shared<ChNodeFEAxyz>(ChVector<>(2, 11, 3));
    my_mesh->AddNode(mnodeF);

    class MyLoadCustomMultiple : public ChLoadCustomMultiple {
    public:   
            MyLoadCustomMultiple(std::vector< std::shared_ptr<ChLoadable> >& mloadables) :  ChLoadCustomMultiple(mloadables) {};

            // Compute Q=Q(x,v)
            // This is the function that you have to implement. It should return the generalized Q load 
            // (i.e.the force in generalized lagrangian coordinates).
            // Since here we have multiple connected ChLoadable objects (the two nodes), the rule is that
            // all the vectors (load_Q, state_x, state_w) are split in the same order that the loadable objects
            // are added to MyLoadCustomMultiple; in this case for instance Q={Efx,Efy,Efz,Ffx,Ffy,Ffz}.
            // As this is a stiff force field, dependency from state_x and state_y must be considered.
            virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                                  ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) { 
                ChVector<> Enode_pos;
                ChVector<> Enode_vel;
                ChVector<> Fnode_pos;
                ChVector<> Fnode_vel;
                if (state_x && state_w) {
                    Enode_pos = state_x->ClipVector(0,0);
                    Enode_vel = state_w->ClipVector(0,0);
                    Fnode_pos = state_x->ClipVector(3,0);
                    Fnode_vel = state_w->ClipVector(3,0);
                }
                else { 
                    // explicit integrators might call ComputeQ(0,0), null pointers mean
                    // that we assume current state, without passing state_x for efficiency
                    Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPos();
                    Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[0])->GetPos_dt();
                    Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPos();
                    Enode_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(loadables[1])->GetPos_dt();
                }
                // Just implement two simple force+spring+dampers in xy plane:
                    // ... from node E to ground, 
                double Kx1 = 60; 
                double Ky1 = 50;
                double Dx1 = 0.3;
                double Dy1 = 0.2;
                double E_x_offset = 2;
                double E_y_offset = 10;
                ChVector<> spring1 (-Kx1*(Enode_pos.x() - E_x_offset) -Dx1*Enode_vel.x(), 
                                    -Ky1*(Enode_pos.y() - E_y_offset) -Dy1*Enode_vel.y(), 
                                    0);
                    // ... from node F to node E, 
                double Ky2 = 10;
                double Dy2 = 0.2;
                double EF_dist = 1;
                ChVector<> spring2 (0, 
                                    -Ky2*(Fnode_pos.y() - Enode_pos.y() - EF_dist) -Dy2*(Enode_vel.y() - Fnode_vel.y()), 
                                    0);
                double Fforcey = 2;
                // store generalized forces as a contiguous vector in this->load_Q, with same order of state_w
                this->load_Q(0) = spring1.x() - spring2.x(); // Fx component of force on 1st node
                this->load_Q(1) = spring1.y() - spring2.y(); // Fy component of force on 1st node
                this->load_Q(2) = spring1.z() - spring2.z(); // Fz component of force on 1st node
                this->load_Q(3) = spring2.x()         ; // Fx component of force on 2nd node
                this->load_Q(4) = spring2.y() +Fforcey; // Fy component of force on 2nd node
                this->load_Q(5) = spring2.z()         ; // Fz component of force on 2nd node
            }

            // OPTIONAL: if you want to provide an analytical jacobian, just implement the following:
            //   virtual void ComputeJacobian(...)

            // Remember to set this as stiff, to enable the jacobians
            virtual bool IsStiff() {return true;}
    };

    // Instance load object. This require a list of ChLoadable objects
    // (these are our two nodes,pay attention to the sequence order), and add to container.
    std::vector< std::shared_ptr< ChLoadable > > mnodelist;
    mnodelist.push_back(mnodeE);
    mnodelist.push_back(mnodeF);
    auto mloadcustommultiple = std::make_shared<MyLoadCustomMultiple>(mnodelist);
    mloadcontainer->Add(mloadcustommultiple);  

  

    ///////////////////////////////////////

    // Mark completion of system construction
    my_system.SetupInitial();

    // Setup a MINRES solver. For FEA one cannot use the default SOR type solver.

    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);
    my_system.SetMaxItersSolverSpeed(100);
    my_system.SetMaxItersSolverStab(100);
    my_system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);

    // Perform a static analysis:
    my_system.DoStaticLinear();

    GetLog() << " constr_a reaction force  F= " << constr_a->Get_react_force() <<  "  \n";
    GetLog() << " constr_a reaction torque T= " << constr_a->Get_react_torque() <<  "  \n";

    GetLog() << " mnodeC position = "     << mnodeC->GetPos() <<  "  \n";
    GetLog() << " mloadstiff K jacobian=" << mloadstiff->GetJacobians()->K <<"\n";

    GetLog() << " mnodeD position = "     << mnodeD->GetPos() <<  "  \n";
    GetLog() << " mloadcustom K jacobian="<< mloadcustom->GetJacobians()->K <<"\n";

    GetLog() << " mnodeE position = "     << mnodeE->GetPos() <<  "  \n";
    GetLog() << " mnodeF position = "     << mnodeF->GetPos() <<  "  \n";
    GetLog() << " mloadcustommultiple K jacobian="<< mloadcustommultiple->GetJacobians()->K <<"\n";
}



int main(int argc, char* argv[]) {

    test_1();
	
    return 0;
}
