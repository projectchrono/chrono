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

#include "chrono/physics/ChSystem.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "physics/ChLinkMate.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementBrick.h"
#include "chrono_fea/ChElementBar.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementTetra_10.h"
#include "chrono_fea/ChElementHexa_8.h"
#include "chrono_fea/ChElementHexa_20.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLoadsBeam.h"
#include "physics/ChLoadContainer.h"

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
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);
    my_system.Add(my_mesh);

    // Create some nodes. 
    ChSharedPtr<ChNodeFEAxyzrot> mnodeA(new ChNodeFEAxyzrot( ChFrame<>(ChVector<>(0, 0, 0)) ));
    ChSharedPtr<ChNodeFEAxyzrot> mnodeB(new ChNodeFEAxyzrot( ChFrame<>(ChVector<>(2, 0, 0)) ));

    // Default mass for FEM nodes is zero
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);

    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create beam section & material
    ChSharedPtr<ChBeamSectionAdvanced> msection(new ChBeamSectionAdvanced);
	double beam_wy = 0.1;
	double beam_wz = 0.2;
	msection->SetAsRectangularSection(beam_wy, beam_wz);
	msection->SetYoungModulus (0.01e9);
	msection->SetGshearModulus(0.01e9 * 0.3);
	msection->SetBeamRaleyghDamping(0.200);
    msection->SetDensity(1500);

    // Create a beam of Eulero-Bernoulli type:
    ChSharedPtr<ChElementBeamEuler> melementA(new ChElementBeamEuler);
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSection(msection);
    my_mesh->AddElement(melementA);


    // Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint at the end of the beam
    ChSharedPtr<ChLinkMateGeneric> constr_a(new ChLinkMateGeneric);
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
    ChSharedPtr< ChLoadContainer > mloadcontainer(new ChLoadContainer);
    my_system.Add(mloadcontainer);


    // Example 1:
    
    // Add a vertical load to the end of the beam element:
    ChSharedPtr<ChLoadBeamWrench> mwrench(new ChLoadBeamWrench(melementA));
    mwrench->loader.SetApplication(1.0); // in -1..+1 range, -1: end A, 0: mid, +1: end B
    mwrench->loader.SetForce(ChVector<>(0,-0.2,0));
    mloadcontainer->Add(mwrench);  // do not forget to add the load to the load container.


    // Example 2:

    // Add a distributed load along the beam element:
    ChSharedPtr<ChLoadBeamWrenchDistributed> mwrenchdis(new ChLoadBeamWrenchDistributed(melementA));
    mwrenchdis->loader.SetForcePerUnit(ChVector<>(0,-0.1,0)); // load per unit length
    mloadcontainer->Add(mwrenchdis);  


    // Example 3:

    // Add gravity (constant volumetric load)
    ChSharedPtr< ChLoad<ChLoaderGravity> > mgravity(new ChLoad<ChLoaderGravity>(melementA));
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
            MyLoaderTriangular(ChSharedPtr<ChLoadableU> mloadable) :  ChLoaderUdistributed(mloadable) {};

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

    ChSharedPtr< ChLoad<MyLoaderTriangular> > mloadtri (new ChLoad<MyLoaderTriangular>(melementA) );
    mloadcontainer->Add(mloadtri);  // do not forget to add the load to the load container.




    // This is mandatory !
    my_mesh->SetupInitial();


    // Setup a MINRES solver. For FEA one cannot use the default SOR type solver.

    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED THIS OR ::LCP_SIMPLEX because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(460);
	my_system.SetIterLCPmaxItersStab(460);
	my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);

    // Perform a static analysis:
    my_system.DoStaticLinear();

    GetLog() << " reaction force  F= " << constr_a->Get_react_force() <<  "  \n";
    GetLog() << " reaction torque T= " << constr_a->Get_react_torque() <<  "  \n";


}



int main(int argc, char* argv[]) {

    test_1();
	
    system("pause");
    return 0;
}
