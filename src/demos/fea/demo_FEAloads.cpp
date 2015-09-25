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

    // Now, create an atomic load for the beam.
    // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h  headers,
    // from which you can inherit.
    // For example, an atomic load on the beam is a wrench, i.e. force+load aplied at 
    // a certain abscyssa U, that is a six-dimensional load. 
    // It is not a distributed load, so inherit it from ChLoaderUatomic:

    class MyLoaderWrench : public ChLoaderUatomic {
    public:
            // Useful: a constructor that also sets ChLoadable    
            MyLoaderWrench(ChSharedPtr<ChLoadableU> mloadable) :  ChLoaderUatomic(mloadable) {};

            // Compute F=F(u)
            // This is the function that you have to implement. It should return the 
            // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
            // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ. 
            virtual void ComputeF(const double U,     ///< parametric coordinate in line
                          ChVectorDynamic<>& F,       ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
            F.PasteVector( ChVector<>(0,-2,0) ,0,0); // load, force part; hardwired for brevity
            F.PasteVector( ChVector<>(0,0,0) ,3,0);   // load, torque part; hardwired for brevity
        }
    };

    // Create the load (and handle it with a shared pointer).
    // The ChLoad is a 'container' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<a_loader_class>()

    ChSharedPtr< ChLoad<MyLoaderWrench> > mloadA (new ChLoad<MyLoaderWrench>(melementA) );
    mloadA->loader.SetApplication(0.0) ; // this ChLoaderUatomic method sets the U abscyssa of load application (-1..+1, so 0=in the middle).
    mloadcontainer->Add(mloadA);  // do not forget to add the load to the load container.


    

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
