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

///////////////////////////////////////////////////
//
//   Demos code about 
//
//     - FEM (introduction to dynamics)
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
// 
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
     
// Include some headers used by this tutorial...

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChNodeBody.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementSpring.h"
#include "unit_FEM/ChElementBar.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChMesh.h"


// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;



void test_1()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: spring FEM dynamics,  implicit integration \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	
				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnodeA(ChVector<>(0,0,0));
	ChNodeFEMxyz mnodeB(ChVector<>(0,1,0));
				
				// Default mass for FEM nodes is zero, so set point-like 
				// masses because the ChElementSpring FEM element that we
				// are going to use won't add any mass:
	mnodeA.SetMass(100.0);
	mnodeB.SetMass(100.0);
	
				// For example, set an applied force to a node:
	mnodeB.SetForce(ChVector<>(0,5,0));

				// For example, set an initial displacement to a node:
	mnodeB.SetPos( mnodeB.GetX0() + ChVector<>(0,0.1,0) );


				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnodeA);
	my_mesh->AddNode(mnodeB);

				// Create some elements of 'spring-damper' type, each connecting
				// two 3D nodes:
	ChElementSpring melementA;
	melementA.SetNodes(&mnodeA, &mnodeB);
	melementA.SetSpringK(200);
	melementA.SetDamperR(4);

				// Remember to add elements to the mesh!
	my_mesh->AddElement(melementA);


				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);
			

				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	truss->SetBodyFixed(true);
	my_system.Add(truss);

				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraintA(new ChNodeBody);

	constraintA->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to
							
	my_system.Add(constraintA);
		
				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a dynamic time integration:

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	my_system.SetIterLCPmaxItersSpeed(40);
	my_system.SetTolSpeeds(1e-10);

	double timestep = 0.01;
	while (my_system.GetChTime() < 2)
	{
		my_system.DoStepDynamics(timestep);

		GetLog() << " t=" << my_system.GetChTime() << "  nodeB.pos.y=" << mnodeB.GetPos().y << "  \n";
	}

}




void test_2()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: bar FEM dynamics,  implicit integration \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	
				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnodeA(ChVector<>(0,0,0));
	ChNodeFEMxyz mnodeB(ChVector<>(0,1,0));

				// Set no point-like masses because mass is already in bar element:
	//mnodeA.SetMass(100.0);	
	//mnodeB.SetMass(100.0);
	
				// For example, set an applied force to a node:
	mnodeB.SetForce(ChVector<>(0,5,0));

				// For example, set an initial displacement to a node:
	mnodeB.SetPos( mnodeB.GetX0() + ChVector<>(0,0.1,0) );

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnodeA);
	my_mesh->AddNode(mnodeB);


				// Create some elements of 'bar' type, each connecting
				// two 3D nodes:
	ChElementBar melementA;
	melementA.SetNodes(&mnodeA, &mnodeB);
	melementA.SetBarArea(0.1*0.02);
	melementA.SetBarYoungModulus(0.01e9); // rubber 0.01e9, steel 200e9
	melementA.SetBarRaleyghDamping(0.0);
	melementA.SetBarDensity(2.*0.1/(melementA.GetBarArea()*1.0));
	//melementA.SetBarDensity(0);
	


				// Remember to add elements to the mesh!
	my_mesh->AddElement(melementA);

				// This is mandatory, to initialize rest lengths, masses, etc.
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);
			

				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	truss->SetBodyFixed(true);
	my_system.Add(truss);

				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraintA(new ChNodeBody);

	constraintA->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to
							
	my_system.Add(constraintA);
		
				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a dynamic time integration:

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(false);
	my_system.SetIterLCPmaxItersSpeed(100);
	my_system.SetTolSpeeds(1e-10);

	GetLog() << "K of bar = " << (melementA.GetBarArea() * melementA.GetBarYoungModulus())/melementA.GetRestLength() << "\n";

	double timestep = 0.001;
	while (my_system.GetChTime() < 0.2)
	{
		my_system.DoStepDynamics(timestep);

		GetLog() << " t=" << my_system.GetChTime() << "  nodeB.pos.y=" << mnodeB.GetPos().y << "  \n";
	}

	GetLog() << " Bar mass = " << melementA.GetMass() << "  restlength = " << melementA.GetRestLength() << "\n";

	GetLog() << "K of bar = " << (melementA.GetBarArea() * melementA.GetBarYoungModulus())/melementA.GetRestLength() << "\n";
	GetLog() << "kmatr = " << *melementA.Kstiffness().Get_K() ;

}




void test_3()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: tetahedron FEM dynamics, implicit integration \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	
				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(0.01e9); // rubber 0.01e9, steel 200e9
	mmaterial->Set_v(0.3);
	mmaterial->Set_RayleighDampingK(0.01);
	mmaterial->Set_density(1000);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnode1(ChVector<>(0,0,0));
	ChNodeFEMxyz mnode2(ChVector<>(0,0,1));
	ChNodeFEMxyz mnode3(ChVector<>(0,1,0));
	ChNodeFEMxyz mnode4(ChVector<>(1,0,0));

				// For example, set a point-like mass at a node:
	mnode3.SetMass(200);

				// For example, set an initial displacement to a node:
	mnode3.SetPos( mnode3.GetX0() + ChVector<>(0,0.01,0) );

	// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);

				// Create the tetrahedron element, and assign 
				// nodes and material
	ChElementTetra_4 melement1;
	melement1.SetNodes(&mnode1, &mnode2, &mnode3, &mnode4);
	melement1.SetMaterial(mmaterial);

				// Remember to add elements to the mesh!
	my_mesh->AddElement(melement1);

				// This is necessary in order to precompute the 
				// stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);


				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	truss->SetBodyFixed(true);
	my_system.Add(truss);
	
				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraint1(new ChNodeBody);
	ChSharedPtr<ChNodeBody> constraint2(new ChNodeBody);
	ChSharedPtr<ChNodeBody> constraint3(new ChNodeBody);

	constraint1->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to

	constraint2->Initialize(my_mesh,		// node container
							1,				// index of node in node container 
							truss);			// body to be connected to
							
	constraint3->Initialize(my_mesh,		// node container
							3,				// index of node in node container 
							truss);			// body to be connected to
					
	my_system.Add(constraint1);
	my_system.Add(constraint2);
	my_system.Add(constraint3);

				// Perform a dynamic time integration:

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	my_system.SetIterLCPmaxItersSpeed(40);
	my_system.SetTolSpeeds(1e-10);

	double timestep = 0.001;
	while (my_system.GetChTime() < 0.6)
	{
		my_system.DoStepDynamics(timestep);

		GetLog() << " t =" << my_system.GetChTime() << "  mnode3.pos.y=" << mnode3.GetPos().y << "  \n";
	}

}




// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();


	// Test: an introductory problem:
	test_2();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	system("pause");
	return 0;
}


