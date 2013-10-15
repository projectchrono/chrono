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
//     - FEM (basic introduction)
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
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChMesh.h"


// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;



//////////////////////////////////////////
// ====================================	//
// Test 1								//
// First example: SPRING ELEMENT		//
// ==================================== //
void test_1()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: spring element FEM  \n\n";

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
	mnodeA.SetMass(0.0);
	mnodeB.SetMass(0.0);
	
				// For example, set an applied force to a node:
	mnodeB.SetForce(ChVector<>(0,5,0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnodeA);
	my_mesh->AddNode(mnodeB);

				// Create some elements of 'spring-damper' type, each connecting
				// two 3D nodes:
	ChElementSpring melementA;
	melementA.SetNodes(&mnodeA, &mnodeB);
	melementA.SetSpringK(100000);

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


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(40);
	my_system.SetTolSpeeds(1e-10);

	my_system.DoStaticLinear();

				// Output result
	GetLog() << "poss after linear static analysis: \n";
	GetLog() << "  nodeA.pos \n" << mnodeA.GetPos();
	GetLog() << "  nodeB.pos \n" << mnodeB.GetPos();
	GetLog() << "Forces after linear static analysis: \n";
	GetLog() << "  constraintA.react \n" << constraintA->GetReactionOnBody();
}


//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 2													    //
// Second example: LINEAR TETRAHEDRAL ELEMENT				    //
// ============================================================ //
void test_2()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: LINEAR tetrahedral element FEM  \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);

				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(209e9);
	mmaterial->Set_v(0.3);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnode1(ChVector<>(0,0,0));
	ChNodeFEMxyz mnode2(ChVector<>(0,0,1));
	ChNodeFEMxyz mnode3(ChVector<>(0,1,0));
	ChNodeFEMxyz mnode4(ChVector<>(1,0,0));
	mnode1.SetMass(0.001);
	mnode2.SetMass(0.001);
	mnode3.SetMass(0.001);
	mnode4.SetMass(0.001);
	
				// For example, set an applied force to a node:
	mnode3.SetForce(ChVector<>(0, 10000,0));

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

	GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
	GetLog()<<melement1.GetMatrB()<<"\n";

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

				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(80);
	my_system.SetTolSpeeds(1e-10);

	my_system.DoStaticLinear();

				// Output result
	GetLog()<<mnode1.pos<<"\n";
	GetLog()<<mnode2.pos<<"\n";
	GetLog()<<mnode3.pos<<"\n";
	GetLog()<<mnode4.pos<<"\n";

}


//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 3													    //
// Second example: QUADRATIC TETRAHEDRAL ELEMENT				//
// ============================================================ //
void test_3()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: QUADRATIC tetrahedral element FEM  \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);

				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(207e9);
	mmaterial->Set_v(0.3);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnode1(ChVector<>(0,0,0));
	ChNodeFEMxyz mnode2(ChVector<>(0.001,0,0));
	ChNodeFEMxyz mnode3(ChVector<>(0,0.001,0));
	ChNodeFEMxyz mnode4(ChVector<>(0,0,0.001));
				// Build a Constant Metric tetrahedron
	ChNodeFEMxyz mnode5(ChVector<>((mnode1.pos.x+mnode2.pos.x)/2,(mnode1.pos.y+mnode2.pos.y)/2,(mnode1.pos.z+mnode2.pos.z)/2));
	ChNodeFEMxyz mnode6(ChVector<>((mnode2.pos.x+mnode3.pos.x)/2,(mnode2.pos.y+mnode3.pos.y)/2,(mnode2.pos.z+mnode3.pos.z)/2));
	ChNodeFEMxyz mnode7(ChVector<>((mnode3.pos.x+mnode1.pos.x)/2,(mnode3.pos.y+mnode1.pos.y)/2,(mnode3.pos.z+mnode1.pos.z)/2));
	ChNodeFEMxyz mnode8(ChVector<>((mnode1.pos.x+mnode4.pos.x)/2,(mnode1.pos.y+mnode4.pos.y)/2,(mnode1.pos.z+mnode4.pos.z)/2));
	ChNodeFEMxyz mnode9(ChVector<>((mnode4.pos.x+mnode2.pos.x)/2,(mnode4.pos.y+mnode2.pos.y)/2,(mnode4.pos.z+mnode2.pos.z)/2));
	ChNodeFEMxyz mnode10(ChVector<>((mnode3.pos.x+mnode4.pos.x)/2,(mnode3.pos.y+mnode4.pos.y)/2,(mnode3.pos.z+mnode4.pos.z)/2));
	mnode1.SetMass(0.001);
	mnode2.SetMass(0.001);
	mnode3.SetMass(0.001);
	mnode4.SetMass(0.001);
	mnode5.SetMass(0.001);
	mnode6.SetMass(0.001);
	mnode7.SetMass(0.001);
	mnode8.SetMass(0.001);
	mnode9.SetMass(0.001);
	mnode10.SetMass(0.001);
	
				// For example, set an applied force to a node:
	mnode3.SetForce(ChVector<>(0, -1000, 0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);
	my_mesh->AddNode(mnode5);
	my_mesh->AddNode(mnode6);
	my_mesh->AddNode(mnode7);
	my_mesh->AddNode(mnode8);
	my_mesh->AddNode(mnode9);
	my_mesh->AddNode(mnode10);

				// Create the tetrahedron element, and assign 
				// it nodes and material
	ChElementTetra_10 melement1;
	melement1.SetNodes(&mnode1, &mnode2, &mnode3, &mnode4, &mnode5, &mnode6, &mnode7, &mnode8, &mnode9, &mnode10);
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
	my_system.Add(truss);
	truss->SetBodyFixed(true);
	
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

				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(100);
	my_system.SetTolSpeeds(1e-12);

	my_system.DoStaticLinear();

				// Output result
	//GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
	//GetLog()<<melement1.GetMatrB()<<"\n";
	GetLog()<<mnode1.GetPos()<<"\n";
	GetLog()<<mnode2.GetPos()<<"\n";
	GetLog()<<mnode3.GetPos()<<"\n";
	GetLog()<<mnode4.GetPos()<<"\n";
	GetLog()<<"node3 displ: "<< mnode3.GetPos()-mnode3.GetX0()<<"\n";

}


//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 4													    //
// Second example: LINEAR HEXAHEDRAL ELEMENT					//
// ============================================================ //
void test_4()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: LINEAR hexahedral element FEM  \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);

				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(207e6);
	mmaterial->Set_v(0.3);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnode1(ChVector<>(0,0,0));
	ChNodeFEMxyz mnode2(ChVector<>(0,0,0.001));
	ChNodeFEMxyz mnode3(ChVector<>(0.001,0,0.001));
	ChNodeFEMxyz mnode4(ChVector<>(0.001,0,0));
	ChNodeFEMxyz mnode5(ChVector<>(0,0.001,0));
	ChNodeFEMxyz mnode6(ChVector<>(0,0.001,0.001));
	ChNodeFEMxyz mnode7(ChVector<>(0.001,0.001,0.001));
	ChNodeFEMxyz mnode8(ChVector<>(0.001,0.001,0));
	mnode1.SetMass(0.001);
	mnode2.SetMass(0.001);
	mnode3.SetMass(0.001);
	mnode4.SetMass(0.001);
	mnode5.SetMass(0.001);
	mnode6.SetMass(0.001);
	mnode7.SetMass(0.001);
	mnode8.SetMass(0.001);
	
				// For example, set an applied force to a node:
	mnode7.SetForce(ChVector<>(0, -1000, 0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);
	my_mesh->AddNode(mnode5);
	my_mesh->AddNode(mnode6);
	my_mesh->AddNode(mnode7);
	my_mesh->AddNode(mnode8);

				// Create the tetrahedron element, and assign 
				// it nodes and material
	ChElementHexa_8 melement1;
	melement1.SetNodes(&mnode1, &mnode2, &mnode3, &mnode4, &mnode5, &mnode6, &mnode7, &mnode8);
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
	my_system.Add(truss);
	truss->SetBodyFixed(true);
	
				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraint1(new ChNodeBody);
	constraint1->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to

	ChSharedPtr<ChNodeBody> constraint2(new ChNodeBody);
	constraint2->Initialize(my_mesh,		// node container
							1,				// index of node in node container 
							truss);			// body to be connected to
						
	ChSharedPtr<ChNodeBody> constraint3(new ChNodeBody);
	constraint3->Initialize(my_mesh,		// node container
							2,				// index of node in node container 
							truss);			// body to be connected to

	ChSharedPtr<ChNodeBody> constraint4(new ChNodeBody);
	constraint4->Initialize(my_mesh,		// node container
							3,				// index of node in node container 
							truss);			// body to be connected to
					
	my_system.Add(constraint1);
	my_system.Add(constraint2);
	my_system.Add(constraint3);
	my_system.Add(constraint4);

				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(100);
	my_system.SetTolSpeeds(1e-12);

	my_system.DoStaticLinear();

				// Output result
	//GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
	//GetLog()<<melement1.GetMatrB()<<"\n";
	GetLog()<<mnode1.GetPos()<<"\n";
	GetLog()<<mnode2.GetPos()<<"\n";
	GetLog()<<mnode3.GetPos()<<"\n";
	GetLog()<<mnode4.GetPos()<<"\n";
	GetLog()<<mnode7.GetPos()<<"\n";
	GetLog()<<"node7 displ: "<< mnode7.GetPos()-mnode7.GetX0()<<"\n";

}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 5													    //
// Second example: QUADRATIC HEXAHEDRAL ELEMENT					//
// ============================================================ //
void test_5()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: QUADRATIC hexahedral element FEM  \n\n";

				// The physical system: it contains all physical objects.
	ChSystem my_system; 
					
				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);

				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(207e6);
	mmaterial->Set_v(0.3);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChNodeFEMxyz mnode1(ChVector<>(0,0,0));
	ChNodeFEMxyz mnode2(ChVector<>(0,0,0.001));
	ChNodeFEMxyz mnode3(ChVector<>(0.001,0,0.001));
	ChNodeFEMxyz mnode4(ChVector<>(0.001,0,0));
	ChNodeFEMxyz mnode5(ChVector<>(0,0.001,0));
	ChNodeFEMxyz mnode6(ChVector<>(0,0.001,0.001));
	ChNodeFEMxyz mnode7(ChVector<>(0.001,0.001,0.001));
	ChNodeFEMxyz mnode8(ChVector<>(0.001,0.001,0));
	
	ChNodeFEMxyz mnode9(ChVector<>((mnode1.pos.x+mnode2.pos.x)/2,(mnode1.pos.y+mnode2.pos.y)/2,(mnode1.pos.z+mnode2.pos.z)/2));
	ChNodeFEMxyz mnode10(ChVector<>((mnode2.pos.x+mnode3.pos.x)/2,(mnode2.pos.y+mnode3.pos.y)/2,(mnode2.pos.z+mnode3.pos.z)/2));
	ChNodeFEMxyz mnode11(ChVector<>((mnode3.pos.x+mnode4.pos.x)/2,(mnode3.pos.y+mnode4.pos.y)/2,(mnode3.pos.z+mnode4.pos.z)/2));
	ChNodeFEMxyz mnode12(ChVector<>((mnode1.pos.x+mnode4.pos.x)/2,(mnode1.pos.y+mnode4.pos.y)/2,(mnode1.pos.z+mnode4.pos.z)/2));

	ChNodeFEMxyz mnode13(ChVector<>((mnode5.pos.x+mnode6.pos.x)/2,(mnode5.pos.y+mnode6.pos.y)/2,(mnode5.pos.z+mnode6.pos.z)/2));
	ChNodeFEMxyz mnode14(ChVector<>((mnode6.pos.x+mnode7.pos.x)/2,(mnode6.pos.y+mnode7.pos.y)/2,(mnode6.pos.z+mnode7.pos.z)/2));
	ChNodeFEMxyz mnode15(ChVector<>((mnode7.pos.x+mnode8.pos.x)/2,(mnode7.pos.y+mnode8.pos.y)/2,(mnode7.pos.z+mnode8.pos.z)/2));
	ChNodeFEMxyz mnode16(ChVector<>((mnode8.pos.x+mnode5.pos.x)/2,(mnode8.pos.y+mnode5.pos.y)/2,(mnode8.pos.z+mnode5.pos.z)/2));

	ChNodeFEMxyz mnode17(ChVector<>((mnode2.pos.x+mnode6.pos.x)/2,(mnode2.pos.y+mnode6.pos.y)/2,(mnode2.pos.z+mnode6.pos.z)/2));
	ChNodeFEMxyz mnode18(ChVector<>((mnode3.pos.x+mnode7.pos.x)/2,(mnode3.pos.y+mnode7.pos.y)/2,(mnode3.pos.z+mnode7.pos.z)/2));
	ChNodeFEMxyz mnode19(ChVector<>((mnode4.pos.x+mnode8.pos.x)/2,(mnode4.pos.y+mnode8.pos.y)/2,(mnode4.pos.z+mnode8.pos.z)/2));
	ChNodeFEMxyz mnode20(ChVector<>((mnode1.pos.x+mnode5.pos.x)/2,(mnode1.pos.y+mnode5.pos.y)/2,(mnode1.pos.z+mnode5.pos.z)/2));

	mnode1.SetMass(0.001);
	mnode2.SetMass(0.001);
	mnode3.SetMass(0.001);
	mnode4.SetMass(0.001);
	mnode5.SetMass(0.001);
	mnode6.SetMass(0.001);
	mnode7.SetMass(0.001);
	mnode8.SetMass(0.001);
	mnode9.SetMass(0.001);
	mnode10.SetMass(0.001);
	mnode11.SetMass(0.001);
	mnode12.SetMass(0.001);
	mnode13.SetMass(0.001);
	mnode14.SetMass(0.001);
	mnode15.SetMass(0.001);
	mnode16.SetMass(0.001);
	mnode17.SetMass(0.001);
	mnode18.SetMass(0.001);
	mnode19.SetMass(0.001);
	mnode20.SetMass(0.001);
	
				// For example, set an applied force to a node:
	mnode7.SetForce(ChVector<>(0, -1000, 0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);
	my_mesh->AddNode(mnode5);
	my_mesh->AddNode(mnode6);
	my_mesh->AddNode(mnode7);
	my_mesh->AddNode(mnode8);
	my_mesh->AddNode(mnode9);
	my_mesh->AddNode(mnode10);
	my_mesh->AddNode(mnode11);
	my_mesh->AddNode(mnode12);
	my_mesh->AddNode(mnode13);
	my_mesh->AddNode(mnode14);
	my_mesh->AddNode(mnode15);
	my_mesh->AddNode(mnode16);
	my_mesh->AddNode(mnode17);
	my_mesh->AddNode(mnode18);
	my_mesh->AddNode(mnode19);
	my_mesh->AddNode(mnode20);

				// Create the tetrahedron element, and assign 
				// it nodes and material
	ChElementHexa_20 melement1;
	melement1.SetNodes(&mnode1, &mnode2, &mnode3, &mnode4, &mnode5, &mnode6, &mnode7, &mnode8,
						&mnode9, &mnode10, &mnode11, &mnode12, &mnode13, &mnode14, &mnode15, &mnode16,
						&mnode17, &mnode18, &mnode19, &mnode20);
	melement1.SetMaterial(mmaterial);

				// Use this statement to use the reduced integration
				// Default number of gauss point: 27. Reduced integration -> 8 Gp.
	melement1.SetReducedIntegrationRule();

				// Remember to add elements to the mesh!
	my_mesh->AddElement(melement1);

				// This is necessary in order to precompute the 
				// stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);


				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	my_system.Add(truss);
	truss->SetBodyFixed(true);
	
				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraint1(new ChNodeBody);
	constraint1->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to

	ChSharedPtr<ChNodeBody> constraint2(new ChNodeBody);
	constraint2->Initialize(my_mesh,		// node container
							1,				// index of node in node container 
							truss);			// body to be connected to
						
	ChSharedPtr<ChNodeBody> constraint3(new ChNodeBody);
	constraint3->Initialize(my_mesh,		// node container
							2,				// index of node in node container 
							truss);			// body to be connected to

	ChSharedPtr<ChNodeBody> constraint4(new ChNodeBody);
	constraint4->Initialize(my_mesh,		// node container
							3,				// index of node in node container 
							truss);			// body to be connected to
					
	my_system.Add(constraint1);
	my_system.Add(constraint2);
	my_system.Add(constraint3);
	my_system.Add(constraint4);

				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(100);
	my_system.SetTolSpeeds(1e-12);

	my_system.DoStaticLinear();

				// Output result
	//GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
	//GetLog()<<melement1.GetMatrB()<<"\n";
	GetLog()<<mnode1.GetPos()<<"\n";
	GetLog()<<mnode2.GetPos()<<"\n";
	GetLog()<<mnode3.GetPos()<<"\n";
	GetLog()<<mnode4.GetPos()<<"\n";
	GetLog()<<mnode7.GetPos()<<"\n";
	GetLog()<<"node7 displ: "<< mnode7.GetPos()-mnode7.GetX0()<<"\n";

}


// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	GetLog() << " Example: the FEM techology for finite elements \n\n\n";


	// Test: an introductory problem:
	test_5();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	system("pause");
	return 0;
}


