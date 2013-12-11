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
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementSpring.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChNodeBody.h"


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
				// While creating them, also set X0 undeformed positions.
	ChSharedPtr<ChNodeFEMxyz> mnodeA( new ChNodeFEMxyz(ChVector<>(0,0,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnodeB( new ChNodeFEMxyz(ChVector<>(0,1,0)) );
	mnodeA->SetMass(0.0);
	mnodeB->SetMass(0.0);
	
				// For example, set an applied force to a node:
	mnodeB->SetForce(ChVector<>(0,5,0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnodeA);
	my_mesh->AddNode(mnodeB);

				// Create some elements of 'spring-damper' type, each connecting
				// two 3D nodes:
	ChSharedPtr<ChElementSpring> melementA( new ChElementSpring );
	melementA->SetNodes(mnodeA, mnodeB);
	melementA->SetSpringK(100000);

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
	GetLog() << "  nodeA->pos \n" << mnodeA->GetPos();
	GetLog() << "  nodeB->pos \n" << mnodeB->GetPos();
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
	mmaterial->Set_E(0.01e9); // rubber 0.01e9, steel 200e9
	mmaterial->Set_v(0.3);

				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
				// While creating them, also set X0 undeformed positions.
	ChSharedPtr<ChNodeFEMxyz> mnode1( new ChNodeFEMxyz(ChVector<>(0,0,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode2( new ChNodeFEMxyz(ChVector<>(0,0,1)) );
	ChSharedPtr<ChNodeFEMxyz> mnode3( new ChNodeFEMxyz(ChVector<>(0,1,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode4( new ChNodeFEMxyz(ChVector<>(1,0,0)) );


				// For example, set an applied force to a node:
	mnode3->SetForce(ChVector<>(0, 10000, 0));

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);

				// Create the tetrahedron element, and assign 
				// nodes and material
	ChSharedPtr<ChElementTetra_4> melement1 (new ChElementTetra_4);
	melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
	melement1->SetMaterial(mmaterial);

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

				// Set no gravity
	//my_system.Set_G_acc(VNULL);


				// Perform a linear static analysis
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetDiagonalPreconditioning(true);
	msolver->SetVerbose(true);
	my_system.SetIterLCPmaxItersSpeed(100);
	my_system.SetTolSpeeds(1e-10);

	my_system.DoStaticLinear();

				// Output result
	GetLog()<< "Resulting node positions:\n";
	GetLog()<< mnode1->pos<<"\n";
	GetLog()<< mnode2->pos<<"\n";
	GetLog()<< mnode3->pos<<"\n";
	GetLog()<< mnode4->pos<<"\n";

	GetLog()<< "Resulting constraint reactions:\n";
	GetLog()<< constraint1->GetReactionOnBody();
	GetLog()<< constraint2->GetReactionOnBody();
	GetLog()<< constraint3->GetReactionOnBody();
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
				// While creating them, also set X0 undeformed positions.
	ChSharedPtr<ChNodeFEMxyz> mnode1(new ChNodeFEMxyz(ChVector<>(0,0,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode2(new ChNodeFEMxyz(ChVector<>(0.001,0,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode3(new ChNodeFEMxyz(ChVector<>(0,0.001,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode4(new ChNodeFEMxyz(ChVector<>(0,0,0.001)) );
	ChSharedPtr<ChNodeFEMxyz> mnode5 (new ChNodeFEMxyz((mnode1->pos + mnode2->pos)*0.5) ); //  nodes at mid length of edges
	ChSharedPtr<ChNodeFEMxyz> mnode6 (new ChNodeFEMxyz((mnode2->pos + mnode3->pos)*0.5) );
	ChSharedPtr<ChNodeFEMxyz> mnode7 (new ChNodeFEMxyz((mnode3->pos + mnode1->pos)*0.5) );
	ChSharedPtr<ChNodeFEMxyz> mnode8 (new ChNodeFEMxyz((mnode1->pos + mnode4->pos)*0.5) );
	ChSharedPtr<ChNodeFEMxyz> mnode9 (new ChNodeFEMxyz((mnode4->pos + mnode2->pos)*0.5) );
	ChSharedPtr<ChNodeFEMxyz> mnode10(new ChNodeFEMxyz((mnode3->pos + mnode4->pos)*0.5) );
	
				// For example, set an applied force to a node:
	mnode3->SetForce(ChVector<>(0, -1000, 0));

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
	ChSharedPtr<ChElementTetra_10> melement1 ( new ChElementTetra_10 );
	melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8, mnode9, mnode10);
	melement1->SetMaterial(mmaterial);

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
	GetLog()<<mnode1->GetPos()<<"\n";
	GetLog()<<mnode2->GetPos()<<"\n";
	GetLog()<<mnode3->GetPos()<<"\n";
	GetLog()<<mnode4->GetPos()<<"\n";
	GetLog()<<"node3 displ: "<< mnode3->GetPos()-mnode3->GetX0()<<"\n";

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
				// While creating them, also set X0 undeformed positions.
	double sx = 0.01;
	double sy = 0.10;
	double sz = 0.01;
	ChSharedPtr<ChNodeFEMxyz> mnode1(new ChNodeFEMxyz(ChVector<>(0, 0,  0)));
	ChSharedPtr<ChNodeFEMxyz> mnode2(new ChNodeFEMxyz(ChVector<>(0, 0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode3(new ChNodeFEMxyz(ChVector<>(sx,0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode4(new ChNodeFEMxyz(ChVector<>(sx,0,  0)));
	ChSharedPtr<ChNodeFEMxyz> mnode5(new ChNodeFEMxyz(ChVector<>(0, sy, 0)));
	ChSharedPtr<ChNodeFEMxyz> mnode6(new ChNodeFEMxyz(ChVector<>(0, sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode7(new ChNodeFEMxyz(ChVector<>(sx,sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode8(new ChNodeFEMxyz(ChVector<>(sx,sy, 0)));

				// For example, set applied forces to nodes:
	mnode5->SetForce(ChVector<>(0, -1000, 0));
	mnode6->SetForce(ChVector<>(0, -1000, 0));
	mnode7->SetForce(ChVector<>(0, -1000, 0));
	mnode8->SetForce(ChVector<>(0, -1000, 0));

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
	ChSharedPtr<ChElementHexa_8> melement1 (new ChElementHexa_8);
	melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8);
	melement1->SetMaterial(mmaterial);

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
	GetLog()<<mnode1->GetPos()<<"\n";
	GetLog()<<mnode2->GetPos()<<"\n";
	GetLog()<<mnode3->GetPos()<<"\n";
	GetLog()<<mnode4->GetPos()<<"\n";
	GetLog()<<"node5 displ: "<< mnode5->GetPos()-mnode5->GetX0()<<"\n";
	GetLog()<<"node6 displ: "<< mnode6->GetPos()-mnode6->GetX0()<<"\n";
	GetLog()<<"node7 displ: "<< mnode7->GetPos()-mnode7->GetX0()<<"\n";
	GetLog()<<"node8 displ: "<< mnode8->GetPos()-mnode8->GetX0()<<"\n";
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
				// While creating them, also set X0 undeformed positions.
	double sx = 0.01;
	double sy = 0.1;
	double sz = 0.01;
	ChSharedPtr<ChNodeFEMxyz> mnode1(new ChNodeFEMxyz(ChVector<>(0, 0,  0)));
	ChSharedPtr<ChNodeFEMxyz> mnode2(new ChNodeFEMxyz(ChVector<>(0, 0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode3(new ChNodeFEMxyz(ChVector<>(sx,0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode4(new ChNodeFEMxyz(ChVector<>(sx,0,  0)));
	ChSharedPtr<ChNodeFEMxyz> mnode5(new ChNodeFEMxyz(ChVector<>(0, sy, 0)));
	ChSharedPtr<ChNodeFEMxyz> mnode6(new ChNodeFEMxyz(ChVector<>(0, sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode7(new ChNodeFEMxyz(ChVector<>(sx,sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> mnode8(new ChNodeFEMxyz(ChVector<>(sx,sy, 0)));
	ChSharedPtr<ChNodeFEMxyz> mnode9 (new ChNodeFEMxyz((mnode1->pos + mnode2->pos)*0.5)); // in between front face
	ChSharedPtr<ChNodeFEMxyz> mnode10(new ChNodeFEMxyz((mnode2->pos + mnode3->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode11(new ChNodeFEMxyz((mnode3->pos + mnode4->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode12(new ChNodeFEMxyz((mnode1->pos + mnode4->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode13(new ChNodeFEMxyz((mnode5->pos + mnode6->pos)*0.5)); // in between back face
	ChSharedPtr<ChNodeFEMxyz> mnode14(new ChNodeFEMxyz((mnode6->pos + mnode7->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode15(new ChNodeFEMxyz((mnode7->pos + mnode8->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode16(new ChNodeFEMxyz((mnode8->pos + mnode5->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode17(new ChNodeFEMxyz((mnode2->pos + mnode6->pos)*0.5)); // in between side edges
	ChSharedPtr<ChNodeFEMxyz> mnode18(new ChNodeFEMxyz((mnode3->pos + mnode7->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode19(new ChNodeFEMxyz((mnode4->pos + mnode8->pos)*0.5)); 
	ChSharedPtr<ChNodeFEMxyz> mnode20(new ChNodeFEMxyz((mnode1->pos + mnode5->pos)*0.5)); 

				// For example, set applied forces to nodes:
	mnode5->SetForce(ChVector<>(0, -500, 0));
	mnode6->SetForce(ChVector<>(0, -500, 0));
	mnode7->SetForce(ChVector<>(0, -500, 0));
	mnode8->SetForce(ChVector<>(0, -500, 0));
	mnode13->SetForce(ChVector<>(0, -500, 0));
	mnode14->SetForce(ChVector<>(0, -500, 0));
	mnode15->SetForce(ChVector<>(0, -500, 0));
	mnode16->SetForce(ChVector<>(0, -500, 0));

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
				// its nodes and material
	ChSharedPtr<ChElementHexa_20> melement1 (new ChElementHexa_20);
	melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8,
						mnode9, mnode10, mnode11, mnode12, mnode13, mnode14, mnode15, mnode16,
						mnode17, mnode18, mnode19, mnode20);
	melement1->SetMaterial(mmaterial);

				// Use this statement to use the reduced integration
				// Default number of gauss point: 27. Reduced integration -> 8 Gp.
	melement1->SetReducedIntegrationRule();

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

				// Output some results
	GetLog()<<"node5 displ: "<< mnode5->GetPos()-mnode5->GetX0()<<"\n";
	GetLog()<<"node6 displ: "<< mnode6->GetPos()-mnode6->GetX0()<<"\n";
	GetLog()<<"node7 displ: "<< mnode7->GetPos()-mnode7->GetX0()<<"\n";
	GetLog()<<"node8 displ: "<< mnode8->GetPos()-mnode8->GetX0()<<"\n";
	GetLog()<<"Element volume" << melement1->GetVolume() << "\n";

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
	test_4();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	system("pause");
	return 0;
}


