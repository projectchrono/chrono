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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
     
// Include some headers used by this tutorial...

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChNodeBody.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "fem/ChElementSpring.h"
#include "fem/ChElementTetra_4.h"
#include "fem/ChMesh.h"


// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;




// Test 1
// First example: 

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
	GetLog() << "Positions after linear static analysis: \n";
	GetLog() << "  nodeA.pos \n" << mnodeA.GetPos();
	GetLog() << "  nodeB.pos \n" << mnodeB.GetPos();
	GetLog() << "Forces after linear static analysis: \n";
	GetLog() << "  constraintA.react \n" << constraintA->GetReactionOnBody();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Test 2
// Second example: TETRAHEDRAL ELEMENT

void test_2()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: tetrahedral element FEM  \n\n";

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



// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();


	GetLog() << " Example: the FEM techology for finite elements \n\n\n";

	// Test: an introductory problem:
	test_2();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	system("pause");
	return 0;
}


