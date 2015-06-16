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
//     - FEA (introduction to dynamics)
//

// Include some headers used by this tutorial...

#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEA/ChElementSpring.h"
#include "unit_FEA/ChElementBar.h"
#include "unit_FEA/ChElementTetra_4.h"
#include "unit_FEA/ChElementTetra_10.h"
#include "unit_FEA/ChElementHexa_8.h"
#include "unit_FEA/ChElementHexa_20.h"
#include "unit_FEA/ChElementShellANCF.h"
#include "unit_FEA/ChMesh.h"
#include "unit_FEA/ChLinkPointFrame.h"
#include "unit_FEA/ChLinkDirFrame.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

void test_1() {
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
    ChSharedPtr<ChNodeFEAxyz> mnodeA(new ChNodeFEAxyz(ChVector<>(0, 0, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnodeB(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(100.0);
    mnodeB->SetMass(100.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    ChSharedPtr<ChElementSpring> melementA(new ChElementSpring);
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(2000);
    melementA->SetDamperR(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    ChSharedPtr<ChLinkPointFrame> constraintA(new ChLinkPointFrame);

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // INT_HHT);//INT_EULER_IMPLICIT);

    double timestep = 0.01;
    while (my_system.GetChTime() < 2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y=" << mnodeB->GetPos().y << "  \n";
    }
}

void test_2() {
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
    ChSharedPtr<ChNodeFEAxyz> mnodeA(new ChNodeFEAxyz(ChVector<>(0, 0, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnodeB(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    ChSharedPtr<ChElementBar> melementA(new ChElementBar);
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));
    // melementA->SetBarDensity(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    ChSharedPtr<ChLinkPointFrame> constraintA(new ChLinkPointFrame);

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT);  // INT_HHT);//INT_EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y=" << mnodeB->GetPos().y << "  \n";
    }

    GetLog() << " Bar mass = " << melementA->GetMass() << "  restlength = " << melementA->GetRestLength() << "\n";
}

void test_2b() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: spring FEM dynamics compare to bar \n\n";

    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    ChSharedPtr<ChNodeFEAxyz> mnodeA(new ChNodeFEAxyz(ChVector<>(0, 0, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnodeB(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(0.1);
    mnodeB->SetMass(0.1);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    ChSharedPtr<ChElementSpring> melementA(new ChElementSpring);
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(20000);
    melementA->SetDamperR(200);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    ChSharedPtr<ChLinkPointFrame> constraintA(new ChLinkPointFrame);

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    my_system.SetIterLCPmaxItersSpeed(200);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT);  // INT_HHT);//INT_EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y=" << mnodeB->GetPos().y << "  \n";
    }
}

void test_3() {
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
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.01);
    mmaterial->Set_density(1000);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    ChSharedPtr<ChNodeFEAxyz> mnode1(new ChNodeFEAxyz(ChVector<>(0, 0, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode2(new ChNodeFEAxyz(ChVector<>(0, 0, 1)));
    ChSharedPtr<ChNodeFEAxyz> mnode3(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode4(new ChNodeFEAxyz(ChVector<>(1, 0, 0)));

    // For example, set a point-like mass at a node:
    mnode1->SetMass(200);
    mnode2->SetMass(200);
    mnode3->SetMass(200);
    mnode4->SetMass(200);
    // For example, set an initial displacement to a node:
    mnode3->SetPos(mnode3->GetX0() + ChVector<>(0, 0.01, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);

    // Create the tetrahedron element, and assign
    // nodes and material
    ChSharedPtr<ChElementTetra_4> melement1(new ChElementTetra_4);
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
    ChSharedPtr<ChLinkPointFrame> constraint1(new ChLinkPointFrame);
    ChSharedPtr<ChLinkPointFrame> constraint2(new ChLinkPointFrame);
    ChSharedPtr<ChLinkPointFrame> constraint3(new ChLinkPointFrame);

    constraint1->Initialize(mnode1,  // node
                            truss);  // body to be connected to

    constraint2->Initialize(mnode2,  // node
                            truss);  // body to be connected to

    constraint3->Initialize(mnode4,  // node
                            truss);  // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);

    // Perform a dynamic time integration:

    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(
        ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // INT_EULER_IMPLICIT_LINEARIZED);//INT_EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.1) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t =" << my_system.GetChTime() << "  mnode3 pos.y=" << mnode3->GetPos().y << "  \n";
    }
}

void test_4() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics (2 elements),  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    ChSharedPtr<ChNodeFEAxyz> mnodeA(new ChNodeFEAxyz(ChVector<>(0, 0, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnodeB(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnodeC(new ChNodeFEAxyz(ChVector<>(0, 2, 0)));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);
    mnodeC->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));
    // For example, set an applied force to a node:
    mnodeC->SetForce(ChVector<>(0, 2, 0));

    // For example, set an initial displacement to a node:
    mnodeC->SetPos(mnodeC->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);
    my_mesh->AddNode(mnodeC);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    ChSharedPtr<ChElementBar> melementA(new ChElementBar);
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));

    ChSharedPtr<ChElementBar> melementB(new ChElementBar);
    melementB->SetNodes(mnodeB, mnodeC);
    melementB->SetBarArea(0.1 * 0.02);
    melementB->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementB->SetBarRaleyghDamping(0.01);
    melementB->SetBarDensity(2. * 0.1 / (melementB->GetBarArea() * 1.0));

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);
    my_mesh->AddElement(melementB);

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    ChSharedPtr<ChLinkPointFrame> constraintA(new ChLinkPointFrame);

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT);  // INT_HHT);//INT_EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y=" << mnodeB->GetPos().y
                 << "  nodeC pos.y=" << mnodeC->GetPos().y << "  \n";
    }
}
void test_5() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics (2 elements),  implicit integration \n\n";

	FILE *inputfile; // input stream
	FILE *outputfile;

    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

	int numFlexBody=0;
	int numDiv=0;
	int numDiv_x=0; 
	int numDiv_y=0;
	int numNode=0;
	int dammy=0;
	int count=0;
	int MaxMNUM=0;
	int MTYPE=0;
	int MaxLayNum=0;
	int TotalNumNodes=0;
	char str1[100];


	// There is a limitation now as follows
	//  Max #element is 1000;
	//  Max #Layer   is 7
    //  Only Orthotropic material

	double COORDFlex[5000][6];
	double VELCYFlex[5000][6];
	double LayPROP[10][7][2];
	int LayNUM[5000];
	int NumNodes[5000][4];
	int NDR[5000][6];
	int NumLayer[10];
	int MNUM[10][7];
	double MPROP[10][12];
	double ElemLengthXY[5000][2];


	int MAXCOUNT = 100;
	inputfile = fopen("IndataBiLinearShell_Simple1.DAT","r");
	printf("Open IndataBiLinearShell_Simple1.DAT\n");
	if(inputfile == NULL){
		printf("IndataBiLinearShell_Simple1.DAT cannot open!!\n");
		system("pause");
		exit(1);
	}
	//!--------------------------------------!
    //!-- Elememt data            -----------!
    //!--------------------------------------!
	
	fgets(str1,MAXCOUNT,inputfile);
	printf("%s\n",str1);
	fscanf(inputfile,"%d\n",&numFlexBody);

	fgets(str1,MAXCOUNT,inputfile);
	printf("%s\n",str1);
	fscanf(inputfile,"%d %d %d %d\n",&numDiv,&numDiv_x,&numDiv_y,&numNode);
	fgets(str1,MAXCOUNT,inputfile);

	TotalNumNodes=numNode;

	printf("%s\n",str1);
	for(int i=0;i<numDiv;i++)
	{
		fscanf(inputfile,"%d %d %d %d %d %d %d\n",&count,&dammy,&LayNUM[i],&NumNodes[i][0],&NumNodes[i][1],&NumNodes[i][2],&NumNodes[i][3]);
		printf("LayNUM[i] %d\n  ",LayNUM[i]);

		fscanf(inputfile," %lf %lf\n",&ElemLengthXY[i][0],&ElemLengthXY[i][1]);
		if(MaxLayNum<LayNUM[i])
		{MaxLayNum=LayNUM[i];}
		//if(TotalNumNodes<max(NumNodes[i][0],max(NumNodes[i][1],max(NumNodes[i][2],NumNodes[i][3]))))
		//{TotalNumNodes=max(NumNodes[i][0],max(NumNodes[i][1],max(NumNodes[i][2],NumNodes[i][3])));}

		//printf("MaxLayNum %lf, %lf \n ", ElemLengthXY[i][0],ElemLengthXY[i][1]);
	}

	//!--------------------------------------!
    //!-- NDR,COORDFlex,VELCYFlex -----------!
    //!--------------------------------------!
	//fscanf(inputfile,"%s\n",str1);
	fgets(str1,MAXCOUNT,inputfile);
	printf("%s\n",str1);
	for(int i=0;i<TotalNumNodes;i++)
	{
		fscanf(inputfile,"%d %d %d %d %d %d %d\n",&count,&NDR[i][0],&NDR[i][1],&NDR[i][2],&NDR[i][3],&NDR[i][4],&NDR[i][5]);
		fscanf(inputfile,"%lf %lf %lf %lf %lf %lf\n",&COORDFlex[i][0],&COORDFlex[i][1],&COORDFlex[i][2],&COORDFlex[i][3],&COORDFlex[i][4],&COORDFlex[i][5]);
		fscanf(inputfile,"%lf %lf %lf %lf %lf %lf\n",&VELCYFlex[i][0],&VELCYFlex[i][1],&VELCYFlex[i][2],&VELCYFlex[i][3],&VELCYFlex[i][4],&VELCYFlex[i][5]);
		//printf("NumNodes %d %d %d %d %d %d\n",NDR[i][0],NDR[i][1],NDR[i][2],NDR[i][3],NDR[i][4],NDR[i][5]);
		//printf("NumNodes %lf %lf %lf %lf %lf %lf\n",COORDFlex[i][0],COORDFlex[i][1],COORDFlex[i][2],COORDFlex[i][3],COORDFlex[i][4],COORDFlex[i][5]);
		//system("pause");
	}

	//!--------------------------------------!
    //!--- Read Layer Data ------------------!
    //!--------------------------------------!
	//fscanf(inputfile,"%s\n",str1);
	fgets(str1,MAXCOUNT,inputfile);
	printf("%s\n",str1);
	for(int i=0;i<MaxLayNum;i++)
	{
		fscanf(inputfile,"%d %d\n",&count,&NumLayer[i]);
	    for(int j=0;j<NumLayer[i];j++)
		{
		fscanf(inputfile,"%lf %lf %d\n",&LayPROP[i][j][0],&LayPROP[i][j][1],&MNUM[i][j]);
		if(MaxMNUM<MNUM[i][j])
		{MaxMNUM=MNUM[i][j];}
		//printf("%lf %lf %d\n",LayPROP[i][j][0],LayPROP[i][j][1],MNUM[i][j]);
		}
		//system("pause");
	}

	//!--------------------------------------!
    //!--- Read Material Data ---------------!
    //!--------------------------------------!
	//fscanf(inputfile,"%s\n",str1);
	fgets(str1,MAXCOUNT,inputfile);
	printf("%s\n",str1);
	for(int i=0;i<MaxMNUM;i++)
	{
		fscanf(inputfile,"%d %d\n",&count,&MTYPE);
		if(MTYPE==1)
		{
			fscanf(inputfile,"%lf %lf %lf %lf\n",&MPROP[i][0],&MPROP[i][1],&MPROP[i][2],&MPROP[i][3]);
		}
		if(MTYPE==2)
		{
			fscanf(inputfile,"%lf %lf %lf %lf\n",&MPROP[i][0],&MPROP[i][1],&MPROP[i][2],&MPROP[i][3]);
			fscanf(inputfile,"%lf %lf %lf %lf %lf %lf\n",&MPROP[i][4],&MPROP[i][5],&MPROP[i][6],&MPROP[i][7],&MPROP[i][8],&MPROP[i][9]);
		}
	}


	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_RayleighDampingK(0.0);
	mmaterial->Set_RayleighDampingM(0.0);

	
	// Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

	int i=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(ChVector<>(COORDFlex[i][0], COORDFlex[i][1], COORDFlex[i][2]),ChVector<>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5])));
		ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
		ChVector<double> dir =  (COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]);

		//GetLog() << "TotalNumNodes" << TotalNumNodes << "\n\n";
		node->SetMass(0.0);
		my_mesh->AddNode(node);
		//if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1&&NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		//{
		//	//node->SetFixed(true);
		//	constraint->Initialize(node,		// node
		//						   truss);		// body to be connected to

		//	constraintD->Initialize(node,
		//							truss,
		//							&ChVector<>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));

		//my_system.Add(constraint);
		//my_system.Add(constraintD);
		//
		//GetLog()<<constraint.DynamicCastTo<ChLinkPointFrame>()->GetDOC()<<","<<constraintD.DynamicCastTo<ChLinkDirFrame>()->GetDirection()<<"\n";
		//system("pause");

		//}else if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1&&NDR[i][3]==0&&NDR[i][4]==0&&NDR[i][5]==0)
		//{
		//	node->SetFixedPos(true);
		//}else if(NDR[i][0]==0&&NDR[i][1]==0&&NDR[i][2]==0&&NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		//{
		//	node->SetFixedD(true);
		//}
		i++;
	}
	GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";


	ChSharedPtr<ChNodeFEAxyzD> nodetip (my_mesh->GetNode(TotalNumNodes-1).DynamicCastTo<ChNodeFEAxyzD>());
	GetLog() << "X : " << nodetip->GetPos().x <<" Y : "<< nodetip->GetPos().y <<" Z : "<< nodetip->GetPos().z <<"\n\n";
	GetLog() << "dX : " << nodetip->GetD().x <<" dY : "<< nodetip->GetD().y <<" dZ : "<< nodetip->GetD().z <<"\n\n";

	int elemcount=0;
	while(elemcount<numDiv)
	{

		ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
		// Save material data into InertFlexVec(98x1) at each layer
		ChMatrixNM<double,98,1> InertFlexVec;
		InertFlexVec.Reset();
		double TotalThickness; // element thickness
		TotalThickness=0.0;
		int i=elemcount;
		for(int j=0;j<NumLayer[LayNUM[i]-1];j++)
		{
		int ij=14*j;
        InertFlexVec(ij)=MPROP[MNUM[LayNUM[i]-1][j]-1][0]; // Density
		InertFlexVec(ij+1)=ElemLengthXY[i][0];             // EL
		InertFlexVec(ij+2)=ElemLengthXY[i][1];             // EW
		InertFlexVec(ij+3)=LayPROP[LayNUM[i]-1][j][0];     // Thickness per layer
		TotalThickness+=InertFlexVec(ij+3);
		InertFlexVec(ij+4)=LayPROP[LayNUM[i]-1][j][1];     // Fiber angle
		InertFlexVec(ij+5)=MPROP[MNUM[LayNUM[i]-1][j]-1][1]; // Ex
		InertFlexVec(ij+6)=MPROP[MNUM[LayNUM[i]-1][j]-1][2]; // Ey
		InertFlexVec(ij+7)=MPROP[MNUM[LayNUM[i]-1][j]-1][3]; // Ez
		InertFlexVec(ij+8)=MPROP[MNUM[LayNUM[i]-1][j]-1][4]; // nuxy
		InertFlexVec(ij+9)=MPROP[MNUM[LayNUM[i]-1][j]-1][5]; // nuxz
		InertFlexVec(ij+10)=MPROP[MNUM[LayNUM[i]-1][j]-1][6];// nuyz
		InertFlexVec(ij+11)=MPROP[MNUM[LayNUM[i]-1][j]-1][7];// Gxy
		InertFlexVec(ij+12)=MPROP[MNUM[LayNUM[i]-1][j]-1][8];// Gxz
		InertFlexVec(ij+13)=MPROP[MNUM[LayNUM[i]-1][j]-1][9];// Gyz
		}
		//for(int ijkll=0;ijkll<98;ijkll++){
		//GetLog() <<InertFlexVec(ijkll) <<"\n";
		//}
		//system("pause");
		ChMatrixNM<double,7,2> GaussZRange;
		GaussZRange.Reset();
		double CurrentHeight=0.0;
		for(int j=0;j<NumLayer[LayNUM[i]-1];j++)
		{
			double AA=(CurrentHeight/TotalThickness-0.5)*2.0;
			CurrentHeight += LayPROP[LayNUM[i]-1][j][0];
			double AAA=(CurrentHeight/TotalThickness-0.5)*2.0;
			GaussZRange(j,0)=AA;
			GaussZRange(j,1)=AAA;
		}
		element->SetInertFlexVec(InertFlexVec);
		element->SetGaussZRange(GaussZRange);
		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyzD>());
		element->SetMaterial(mmaterial);
		element->SetNumLayer(NumLayer[LayNUM[i]-1]);
		element->SetThickness(TotalThickness);
		element->SetElemNum(elemcount);
		ChMatrixNM<double,35,1> StockAlpha_EAS; // StockAlpha(5*7,1): Max #Layer is 7
		StockAlpha_EAS.Reset();
		element->SetStockAlpha(StockAlpha_EAS);
		my_mesh->AddElement(element);
		elemcount++;
	}

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
	GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
	i=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
		ChVector<double> dir =  (COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]);


		if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1&&NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		{
			//node->SetFixed(true);
			constraint->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(),		// node
								   truss);		// body to be connected to

			constraintD->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>(),
									truss);
									//&ChVector<double>(0.0,0.0,0.0));
									//&ChVector<double>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));

		my_system.Add(constraint);
		my_system.Add(constraintD);
		
		}
		
		i++;
	}

    // Perform a dynamic time integration:
    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_HHT);  // INT_HHT);//INT_EULER_IMPLICIT);
	if( ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>() )
	{
		mystepper->SetAlpha(-0.2);
		mystepper->SetMaxiters(100);
		mystepper->SetTolerance(1e-4);
		//mystepper->Iterations=0;
	}
	outputfile = fopen("position.txt","w");
	double start = std::clock();
    double timestep = 0.001;
    while (my_system.GetChTime() < 2.0) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n\n";

		fprintf(outputfile,"  %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"%e  ",nodetip->GetD().x);
		fprintf(outputfile,"%e  ",nodetip->GetD().y);
		fprintf(outputfile,"%e  ",nodetip->GetD().z);
		fprintf(outputfile,"\n  ");
		//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	//GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}

// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    // Test: an introductory problem:
    test_5();

    system("pause");
    return 0;
}
