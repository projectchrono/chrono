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
//     - FEA using ANCF (introduction to dynamics)
//

// Include some headers used by this tutorial...

#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEA/ChElementSpring.h"
#include "unit_FEA/ChElementShellANCF.h"
#include "unit_FEA/ChElementBrick.h"
#include "unit_FEA/ChElementBar.h"
#include "unit_FEA/ChElementTetra_4.h"
#include "unit_FEA/ChElementTetra_10.h"
#include "unit_FEA/ChElementHexa_8.h"
#include "unit_FEA/ChElementHexa_20.h"
#include "unit_FEA/ChMesh.h"
#include "unit_FEA/ChLinkPointFrame.h"
#include "unit_FEA/ChLinkDirFrame.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

void test_1() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Shell Simple Case (Constraints),  implicit integration \n\n";

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
	//int ICON[1000][24];
	//double QI[1000][24];
	//double QDI[1000][24];
	//double INRTAFlex[1000][98]; //[1000][7][14]
	//int NonlinearMaterialFlag=2;
	//
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
	inputfile = fopen("IndataBiLinearShell_Simple1_8x8.DAT","r");
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
	//printf("%lf %lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2],MPROP[i][3]);
	}
	//!--------------------------------------!
    //!--- Save into INRTAFlex --------------!
    //!--------------------------------------!
	//vector<vector<vector<double>>> INRTAFlex(numDiv, vector<vector<double>>(MaxLayInf, vector<double>(MaxMatInf, 0)));
    //Al_INRTAFlex(INRTAFlex, numDiv, MaxLayInf, MaxMatInf);
	//for(int i=0;i<numDiv;i++)
	//{
	//	//

	//}

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
		node->SetMass(0.0);
		my_mesh->AddNode(node);
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
		//GetLog() << "GaussZRange" << GaussZRange;
		//
		element->SetInertFlexVec(InertFlexVec);
		element->SetGaussZRange(GaussZRange);
		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyzD>());
		element->SetMaterial(mmaterial);
		element->SetNumLayer(NumLayer[LayNUM[i]-1]);
		element->SetThickness(TotalThickness);
		element->SetElemNum(elemcount);
		element->SetAlphaDamp(0.00);
		//== 7/14/2015
		element->Setdt(0.001); // dt to calculate DampingCoefficient
		element->SetGravityZ(1); // 0:No Gravity, 1:Gravity(Fz=-9.81)
		element->SetAirPressure(0); // 0:No AirPressure, 1:220kPa Air Pressure
		//element->SetStockAlpha(0.0,0.0,0.0,0.0,0.0);
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
	//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
	i=0;
	int icount=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
		if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1)
		{
			constraint->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(),		// node
								   truss);		// body to be connected to
			
			my_system.Add(constraint);
		}
		if(NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		{
			constraintD->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>(),
									truss);

			constraintD->SetDirectionInBodyCoords(ChVector<double>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));
			my_system.Add(constraintD);
			icount++;
		}
		
		//GetLog()<<"\n"<<i<<"\n";
		//GetLog()<<"\n"<<COORDFlex[i][3]<<"\n"<<COORDFlex[i][4]<<"\n"<<COORDFlex[i][5]<<"\n";
		//system("pause");
		i++;
	}
	GetLog()<<"\n"<<icount<<"\n";
	system("pause");
    // Perform a dynamic time integration:
    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetTolForce(1e-5);

    my_system.SetIntegrationType(ChSystem::INT_HHT);  // INT_HHT);//INT_EULER_IMPLICIT);
	if( ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>() )
	{
		mystepper->SetAlpha(-0.2);
		mystepper->SetMaxiters(100);
		mystepper->SetTolerance(1e-5);
		mystepper->Iterations=0;
	}
	outputfile = fopen("position.txt","w");
	double start = std::clock();
    double timestep = 0.001;
    while (my_system.GetChTime() < 2.0) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
		
		//GetLog() << "N0: t=" << my_system.GetChTime() << "  x:"<<mnode0->GetPos().x << "  y:"<<mnode0->GetPos().y <<"  z:"<<mnode0->GetPos().z <<"  rx:"<<mnode0->GetD().x <<"  ry:"<<mnode0->GetD().y <<" rz:"<<mnode0->GetD().z<<" \n";
		//GetLog() << "N11: t=" << my_system.GetChTime() << "  x:"<<mnode11->GetPos().x << "  y:"<<mnode11->GetPos().y <<"  z:"<<mnode11->GetPos().z <<"  rx:"<<mnode11->GetD().x <<"  ry:"<<mnode11->GetD().y <<" rz:"<<mnode11->GetD().z<<" \n";
		//GetLog() << "N21: t=" << my_system.GetChTime() << "  x:"<<mnode21->GetPos().x << "  y:"<<mnode21->GetPos().y <<"  z:"<<mnode21->GetPos().z <<"  rx:"<<mnode21->GetD().x <<"  ry:"<<mnode21->GetD().y <<" rz:"<<mnode21->GetD().z<<" \n";
		//GetLog() << "N10: t=" << my_system.GetChTime() << "  x:"<<mnode10->GetPos().x << "  y:"<<mnode10->GetPos().y <<"  z:"<<mnode10->GetPos().z <<"  rx:"<<mnode10->GetD().x <<"  ry:"<<mnode10->GetD().y <<" rz:"<<mnode10->GetD().z<<" \n";

		fprintf(outputfile,"  %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"%e  ",nodetip->GetD().x);
		fprintf(outputfile,"%e  ",nodetip->GetD().y);
		fprintf(outputfile,"%e  ",nodetip->GetD().z);
		fprintf(outputfile,"\n  ");
		//system("pause");
		//GetLog()<< nodetip->GetPos().x<<"\n"<<nodetip->GetPos().y<<"\n"<<nodetip->GetPos().z<<"\n";
		//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	//GetLog() << "Iterations: " << my_system.Iter << "\n";
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}

void test_2() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Tire (Constraints),  implicit integration \n\n";

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
	//int ICON[1000][24];
	//double QI[1000][24];
	//double QDI[1000][24];
	//double INRTAFlex[1000][98]; //[1000][7][14]
	//int NonlinearMaterialFlag=2;
	//
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
	inputfile = fopen("IndataBiLinearShell_Tire3.DAT","r");
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
	//printf("%lf %lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2],MPROP[i][3]);
	}
	//!--------------------------------------!
    //!--- Save into INRTAFlex --------------!
    //!--------------------------------------!
	//vector<vector<vector<double>>> INRTAFlex(numDiv, vector<vector<double>>(MaxLayInf, vector<double>(MaxMatInf, 0)));
    //Al_INRTAFlex(INRTAFlex, numDiv, MaxLayInf, MaxMatInf);
	//for(int i=0;i<numDiv;i++)
	//{
	//	//

	//}

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
		node->SetMass(0.0);
		my_mesh->AddNode(node);
		i++;
	}
	GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";


	ChSharedPtr<ChNodeFEAxyzD> nodetip (my_mesh->GetNode(/*TotalNumNodes-1*/(numDiv/2)).DynamicCastTo<ChNodeFEAxyzD>());
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
		//GetLog() << "GaussZRange" << GaussZRange;
		//
		element->SetInertFlexVec(InertFlexVec);
		element->SetGaussZRange(GaussZRange);
		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyzD>());
		element->SetMaterial(mmaterial);
		element->SetNumLayer(NumLayer[LayNUM[i]-1]);
		element->SetThickness(TotalThickness);
		element->SetElemNum(elemcount);
		element->SetAlphaDamp(0.005);
		//== 7/14/2015
		element->Setdt(0.00025); // dt to calculate DampingCoefficient
		element->SetGravityZ(0); // 0:No Gravity, 1:Gravity(Fz=-9.81)
		element->SetAirPressure(1); // 0:No AirPressure, 1:220kPa Air Pressure
		//element->SetStockAlpha(0.0,0.0,0.0,0.0,0.0);
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
	//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
	i=0;
	int icount=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD2(new ChLinkDirFrame);
		if(i<numDiv_x||i>=numNode-numDiv_x){
			constraint->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(),		// node
								   truss);		// body to be connected to
			
			constraintD->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>(),
									truss);

			constraintD->SetDirectionInBodyCoords(ChVector<double>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));
			
			my_system.Add(constraint);
			my_system.Add(constraintD);

		icount++;
		//GetLog()<<"\n"<<i<<"\n";
		}
		//GetLog()<<"\n"<<COORDFlex[i][3]<<"\n"<<COORDFlex[i][4]<<"\n"<<COORDFlex[i][5]<<"\n";
		//system("pause");
		i++;
	}
	GetLog()<<"\n"<<icount<<"\n";
	system("pause");
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
		mystepper->SetMaxiters(1000);
		mystepper->SetTolerance(5e-4);
		mystepper->Iterations=0;
	}
	outputfile = fopen("position.txt","w");
	double start = std::clock();
    double timestep = 0.00025;
    while (my_system.GetChTime() < 0.1) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
		
		//GetLog() << "N0: t=" << my_system.GetChTime() << "  x:"<<mnode0->GetPos().x << "  y:"<<mnode0->GetPos().y <<"  z:"<<mnode0->GetPos().z <<"  rx:"<<mnode0->GetD().x <<"  ry:"<<mnode0->GetD().y <<" rz:"<<mnode0->GetD().z<<" \n";
		//GetLog() << "N11: t=" << my_system.GetChTime() << "  x:"<<mnode11->GetPos().x << "  y:"<<mnode11->GetPos().y <<"  z:"<<mnode11->GetPos().z <<"  rx:"<<mnode11->GetD().x <<"  ry:"<<mnode11->GetD().y <<" rz:"<<mnode11->GetD().z<<" \n";
		//GetLog() << "N21: t=" << my_system.GetChTime() << "  x:"<<mnode21->GetPos().x << "  y:"<<mnode21->GetPos().y <<"  z:"<<mnode21->GetPos().z <<"  rx:"<<mnode21->GetD().x <<"  ry:"<<mnode21->GetD().y <<" rz:"<<mnode21->GetD().z<<" \n";
		//GetLog() << "N10: t=" << my_system.GetChTime() << "  x:"<<mnode10->GetPos().x << "  y:"<<mnode10->GetPos().y <<"  z:"<<mnode10->GetPos().z <<"  rx:"<<mnode10->GetD().x <<"  ry:"<<mnode10->GetD().y <<" rz:"<<mnode10->GetD().z<<" \n";

		fprintf(outputfile,"  %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"%e  ",nodetip->GetD().x);
		fprintf(outputfile,"%e  ",nodetip->GetD().y);
		fprintf(outputfile,"%e  ",nodetip->GetD().z);
		fprintf(outputfile,"\n  ");
		//system("pause");
		//GetLog()<< nodetip->GetPos().x<<"\n"<<nodetip->GetPos().y<<"\n"<<nodetip->GetPos().z<<"\n";
		//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	//GetLog() << "Iterations: " << my_system.Iter << "\n";
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}

void test_3() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Shell Simple Case (Fixed),  implicit integration \n\n";

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
	//int ICON[1000][24];
	//double QI[1000][24];
	//double QDI[1000][24];
	//double INRTAFlex[1000][98]; //[1000][7][14]
	//int NonlinearMaterialFlag=2;
	//
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
	inputfile = fopen("IndataBiLinearShell_Simple1_8x8.DAT","r");
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
	//printf("%lf %lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2],MPROP[i][3]);
	}
	//!--------------------------------------!
    //!--- Save into INRTAFlex --------------!
    //!--------------------------------------!
	//vector<vector<vector<double>>> INRTAFlex(numDiv, vector<vector<double>>(MaxLayInf, vector<double>(MaxMatInf, 0)));
    //Al_INRTAFlex(INRTAFlex, numDiv, MaxLayInf, MaxMatInf);
	//for(int i=0;i<numDiv;i++)
	//{
	//	//

	//}

	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_RayleighDampingK(0.0);
	mmaterial->Set_RayleighDampingM(0.0);

	
	// Create also a truss
    //ChSharedPtr<ChBody> truss(new ChBody);
    //truss->SetBodyFixed(true);
    //my_system.Add(truss);

	int i=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(ChVector<>(COORDFlex[i][0], COORDFlex[i][1], COORDFlex[i][2]),ChVector<>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5])));
		node->SetMass(0.0);
		my_mesh->AddNode(node);
		if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1&&NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		{
			node->SetFixed(true);
		}
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
		//GetLog() << "GaussZRange" << GaussZRange;
		//
		element->SetInertFlexVec(InertFlexVec);
		element->SetGaussZRange(GaussZRange);
		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyzD>());
		element->SetMaterial(mmaterial);
		element->SetNumLayer(NumLayer[LayNUM[i]-1]);
		element->SetThickness(TotalThickness);
		element->SetElemNum(elemcount);
		element->SetAlphaDamp(0.00);
		//== 7/14/2015
		element->Setdt(0.001); // dt to calculate DampingCoefficient
		element->SetGravityZ(1); // 0:No Gravity, 1:Gravity(Fz=-9.81)
		element->SetAirPressure(0); // 0:No AirPressure, 1:220kPa Air Pressure
		//element->SetStockAlpha(0.0,0.0,0.0,0.0,0.0);
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
	//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
	i=0;
	int icount=0;
	//while(i<TotalNumNodes)
	//{
	//	ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
	//	ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
	//	if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1)
	//	{
	//		constraint->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(),		// node
	//							   truss);		// body to be connected to
	//		
	//		//my_system.Add(constraint);
	//	}
	//	if(NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
	//	{
	//		constraintD->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>(),
	//								truss);

	//		constraintD->SetDirectionInBodyCoords(ChVector<double>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));
	//		//my_system.Add(constraintD);
	//	}
		//icount++;
		//GetLog()<<"\n"<<i<<"\n";
		//GetLog()<<"\n"<<COORDFlex[i][3]<<"\n"<<COORDFlex[i][4]<<"\n"<<COORDFlex[i][5]<<"\n";
		//system("pause");
		//i++;
	//}
	GetLog()<<"\n"<<icount<<"\n";
	system("pause");
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
		mystepper->Iterations=0;
	}
	outputfile = fopen("position.txt","w");
	double start = std::clock();
    double timestep = 0.001;
    while (my_system.GetChTime() < 2.0) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
		
		//GetLog() << "N0: t=" << my_system.GetChTime() << "  x:"<<mnode0->GetPos().x << "  y:"<<mnode0->GetPos().y <<"  z:"<<mnode0->GetPos().z <<"  rx:"<<mnode0->GetD().x <<"  ry:"<<mnode0->GetD().y <<" rz:"<<mnode0->GetD().z<<" \n";
		//GetLog() << "N11: t=" << my_system.GetChTime() << "  x:"<<mnode11->GetPos().x << "  y:"<<mnode11->GetPos().y <<"  z:"<<mnode11->GetPos().z <<"  rx:"<<mnode11->GetD().x <<"  ry:"<<mnode11->GetD().y <<" rz:"<<mnode11->GetD().z<<" \n";
		//GetLog() << "N21: t=" << my_system.GetChTime() << "  x:"<<mnode21->GetPos().x << "  y:"<<mnode21->GetPos().y <<"  z:"<<mnode21->GetPos().z <<"  rx:"<<mnode21->GetD().x <<"  ry:"<<mnode21->GetD().y <<" rz:"<<mnode21->GetD().z<<" \n";
		//GetLog() << "N10: t=" << my_system.GetChTime() << "  x:"<<mnode10->GetPos().x << "  y:"<<mnode10->GetPos().y <<"  z:"<<mnode10->GetPos().z <<"  rx:"<<mnode10->GetD().x <<"  ry:"<<mnode10->GetD().y <<" rz:"<<mnode10->GetD().z<<" \n";

		fprintf(outputfile,"  %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"%e  ",nodetip->GetD().x);
		fprintf(outputfile,"%e  ",nodetip->GetD().y);
		fprintf(outputfile,"%e  ",nodetip->GetD().z);
		fprintf(outputfile,"\n  ");
		//system("pause");
		//GetLog()<< nodetip->GetPos().x<<"\n"<<nodetip->GetPos().y<<"\n"<<nodetip->GetPos().z<<"\n";
		//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	//GetLog() << "Iterations: " << my_system.Iter << "\n";
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}

void test_4() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Tire (Fixed),  implicit integration \n\n";

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
	//int ICON[1000][24];
	//double QI[1000][24];
	//double QDI[1000][24];
	//double INRTAFlex[1000][98]; //[1000][7][14]
	//int NonlinearMaterialFlag=2;
	//
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
	inputfile = fopen("IndataBiLinearShell_Tire3.DAT","r");
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
	//printf("%lf %lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2],MPROP[i][3]);
	}
	//!--------------------------------------!
    //!--- Save into INRTAFlex --------------!
    //!--------------------------------------!
	//vector<vector<vector<double>>> INRTAFlex(numDiv, vector<vector<double>>(MaxLayInf, vector<double>(MaxMatInf, 0)));
    //Al_INRTAFlex(INRTAFlex, numDiv, MaxLayInf, MaxMatInf);
	//for(int i=0;i<numDiv;i++)
	//{
	//	//

	//}

	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_RayleighDampingK(0.0);
	mmaterial->Set_RayleighDampingM(0.0);

	
	// Create also a truss
    ChSharedPtr<ChBody> truss(new ChBody);
    truss->SetBodyFixed(true);
    my_system.Add(truss);

	//int i=0;
	//while(i<TotalNumNodes)
	//{
	//	ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(ChVector<>(COORDFlex[i][0], COORDFlex[i][1], COORDFlex[i][2]),ChVector<>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5])));
	//	node->SetMass(0.0);
	//	my_mesh->AddNode(node);
	//	//if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1)
	//	//{
	//	//	node->SetFixed(true);
	//	//}
	//	if(i<numDiv_x||i>=numNode-numDiv_x){
	//		node->SetFixed(true);
	//		//GetLog()<<i<<"\n";
	//		//GetLog()<<"\n"<<COORDFlex[i][0]<<"\n"<<COORDFlex[i][1]<<"\n"<<COORDFlex[i][2]<<"\n"<<COORDFlex[i][3]<<"\n"<<COORDFlex[i][4]<<"\n"<<COORDFlex[i][5]<<"\n";
	//		//system("pause");
	//	}
	//	i++;
	//}
	//GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";
	int i=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(ChVector<>(COORDFlex[i][0], COORDFlex[i][1], COORDFlex[i][2]),ChVector<>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5])));
		node->SetMass(0.0);
		my_mesh->AddNode(node);
		if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1&&NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		{
			node->SetFixed(true);
		}
		i++;
	}
	GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";

	ChSharedPtr<ChNodeFEAxyzD> nodetip (my_mesh->GetNode(/*TotalNumNodes-1*/(numDiv/2)).DynamicCastTo<ChNodeFEAxyzD>());
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
		//GetLog() << "GaussZRange" << GaussZRange;
		//
		element->SetInertFlexVec(InertFlexVec);
		element->SetGaussZRange(GaussZRange);
		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyzD>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyzD>());
		element->SetMaterial(mmaterial);
		element->SetNumLayer(NumLayer[LayNUM[i]-1]);
		element->SetThickness(TotalThickness);
		element->SetElemNum(elemcount);
		element->SetAlphaDamp(0.005);
		//== 7/14/2015
		element->Setdt(0.00025); // dt to calculate DampingCoefficient
		element->SetGravityZ(0); // 0:No Gravity, 1:Gravity(Fz=-9.81)
		element->SetAirPressure(1); // 0:No AirPressure, 1:220kPa Air Pressure
		//
		//element->SetStockAlpha(0.0,0.0,0.0,0.0,0.0);
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
	//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
	i=0;
	int icount=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChLinkPointFrame> constraint(new ChLinkPointFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD(new ChLinkDirFrame);
		ChSharedPtr<ChLinkDirFrame> constraintD2(new ChLinkDirFrame);
		if(i<numDiv_x||i>=numNode-numDiv_x){
		//if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1)
		//{
			constraint->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(),		// node
								   truss);		// body to be connected to
			
			//my_system.Add(constraint);
		//}
		//if(NDR[i][3]==1&&NDR[i][4]==1&&NDR[i][5]==1)
		//{
			constraintD->Initialize(my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyzD>(),
									truss);

			constraintD->SetDirectionInBodyCoords(ChVector<double>(COORDFlex[i][3], COORDFlex[i][4], COORDFlex[i][5]));
			//my_system.Add(constraintD);

		//}
		icount++;
		//GetLog()<<"\n"<<i<<"\n";
		}
		//GetLog()<<"\n"<<COORDFlex[i][3]<<"\n"<<COORDFlex[i][4]<<"\n"<<COORDFlex[i][5]<<"\n";
		//system("pause");
		i++;
	}
	GetLog()<<"\n"<<icount<<"\n";
	system("pause");
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
		mystepper->SetMaxiters(1000);
		mystepper->SetTolerance(5e-5);
		mystepper->Iterations=0;
	}
	outputfile = fopen("position.txt","w");
	double start = std::clock();
    double timestep = 0.00025;
    while (my_system.GetChTime() < 0.1) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
		
		//GetLog() << "N0: t=" << my_system.GetChTime() << "  x:"<<mnode0->GetPos().x << "  y:"<<mnode0->GetPos().y <<"  z:"<<mnode0->GetPos().z <<"  rx:"<<mnode0->GetD().x <<"  ry:"<<mnode0->GetD().y <<" rz:"<<mnode0->GetD().z<<" \n";
		//GetLog() << "N11: t=" << my_system.GetChTime() << "  x:"<<mnode11->GetPos().x << "  y:"<<mnode11->GetPos().y <<"  z:"<<mnode11->GetPos().z <<"  rx:"<<mnode11->GetD().x <<"  ry:"<<mnode11->GetD().y <<" rz:"<<mnode11->GetD().z<<" \n";
		//GetLog() << "N21: t=" << my_system.GetChTime() << "  x:"<<mnode21->GetPos().x << "  y:"<<mnode21->GetPos().y <<"  z:"<<mnode21->GetPos().z <<"  rx:"<<mnode21->GetD().x <<"  ry:"<<mnode21->GetD().y <<" rz:"<<mnode21->GetD().z<<" \n";
		//GetLog() << "N10: t=" << my_system.GetChTime() << "  x:"<<mnode10->GetPos().x << "  y:"<<mnode10->GetPos().y <<"  z:"<<mnode10->GetPos().z <<"  rx:"<<mnode10->GetD().x <<"  ry:"<<mnode10->GetD().y <<" rz:"<<mnode10->GetD().z<<" \n";

		fprintf(outputfile,"  %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"%e  ",nodetip->GetD().x);
		fprintf(outputfile,"%e  ",nodetip->GetD().y);
		fprintf(outputfile,"%e  ",nodetip->GetD().z);
		fprintf(outputfile,"\n  ");
		//system("pause");
		//GetLog()<< nodetip->GetPos().x<<"\n"<<nodetip->GetPos().y<<"\n"<<nodetip->GetPos().z<<"\n";
		//GetLog()<< nodetip->GetD().x<<"\n"<<nodetip->GetD().y<<"\n"<<nodetip->GetD().z<<"\n";
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	//GetLog() << "Iterations: " << my_system.Iter << "\n";
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}

void test_5() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: Brick24 FEM (10 elements),  implicit integration \n\n";

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
    int numDiv_z=0;
	int numNode=0;
	int dammy=0;
	int count=0;
	int MaxMNUM=1;
	int MTYPE=0;
	int MaxLayNum=0;
	int TotalNumNodes=0;
	char str1[100];

    double COORDFlex[1000][3];
	double VELCYFlex[1000][3];
	//double LayPROP[10][7][2];
	int LayNUM[1000];
	int NumNodes[1000][8];
	int NDR[1000][3];// constraint flag
	//int NumLayer[10];
	int MNUM[10][7];
	double MPROP[10][12];
	double ElemLengthXY[1000][3];//for brick is 3 (length along x,y,z)

	int MAXCOUNT = 100;
	inputfile = fopen("IndataSolid_Simple3.DAT","r");//now 10by1 brick
	printf("Open IndataSolid_Simple1.DAT\n");
	if(inputfile == NULL){
		printf("IndataSolid_Simple1.DAT cannot open!!\n");
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
	fscanf(inputfile,"%d %d %d %d %d\n",&numDiv,&numDiv_x,&numDiv_y,&numDiv_z,&numNode);
	fgets(str1,MAXCOUNT,inputfile);

	TotalNumNodes=numNode;

	printf("%s\n",str1);



	for(int i=0;i<numDiv;i++)
	{
	fscanf(inputfile,"%d %d %d %d %d %d %d %d %d %d %d\n",&count,&dammy,&LayNUM[i],&NumNodes[i][0],&NumNodes[i][1],&NumNodes[i][2],&NumNodes[i][3],&NumNodes[i][4],&NumNodes[i][5],&NumNodes[i][6],&NumNodes[i][7]);
	printf("LayNUM[i] %d\n  ",LayNUM[i]);

	fscanf(inputfile," %lf %lf %lf\n",&ElemLengthXY[i][0],&ElemLengthXY[i][1],&ElemLengthXY[i][2]);

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
	fscanf(inputfile,"%d %d %d %d\n",&count,&NDR[i][0],&NDR[i][1],&NDR[i][2]);
	fscanf(inputfile,"%lf %lf %lf\n",&COORDFlex[i][0],&COORDFlex[i][1],&COORDFlex[i][2]);
	fscanf(inputfile,"%lf %lf %lf\n",&VELCYFlex[i][0],&VELCYFlex[i][1],&VELCYFlex[i][2]);
	//printf("NumNodes %d %d %d\n",NDR[i][0],NDR[i][1],NDR[i][2]);
	//printf("NumNodes %lf %lf %lf\n",COORDFlex[i][0],COORDFlex[i][1],COORDFlex[i][2]);
	//system("pause");
	}

	//!--------------------------------------!
    //!--- Read Layer Data ------------------!
    //!--------------------------------------!
	////fscanf(inputfile,"%s\n",str1);
	//fgets(str1,MAXCOUNT,inputfile);
	//printf("%s\n",str1);
	//for(int i=0;i<MaxLayNum;i++)
	//{
	//	fscanf(inputfile,"%d %d\n",&count,&NumLayer[i]);
	//    for(int j=0;j<NumLayer[i];j++)
	//	{
	//	fscanf(inputfile,"%lf %lf %d\n",&LayPROP[i][j][0],&LayPROP[i][j][1],&MNUM[i][j]);
	//	if(MaxMNUM<MNUM[i][j])
	//	{MaxMNUM=MNUM[i][j];}
	//	//printf("%lf %lf %d\n",LayPROP[i][j][0],LayPROP[i][j][1],MNUM[i][j]);
	//	}
	//	//system("pause");
	//}

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
		fscanf(inputfile,"%lf %lf %lf\n",&MPROP[i][0],&MPROP[i][1],&MPROP[i][2]);
	}
	//if(MTYPE==2)
	//{
	//	fscanf(inputfile,"%lf %lf %lf %lf\n",&MPROP[i][0],&MPROP[i][1],&MPROP[i][2],&MPROP[i][3]);
	//	fscanf(inputfile,"%lf %lf %lf %lf %lf %lf\n",&MPROP[i][4],&MPROP[i][5],&MPROP[i][6],&MPROP[i][7],&MPROP[i][8],&MPROP[i][9]);
	//}
	//printf("%lf %lf %lf\n",MPROP[i][0],MPROP[i][1],MPROP[i][2]);
	}

	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_density(MPROP[0][0]);
	mmaterial->Set_E(MPROP[0][1]);
	mmaterial->Set_G(8.0769231E6);
	mmaterial->Set_v(MPROP[0][2]);
	mmaterial->Set_RayleighDampingK(0.0);
	mmaterial->Set_RayleighDampingM(0.0);

	int i=0;
	while(i<TotalNumNodes)
	{
		ChSharedPtr<ChNodeFEAxyz> node(new ChNodeFEAxyz(ChVector<>(COORDFlex[i][0], COORDFlex[i][1], COORDFlex[i][2])));
		//GetLog() << "TotalNumNodes" << TotalNumNodes << "\n\n";
		node->SetMass(0.0);
		my_mesh->AddNode(node);
		if(NDR[i][0]==1&&NDR[i][1]==1&&NDR[i][2]==1)
		{
			node->SetFixed(true);
		}
		i++;
	}
	GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";

	//---print tip node displacement---//
	ChSharedPtr<ChNodeFEAxyz> nodetip (my_mesh->GetNode(TotalNumNodes-1).DynamicCastTo<ChNodeFEAxyz>());
	GetLog() << "X : " << nodetip->GetPos().x <<" Y : "<< nodetip->GetPos().y <<" Z : "<< nodetip->GetPos().z <<"\n\n";


    int elemcount=0;
	while(elemcount<numDiv)
	{
		ChSharedPtr<ChElementBrick> element(new ChElementBrick);

		ChMatrixNM<double,3,1> InertFlexVec;// read element length, used in ChElementBrick 
	    InertFlexVec.Reset();
	    InertFlexVec(0)=ElemLengthXY[elemcount][0];
	    InertFlexVec(1)=ElemLengthXY[elemcount][1];
	    InertFlexVec(2)=ElemLengthXY[elemcount][2];
		element->SetInertFlexVec(InertFlexVec);

		element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][1]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][2]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][3]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][4]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][5]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][6]-1).DynamicCastTo<ChNodeFEAxyz>(),my_mesh->GetNode(NumNodes[elemcount][7]-1).DynamicCastTo<ChNodeFEAxyz>());
		element->SetMaterial(mmaterial);
	    element->SetElemNum(elemcount);//for EAS
	    ChMatrixNM<double,9,1> stock_alpha_EAS; // 
		stock_alpha_EAS.Reset();
		element->SetStockAlpha(stock_alpha_EAS(0,0),stock_alpha_EAS(1,0),stock_alpha_EAS(2,0),stock_alpha_EAS(3,0),stock_alpha_EAS(4,0),stock_alpha_EAS(5,0),stock_alpha_EAS(6,0),stock_alpha_EAS(7,0),stock_alpha_EAS(8,0));
		my_mesh->AddElement(element);
		elemcount++;
    }

    // This is mandatory
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

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
		mystepper->SetTolerance(1e-5);
		mystepper->Iterations=0;
	}
	my_system.Iter=0;
	outputfile = fopen("position.txt","w");
	double start = std::clock();

    double timestep = 0.001;
    while (my_system.GetChTime() < 2.0) {
        my_system.DoStepDynamics(timestep);
		GetLog() << " t=  " << my_system.GetChTime() << "\n";
		
		//GetLog() << "N0: t=" << my_system.GetChTime() << "  x:"<<mnode0->GetPos().x << "  y:"<<mnode0->GetPos().y <<"  z:"<<mnode0->GetPos().z <<"  rx:"<<mnode0->GetD().x <<"  ry:"<<mnode0->GetD().y <<" rz:"<<mnode0->GetD().z<<" \n";
		//GetLog() << "N11: t=" << my_system.GetChTime() << "  x:"<<mnode11->GetPos().x << "  y:"<<mnode11->GetPos().y <<"  z:"<<mnode11->GetPos().z <<"  rx:"<<mnode11->GetD().x <<"  ry:"<<mnode11->GetD().y <<" rz:"<<mnode11->GetD().z<<" \n";
		//GetLog() << "N21: t=" << my_system.GetChTime() << "  x:"<<mnode21->GetPos().x << "  y:"<<mnode21->GetPos().y <<"  z:"<<mnode21->GetPos().z <<"  rx:"<<mnode21->GetD().x <<"  ry:"<<mnode21->GetD().y <<" rz:"<<mnode21->GetD().z<<" \n";
		//GetLog() << "N10: t=" << my_system.GetChTime() << "  x:"<<mnode10->GetPos().x << "  y:"<<mnode10->GetPos().y <<"  z:"<<mnode10->GetPos().z <<"  rx:"<<mnode10->GetD().x <<"  ry:"<<mnode10->GetD().y <<" rz:"<<mnode10->GetD().z<<" \n";

		fprintf(outputfile," %e  ",my_system.GetChTime());
		fprintf(outputfile,"%e  ",nodetip->GetPos().x);
		fprintf(outputfile,"%e  ",nodetip->GetPos().y);
		fprintf(outputfile,"%e  ",nodetip->GetPos().z);
		fprintf(outputfile,"\n  ");
		//system("pause");
    }
	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	//GetLog() << "Iterations: " << my_system.Iter << "\n";
	ChSharedPtr<ChTimestepperHHT> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
	//ChSharedPtr<ChTimestepperNewmark> mystepper1 = my_system.GetTimestepper().DynamicCastTo<ChTimestepperNewmark>();
	GetLog() << "Iterations: " << mystepper1->Iterations << "\n";
	GetLog() << "Simulation Time: " << duration << "\n";
}
// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    // Test: an introductory problem:
	//test_1() : ANCF Shell Simple Case (Constraints)
	//test_2() : ANCF Tire (Constraints)
	//test_3() : ANCF Shell Simple Case (Fixed)
	//test_4() : ANCF Tire (Fixed)
	//test_5() : ANCF Brick Simple Case (Fixed)
    test_3();
	
    system("pause");
    return 0;
}