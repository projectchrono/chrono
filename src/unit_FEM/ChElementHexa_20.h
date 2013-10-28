#ifndef CHELEMENTHEXA20_H
#define CHELEMENTHEXA20_H

//////////////////////////////////////////////////
//  
//   ChElementHexa_20.h
//
//   Class for hexaedrons 20 nodes
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright: 
//
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChElement3D.h"
#include "ChNodeFEMxyz.h"

namespace chrono
{
namespace fem
{

class ChApiFem ChElementHexa_20 : public ChHexahedron
{
protected:
		std::vector<ChNodeFEMxyz*> nodes;
		ChSharedPtr<ChContinuumElastic> Material;
		//std::vector< ChMatrixDynamic<> > MatrB;		// matrices of shape function's partial derivatives (one for each integration point)
													// we use a vector to keep in memory all the 27 matrices (-> 27 integr. point)
													// NO! each matrix is stored in the respective gauss point
		ChMatrixDynamic<> StiffnessMatrix;
	
public:

	ChElementHexa_20();
	virtual ~ChElementHexa_20();

	virtual int GetNcoords() {return 60;}
	virtual int GetNnodes()  {return 20;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB, ChNodeFEMxyz* nodeC, ChNodeFEMxyz* nodeD,
						  ChNodeFEMxyz* nodeE, ChNodeFEMxyz* nodeF, ChNodeFEMxyz* nodeG, ChNodeFEMxyz* nodeH,
						  ChNodeFEMxyz* nodeI, ChNodeFEMxyz* nodeJ, ChNodeFEMxyz* nodeK, ChNodeFEMxyz* nodeL,
						  ChNodeFEMxyz* nodeM, ChNodeFEMxyz* nodeN, ChNodeFEMxyz* nodeO, ChNodeFEMxyz* nodeP,
						  ChNodeFEMxyz* nodeQ, ChNodeFEMxyz* nodeR, ChNodeFEMxyz* nodeS, ChNodeFEMxyz* nodeT) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					nodes[2]=nodeC;
					nodes[3]=nodeD;
					nodes[4]=nodeE;
					nodes[5]=nodeF;
					nodes[6]=nodeG;
					nodes[7]=nodeH;
					nodes[8]=nodeI;
					nodes[9]=nodeJ;
					nodes[10]=nodeK;
					nodes[11]=nodeL;
					nodes[12]=nodeM;
					nodes[13]=nodeN;
					nodes[14]=nodeO;
					nodes[15]=nodeP;
					nodes[16]=nodeQ;
					nodes[17]=nodeR;
					nodes[18]=nodeS;
					nodes[19]=nodeT;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					mvars.push_back(&nodes[2]->Variables());
					mvars.push_back(&nodes[3]->Variables());
					mvars.push_back(&nodes[4]->Variables());
					mvars.push_back(&nodes[5]->Variables());
					mvars.push_back(&nodes[6]->Variables());
					mvars.push_back(&nodes[7]->Variables());
					mvars.push_back(&nodes[8]->Variables());
					mvars.push_back(&nodes[9]->Variables());
					mvars.push_back(&nodes[10]->Variables());
					mvars.push_back(&nodes[11]->Variables());
					mvars.push_back(&nodes[12]->Variables());
					mvars.push_back(&nodes[13]->Variables());
					mvars.push_back(&nodes[14]->Variables());
					mvars.push_back(&nodes[15]->Variables());
					mvars.push_back(&nodes[16]->Variables());
					mvars.push_back(&nodes[17]->Variables());
					mvars.push_back(&nodes[18]->Variables());
					mvars.push_back(&nodes[19]->Variables());
					Kmatr.SetVariables(mvars);
				}


			//
			// QUADRATURE functions
			//

	virtual void SetDefaultIntegrationRule()
			{
				this->ir->SetIntOnCube(27, &this->GpVector);
			}

	virtual void SetReducedIntegrationRule()
			{
				this->ir->SetIntOnCube(8, &this->GpVector);
			}

	virtual void SetIntegrationRule(int nPoints)
			{
				this->ir->SetIntOnCube(nPoints, &this->GpVector);
			}

			
			//
			// FEM functions
			//
	
				/// Puts inside 'Jacobian' and 'J1' the Jacobian matrix and the shape functions derivatives matrix of the element
				/// The vector "coord" contains the natural coordinates of the integration point
				/// in case of hexahedral elements natural coords vary in the classical range -1 ... +1
	virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, ChMatrixDynamic<>& J1, ChVector<> coord) 
				{
					ChMatrixDynamic<> J2(20,3);

					J1.SetElement(0,0,-(1-coord.y)*(1-coord.z)*(-1-2*coord.x-coord.y-coord.z)/8);
					J1.SetElement(0,1,+(1-coord.y)*(1-coord.z)*(-1+2*coord.x-coord.y-coord.z)/8);
					J1.SetElement(0,2,+(1+coord.y)*(1-coord.z)*(-1+2*coord.x+coord.y-coord.z)/8);
					J1.SetElement(0,3,-(1+coord.y)*(1-coord.z)*(-1-2*coord.x+coord.y-coord.z)/8);
					J1.SetElement(0,4,-(1-coord.y)*(1+coord.z)*(-1-2*coord.x-coord.y+coord.z)/8);
					J1.SetElement(0,5,+(1-coord.y)*(1+coord.z)*(-1+2*coord.x-coord.y+coord.z)/8);
					J1.SetElement(0,6,+(1+coord.y)*(1+coord.z)*(-1+2*coord.x+coord.y+coord.z)/8);
					J1.SetElement(0,7,-(1+coord.y)*(1+coord.z)*(-1-2*coord.x+coord.y+coord.z)/8);
					J1.SetElement(0,8, coord.x*(1-coord.y)*(1-coord.z)/(-2));
					J1.SetElement(0,9,+(1-coord.y*coord.y)*(1-coord.z)/4);
					J1.SetElement(0,10, coord.x*(1+coord.y)*(1-coord.z)/(-2));
					J1.SetElement(0,11,-(1-coord.y*coord.y)*(1-coord.z)/4);
					J1.SetElement(0,12, coord.x*(1-coord.y)*(1+coord.z)/(-2));
					J1.SetElement(0,13,+(1-coord.y*coord.y)*(1+coord.z)/4);
					J1.SetElement(0,14, coord.x*(1+coord.y)*(1+coord.z)/(-2));
					J1.SetElement(0,15,-(1-coord.y*coord.y)*(1+coord.z)/4);
					J1.SetElement(0,16,+(1-coord.y)*(1-coord.z*coord.z)/4);
					J1.SetElement(0,17,+(1+coord.y)*(1-coord.z*coord.z)/4);
					J1.SetElement(0,18,-(1+coord.y)*(1-coord.z*coord.z)/4);
					J1.SetElement(0,19,-(1-coord.y)*(1-coord.z*coord.z)/4);

					J1.SetElement(1,0,-(1-coord.x)*(1-coord.z)*(-1-coord.x-2*coord.y-coord.z)/8);
					J1.SetElement(1,1,-(1+coord.x)*(1-coord.z)*(-1+coord.x-2*coord.y-coord.z)/8);
					J1.SetElement(1,2,+(1+coord.x)*(1-coord.z)*(-1+coord.x+2*coord.y-coord.z)/8);
					J1.SetElement(1,3,+(1-coord.x)*(1-coord.z)*(-1-coord.x+2*coord.y-coord.z)/8);
					J1.SetElement(1,4,-(1-coord.x)*(1+coord.z)*(-1-coord.x-2*coord.y+coord.z)/8);
					J1.SetElement(1,5,-(1+coord.x)*(1+coord.z)*(-1+coord.x-2*coord.y+coord.z)/8);
					J1.SetElement(1,6,+(1+coord.x)*(1+coord.z)*(-1+coord.x+2*coord.y+coord.z)/8);
					J1.SetElement(1,7,+(1-coord.x)*(1+coord.z)*(-1-coord.x+2*coord.y+coord.z)/8);
					J1.SetElement(1,8,-(1-coord.x*coord.x)*(1-coord.z)/4);
					J1.SetElement(1,9, coord.y*(1+coord.x)*(1-coord.z)/(-2));
					J1.SetElement(1,10,+(1-coord.x*coord.x)*(1-coord.z)/4);
					J1.SetElement(1,11, coord.y*(1-coord.x)*(1-coord.z)/(-2));
					J1.SetElement(1,12,-(1-coord.x*coord.x)*(1+coord.z)/4);
					J1.SetElement(1,13, coord.y*(1+coord.x)*(1+coord.z)/(-2));
					J1.SetElement(1,14,+(1-coord.x*coord.x)*(1+coord.z)/4);
					J1.SetElement(1,15, coord.y*(1-coord.x)*(1+coord.z)/(-2));
					J1.SetElement(1,16,-(1+coord.x)*(1-coord.z*coord.z)/4);
					J1.SetElement(1,17,+(1+coord.x)*(1-coord.z*coord.z)/4);
					J1.SetElement(1,18,+(1-coord.x)*(1-coord.z*coord.z)/4);
					J1.SetElement(1,19,-(1-coord.x)*(1-coord.z*coord.z)/4);

					J1.SetElement(2,0,-(1-coord.x)*(1-coord.y)*(-1-coord.x-coord.y-2*coord.z)/8);
					J1.SetElement(2,1,-(1+coord.x)*(1-coord.y)*(-1+coord.x-coord.y-2*coord.z)/8);
					J1.SetElement(2,2,-(1+coord.x)*(1+coord.y)*(-1+coord.x+coord.y-2*coord.z)/8);
					J1.SetElement(2,3,-(1-coord.x)*(1+coord.y)*(-1-coord.x+coord.y-2*coord.z)/8);
					J1.SetElement(2,4,+(1-coord.x)*(1-coord.y)*(-1-coord.x-coord.y+2*coord.z)/8);
					J1.SetElement(2,5,+(1+coord.x)*(1-coord.y)*(-1+coord.x-coord.y+2*coord.z)/8);
					J1.SetElement(2,6,+(1+coord.x)*(1+coord.y)*(-1+coord.x+coord.y+2*coord.z)/8);
					J1.SetElement(2,7,+(1-coord.x)*(1+coord.y)*(-1-coord.x+coord.y+2*coord.z)/8);
					J1.SetElement(2,8,-(1-coord.x*coord.x)*(1-coord.y)/4);
					J1.SetElement(2,9,-(1+coord.x)*(1-coord.y*coord.y)/4);
					J1.SetElement(2,10,-(1-coord.x*coord.x)*(1+coord.y)/4);
					J1.SetElement(2,11,-(1-coord.x)*(1-coord.y*coord.y)/4);
					J1.SetElement(2,12,+(1-coord.x*coord.x)*(1-coord.y)/4);
					J1.SetElement(2,13,+(1+coord.x)*(1-coord.y*coord.y)/4);
					J1.SetElement(2,14,+(1-coord.x*coord.x)*(1+coord.y)/4);
					J1.SetElement(2,15,+(1-coord.x)*(1-coord.y*coord.y)/4);
					J1.SetElement(2,16,coord.z*(1+coord.x)*(1-coord.y)/(-2));
					J1.SetElement(2,17,coord.z*(1+coord.x)*(1+coord.y)/(-2));
					J1.SetElement(2,18,coord.z*(1-coord.x)*(1+coord.y)/(-2));
					J1.SetElement(2,19,coord.z*(1-coord.x)*(1-coord.y)/(-2));
					
					
					J2.SetElement(0,0,nodes[0]->pos.x);
					J2.SetElement(1,0,nodes[1]->pos.x);
					J2.SetElement(2,0,nodes[2]->pos.x);
					J2.SetElement(3,0,nodes[3]->pos.x);
					J2.SetElement(4,0,nodes[4]->pos.x);
					J2.SetElement(5,0,nodes[5]->pos.x);
					J2.SetElement(6,0,nodes[6]->pos.x);
					J2.SetElement(7,0,nodes[7]->pos.x);
					J2.SetElement(8,0,nodes[8]->pos.x);
					J2.SetElement(9,0,nodes[9]->pos.x);
					J2.SetElement(10,0,nodes[10]->pos.x);
					J2.SetElement(11,0,nodes[11]->pos.x);
					J2.SetElement(12,0,nodes[12]->pos.x);
					J2.SetElement(13,0,nodes[13]->pos.x);
					J2.SetElement(14,0,nodes[14]->pos.x);
					J2.SetElement(15,0,nodes[15]->pos.x);
					J2.SetElement(16,0,nodes[16]->pos.x);
					J2.SetElement(17,0,nodes[17]->pos.x);
					J2.SetElement(18,0,nodes[18]->pos.x);
					J2.SetElement(19,0,nodes[19]->pos.x);
	
					J2.SetElement(0,1,nodes[0]->pos.y);
					J2.SetElement(1,1,nodes[1]->pos.y);
					J2.SetElement(2,1,nodes[2]->pos.y);
					J2.SetElement(3,1,nodes[3]->pos.y);
					J2.SetElement(4,1,nodes[4]->pos.y);
					J2.SetElement(5,1,nodes[5]->pos.y);
					J2.SetElement(6,1,nodes[6]->pos.y);
					J2.SetElement(7,1,nodes[7]->pos.y);
					J2.SetElement(8,1,nodes[8]->pos.y);
					J2.SetElement(9,1,nodes[9]->pos.y);
					J2.SetElement(10,1,nodes[10]->pos.y);
					J2.SetElement(11,1,nodes[11]->pos.y);
					J2.SetElement(12,1,nodes[12]->pos.y);
					J2.SetElement(13,1,nodes[13]->pos.y);
					J2.SetElement(14,1,nodes[14]->pos.y);
					J2.SetElement(15,1,nodes[15]->pos.y);
					J2.SetElement(16,1,nodes[16]->pos.y);
					J2.SetElement(17,1,nodes[17]->pos.y);
					J2.SetElement(18,1,nodes[18]->pos.y);
					J2.SetElement(19,1,nodes[19]->pos.y);

					J2.SetElement(0,2,nodes[0]->pos.z);
					J2.SetElement(1,2,nodes[1]->pos.z);
					J2.SetElement(2,2,nodes[2]->pos.z);
					J2.SetElement(3,2,nodes[3]->pos.z);
					J2.SetElement(4,2,nodes[4]->pos.z);
					J2.SetElement(5,2,nodes[5]->pos.z);
					J2.SetElement(6,2,nodes[6]->pos.z);
					J2.SetElement(7,2,nodes[7]->pos.z);
					J2.SetElement(8,2,nodes[8]->pos.z);
					J2.SetElement(9,2,nodes[9]->pos.z);
					J2.SetElement(10,2,nodes[10]->pos.z);
					J2.SetElement(11,2,nodes[11]->pos.z);
					J2.SetElement(12,2,nodes[12]->pos.z);
					J2.SetElement(13,2,nodes[13]->pos.z);
					J2.SetElement(14,2,nodes[14]->pos.z);
					J2.SetElement(15,2,nodes[15]->pos.z);
					J2.SetElement(16,2,nodes[16]->pos.z);
					J2.SetElement(17,2,nodes[17]->pos.z);
					J2.SetElement(18,2,nodes[18]->pos.z);
					J2.SetElement(19,2,nodes[19]->pos.z);

					Jacobian.MatrMultiply(J1,J2);				
				}


				/// Computes the matrix of partial derivatives and puts data in "GaussPt"
				///	Stores the determinant of the jacobian in "JacobianDet"
	virtual void ComputeMatrB(ChGaussPoint* GaussPt, double& JacobianDet) 
				{
					ChMatrixDynamic<> Jacobian(3,3);
					ChMatrixDynamic<> J1(3,20);
					ComputeJacobian(Jacobian, J1, (*GaussPt).GetLocalCoordinates());
				
					double Jdet=Jacobian.Det();
					JacobianDet = Jdet;		// !!! store the Jacobian Determinant: needed for the integration

					ChMatrixDynamic<> Jinv = Jacobian;
					Jinv.MatrInverse();

					ChMatrixDynamic<> Btemp(3,20);
					Btemp.MatrMultiply(Jinv,J1);
					GaussPt->MatrB->Resize(6,60);	// Remember to resize the matrix!

					GaussPt->MatrB->SetElement(0,0,Btemp(0,0));
					GaussPt->MatrB->SetElement(0,3,Btemp(0,1));
					GaussPt->MatrB->SetElement(0,6,Btemp(0,2));
					GaussPt->MatrB->SetElement(0,9,Btemp(0,3));
					GaussPt->MatrB->SetElement(0,12,Btemp(0,4));
					GaussPt->MatrB->SetElement(0,15,Btemp(0,5));
					GaussPt->MatrB->SetElement(0,18,Btemp(0,6));
					GaussPt->MatrB->SetElement(0,21,Btemp(0,7));
					GaussPt->MatrB->SetElement(0,24,Btemp(0,8));
					GaussPt->MatrB->SetElement(0,27,Btemp(0,9));
					GaussPt->MatrB->SetElement(0,30,Btemp(0,10));
					GaussPt->MatrB->SetElement(0,33,Btemp(0,11));
					GaussPt->MatrB->SetElement(0,36,Btemp(0,12));
					GaussPt->MatrB->SetElement(0,39,Btemp(0,13));
					GaussPt->MatrB->SetElement(0,42,Btemp(0,14));
					GaussPt->MatrB->SetElement(0,45,Btemp(0,15));
					GaussPt->MatrB->SetElement(0,48,Btemp(0,16));
					GaussPt->MatrB->SetElement(0,51,Btemp(0,17));
					GaussPt->MatrB->SetElement(0,54,Btemp(0,18));
					GaussPt->MatrB->SetElement(0,57,Btemp(0,19));

					GaussPt->MatrB->SetElement(1,1,Btemp(1,0));
					GaussPt->MatrB->SetElement(1,4,Btemp(1,1));
					GaussPt->MatrB->SetElement(1,7,Btemp(1,2));
					GaussPt->MatrB->SetElement(1,10,Btemp(1,3));
					GaussPt->MatrB->SetElement(1,13,Btemp(1,4));
					GaussPt->MatrB->SetElement(1,16,Btemp(1,5));
					GaussPt->MatrB->SetElement(1,19,Btemp(1,6));
					GaussPt->MatrB->SetElement(1,22,Btemp(1,7));
					GaussPt->MatrB->SetElement(1,25,Btemp(1,8));
					GaussPt->MatrB->SetElement(1,28,Btemp(1,9));
					GaussPt->MatrB->SetElement(1,31,Btemp(1,10));
					GaussPt->MatrB->SetElement(1,34,Btemp(1,11));
					GaussPt->MatrB->SetElement(1,37,Btemp(1,12));
					GaussPt->MatrB->SetElement(1,40,Btemp(1,13));
					GaussPt->MatrB->SetElement(1,43,Btemp(1,14));
					GaussPt->MatrB->SetElement(1,46,Btemp(1,15));
					GaussPt->MatrB->SetElement(1,49,Btemp(1,16));
					GaussPt->MatrB->SetElement(1,52,Btemp(1,17));
					GaussPt->MatrB->SetElement(1,55,Btemp(1,18));
					GaussPt->MatrB->SetElement(1,58,Btemp(1,19));

					GaussPt->MatrB->SetElement(2,2,Btemp(2,0));
					GaussPt->MatrB->SetElement(2,5,Btemp(2,1));
					GaussPt->MatrB->SetElement(2,8,Btemp(2,2));
					GaussPt->MatrB->SetElement(2,11,Btemp(2,3));
					GaussPt->MatrB->SetElement(2,14,Btemp(2,4));
					GaussPt->MatrB->SetElement(2,17,Btemp(2,5));
					GaussPt->MatrB->SetElement(2,20,Btemp(2,6));
					GaussPt->MatrB->SetElement(2,23,Btemp(2,7));
					GaussPt->MatrB->SetElement(2,26,Btemp(2,8));
					GaussPt->MatrB->SetElement(2,29,Btemp(2,9));
					GaussPt->MatrB->SetElement(2,32,Btemp(2,10));
					GaussPt->MatrB->SetElement(2,35,Btemp(2,11));
					GaussPt->MatrB->SetElement(2,38,Btemp(2,12));
					GaussPt->MatrB->SetElement(2,41,Btemp(2,13));
					GaussPt->MatrB->SetElement(2,44,Btemp(2,14));
					GaussPt->MatrB->SetElement(2,47,Btemp(2,15));
					GaussPt->MatrB->SetElement(2,50,Btemp(2,16));
					GaussPt->MatrB->SetElement(2,53,Btemp(2,17));
					GaussPt->MatrB->SetElement(2,56,Btemp(2,18));
					GaussPt->MatrB->SetElement(2,59,Btemp(2,19));

					GaussPt->MatrB->SetElement(3,0,Btemp(1,0));
					GaussPt->MatrB->SetElement(3,1,Btemp(0,0));
					GaussPt->MatrB->SetElement(3,3,Btemp(1,1));
					GaussPt->MatrB->SetElement(3,4,Btemp(0,1));
					GaussPt->MatrB->SetElement(3,6,Btemp(1,2));
					GaussPt->MatrB->SetElement(3,7,Btemp(0,2));
					GaussPt->MatrB->SetElement(3,9,Btemp(1,3));
					GaussPt->MatrB->SetElement(3,10,Btemp(0,3));
					GaussPt->MatrB->SetElement(3,12,Btemp(1,4));
					GaussPt->MatrB->SetElement(3,13,Btemp(0,4));
					GaussPt->MatrB->SetElement(3,15,Btemp(1,5));
					GaussPt->MatrB->SetElement(3,16,Btemp(0,5));
					GaussPt->MatrB->SetElement(3,18,Btemp(1,6));
					GaussPt->MatrB->SetElement(3,19,Btemp(0,6));
					GaussPt->MatrB->SetElement(3,21,Btemp(1,7));
					GaussPt->MatrB->SetElement(3,22,Btemp(0,7));
					GaussPt->MatrB->SetElement(3,24,Btemp(1,8));
					GaussPt->MatrB->SetElement(3,25,Btemp(0,8));
					GaussPt->MatrB->SetElement(3,27,Btemp(1,9));
					GaussPt->MatrB->SetElement(3,28,Btemp(0,9));
					GaussPt->MatrB->SetElement(3,30,Btemp(1,10));
					GaussPt->MatrB->SetElement(3,31,Btemp(0,10));
					GaussPt->MatrB->SetElement(3,33,Btemp(1,11));
					GaussPt->MatrB->SetElement(3,34,Btemp(0,11));
					GaussPt->MatrB->SetElement(3,36,Btemp(1,12));
					GaussPt->MatrB->SetElement(3,37,Btemp(0,12));
					GaussPt->MatrB->SetElement(3,39,Btemp(1,13));
					GaussPt->MatrB->SetElement(3,40,Btemp(0,13));
					GaussPt->MatrB->SetElement(3,42,Btemp(1,14));
					GaussPt->MatrB->SetElement(3,43,Btemp(0,14));
					GaussPt->MatrB->SetElement(3,45,Btemp(1,15));
					GaussPt->MatrB->SetElement(3,46,Btemp(0,15));
					GaussPt->MatrB->SetElement(3,48,Btemp(1,16));
					GaussPt->MatrB->SetElement(3,49,Btemp(0,16));
					GaussPt->MatrB->SetElement(3,51,Btemp(1,17));
					GaussPt->MatrB->SetElement(3,52,Btemp(0,17));
					GaussPt->MatrB->SetElement(3,54,Btemp(1,18));
					GaussPt->MatrB->SetElement(3,55,Btemp(0,18));
					GaussPt->MatrB->SetElement(3,57,Btemp(1,19));
					GaussPt->MatrB->SetElement(3,58,Btemp(0,19));

					GaussPt->MatrB->SetElement(4,1,Btemp(2,0));
					GaussPt->MatrB->SetElement(4,2,Btemp(1,0));
					GaussPt->MatrB->SetElement(4,4,Btemp(2,1));
					GaussPt->MatrB->SetElement(4,5,Btemp(1,1));
					GaussPt->MatrB->SetElement(4,7,Btemp(2,2));
					GaussPt->MatrB->SetElement(4,8,Btemp(1,2));
					GaussPt->MatrB->SetElement(4,10,Btemp(2,3));
					GaussPt->MatrB->SetElement(4,11,Btemp(1,3));
					GaussPt->MatrB->SetElement(4,13,Btemp(2,4));
					GaussPt->MatrB->SetElement(4,14,Btemp(1,4));
					GaussPt->MatrB->SetElement(4,16,Btemp(2,5));
					GaussPt->MatrB->SetElement(4,17,Btemp(1,5));
					GaussPt->MatrB->SetElement(4,19,Btemp(2,6));
					GaussPt->MatrB->SetElement(4,20,Btemp(1,6));
					GaussPt->MatrB->SetElement(4,22,Btemp(2,7));
					GaussPt->MatrB->SetElement(4,23,Btemp(1,7));
					GaussPt->MatrB->SetElement(4,25,Btemp(2,8));
					GaussPt->MatrB->SetElement(4,26,Btemp(1,8));
					GaussPt->MatrB->SetElement(4,28,Btemp(2,9));
					GaussPt->MatrB->SetElement(4,29,Btemp(1,9));
					GaussPt->MatrB->SetElement(4,31,Btemp(2,10));
					GaussPt->MatrB->SetElement(4,32,Btemp(1,10));
					GaussPt->MatrB->SetElement(4,34,Btemp(2,11));
					GaussPt->MatrB->SetElement(4,35,Btemp(1,11));
					GaussPt->MatrB->SetElement(4,37,Btemp(2,12));
					GaussPt->MatrB->SetElement(4,38,Btemp(1,12));
					GaussPt->MatrB->SetElement(4,40,Btemp(2,13));
					GaussPt->MatrB->SetElement(4,41,Btemp(1,13));
					GaussPt->MatrB->SetElement(4,43,Btemp(2,14));
					GaussPt->MatrB->SetElement(4,44,Btemp(1,14));
					GaussPt->MatrB->SetElement(4,46,Btemp(2,15));
					GaussPt->MatrB->SetElement(4,47,Btemp(1,15));
					GaussPt->MatrB->SetElement(4,49,Btemp(2,16));
					GaussPt->MatrB->SetElement(4,50,Btemp(1,16));
					GaussPt->MatrB->SetElement(4,52,Btemp(2,17));
					GaussPt->MatrB->SetElement(4,53,Btemp(1,17));
					GaussPt->MatrB->SetElement(4,55,Btemp(2,18));
					GaussPt->MatrB->SetElement(4,56,Btemp(1,18));
					GaussPt->MatrB->SetElement(4,58,Btemp(2,19));
					GaussPt->MatrB->SetElement(4,59,Btemp(1,19));

					GaussPt->MatrB->SetElement(5,0,Btemp(2,0));
					GaussPt->MatrB->SetElement(5,2,Btemp(0,0));
					GaussPt->MatrB->SetElement(5,3,Btemp(2,1));
					GaussPt->MatrB->SetElement(5,5,Btemp(0,1));
					GaussPt->MatrB->SetElement(5,6,Btemp(2,2));
					GaussPt->MatrB->SetElement(5,8,Btemp(0,2));
					GaussPt->MatrB->SetElement(5,9,Btemp(2,3));
					GaussPt->MatrB->SetElement(5,11,Btemp(0,3));
					GaussPt->MatrB->SetElement(5,12,Btemp(2,4));
					GaussPt->MatrB->SetElement(5,14,Btemp(0,4));
					GaussPt->MatrB->SetElement(5,15,Btemp(2,5));
					GaussPt->MatrB->SetElement(5,17,Btemp(0,5));
					GaussPt->MatrB->SetElement(5,18,Btemp(2,6));
					GaussPt->MatrB->SetElement(5,20,Btemp(0,6));
					GaussPt->MatrB->SetElement(5,21,Btemp(2,7));
					GaussPt->MatrB->SetElement(5,23,Btemp(0,7));
					GaussPt->MatrB->SetElement(5,24,Btemp(2,8));
					GaussPt->MatrB->SetElement(5,26,Btemp(0,8));
					GaussPt->MatrB->SetElement(5,27,Btemp(2,9));
					GaussPt->MatrB->SetElement(5,29,Btemp(0,9));
					GaussPt->MatrB->SetElement(5,30,Btemp(2,10));
					GaussPt->MatrB->SetElement(5,32,Btemp(0,10));
					GaussPt->MatrB->SetElement(5,33,Btemp(2,11));
					GaussPt->MatrB->SetElement(5,35,Btemp(0,11));
					GaussPt->MatrB->SetElement(5,36,Btemp(2,12));
					GaussPt->MatrB->SetElement(5,38,Btemp(0,12));
					GaussPt->MatrB->SetElement(5,39,Btemp(2,13));
					GaussPt->MatrB->SetElement(5,41,Btemp(0,13));
					GaussPt->MatrB->SetElement(5,42,Btemp(2,14));
					GaussPt->MatrB->SetElement(5,44,Btemp(0,14));
					GaussPt->MatrB->SetElement(5,45,Btemp(2,15));
					GaussPt->MatrB->SetElement(5,47,Btemp(0,15));
					GaussPt->MatrB->SetElement(5,48,Btemp(2,16));
					GaussPt->MatrB->SetElement(5,50,Btemp(0,16));
					GaussPt->MatrB->SetElement(5,51,Btemp(2,17));
					GaussPt->MatrB->SetElement(5,53,Btemp(0,17));
					GaussPt->MatrB->SetElement(5,54,Btemp(2,18));
					GaussPt->MatrB->SetElement(5,56,Btemp(0,18));
					GaussPt->MatrB->SetElement(5,57,Btemp(2,19));
					GaussPt->MatrB->SetElement(5,59,Btemp(0,19));


			}

				/// Computes the global STIFFNESS MATRIX of the element:    
				/// K = Volume * [B]' * [D] * [B]
				/// The number of Gauss Point is defined by SetIntegrationRule function (default: 27 Gp)
	virtual void ComputeStiffnessMatrix() 
		{
			double Jdet;
			ChMatrixDynamic<> *temp = new ChMatrixDynamic<>;
			ChMatrixDynamic<> BT;

			for(int i=0; i < GpVector.size(); i++)
			{
				ComputeMatrB(GpVector[i], Jdet);
				BT = *GpVector[i]->MatrB;
				BT.MatrTranspose();
				*temp = (BT * Material->Get_StressStrainMatrix() * *(GpVector[i]->MatrB));
				temp->MatrScale(GpVector[i]->GetWeight());
				temp->MatrScale(Jdet);
				StiffnessMatrix.MatrAdd(StiffnessMatrix,*temp);

			}
			delete temp;
		}

//////////////////// *** OLD METHOD (before GaussIntegrationRule) *** //////////////////////
				/// Computes the global STIFFNESS MATRIX of the element:    
				/// K = Volume * [B]' * [D] * [B]
				/// 
/*	virtual void ComputeStiffnessMatrix() 
			{
				//========================
				//Exact Integration (27 Gp)
				//========================
					double zeta1, zeta2, zeta3;
					double Jdet;
					ChMatrixDynamic<> temp;
					ChMatrixDynamic<> BT;

				//////////////////////////////////
				/// Reduced Integration (8 Gp) ///
				//////////////////////////////////
			
					zeta1=0.577350269189626;
					zeta2=0.577350269189626;
					zeta3=0.577350269189626;
					
					ComputeMatrB(0, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[0];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[0]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					this->StiffnessMatrix = temp;

					zeta1=-0.577350269189626;
					zeta2=0.577350269189626;
					zeta3=0.577350269189626;

					ComputeMatrB(1, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[1];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[1]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

					zeta1=0.577350269189626;
					zeta2=-0.577350269189626;
					zeta3=0.577350269189626;

					ComputeMatrB(2, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[2];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[2]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

					zeta1=0.577350269189626;
					zeta2=0.577350269189626;
					zeta3=-0.577350269189626;

					ComputeMatrB(3, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[3];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[3]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

					zeta1=-0.577350269189626;
					zeta2=-0.577350269189626;
					zeta3=0.577350269189626;

					ComputeMatrB(4, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[4];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[4]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

					zeta1=-0.577350269189626;
					zeta2=0.577350269189626;
					zeta3=-0.577350269189626;

					ComputeMatrB(5, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[5];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[5]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

					zeta1=0.577350269189626;
					zeta2=-0.577350269189626;
					zeta3=-0.577350269189626;

					ComputeMatrB(6, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[6];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[6]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);
	
					zeta1=-0.577350269189626;
					zeta2=-0.577350269189626;
					zeta3=-0.577350269189626;

					ComputeMatrB(7, zeta1, zeta2, zeta3, Jdet);
					BT=MatrB[7];
					BT.MatrTranspose();
					temp = (BT * Material->Get_StressStrainMatrix() * MatrB[7]);
					temp.MatrScale(Jdet);
					//Gauss integration weight = 1
					StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

			}*/

	virtual void SetupInitial() 
			{
				ComputeStiffnessMatrix();

			}

	virtual void GetStrain()
			{
						// Set up vector of nodal displacements
				ChMatrixDynamic<> displ(GetNcoords(),1);
				for(int i=0; i<GetNnodes(); i++)
					displ.PasteVector(this->nodes[i]->GetPos()-nodes[i]->GetX0(),i*3,0);
					
				for(int i=0; i<GpVector.size(); i++)
				{
					GpVector[i]->Strain.MatrMultiply((*GpVector[i]->MatrB), displ);
					delete GpVector[i]->MatrB;
				}
			}

	virtual void GetStress()
			{
				for(int i=0; i<GpVector.size(); i++)
				{
					GpVector[i]->Stress.MatrMultiply(Material->Get_StressStrainMatrix(), GpVector[i]->Strain);
				}
			}
	
				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
				/// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 60) && (H.GetColumns()==60));

					// compute stiffness matrix (this is already the explicit
					// formulation of the corotational stiffness matrix in 3D)
					
						// calculate global stiffness matrix
					//SetupInitial(); // NO, we assume it has been computed at the beginning of the simulation
					ChMatrixDynamic<> tempMatr = StiffnessMatrix;
					tempMatr.MatrScale( Kfactor );

					H.PasteMatrix(&tempMatr,0,0);

					// That's all, there is no damping and diffuse mass anyway.
				}

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor, and local mass matrix M multiplied by Mfactor.
				/// This is usually called only once in the simulation. 
	virtual void ComputeKRMmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0)
				{
					assert((Hl.GetRows() == 60) && (Hl.GetColumns() == 60));

					// to keep things short, here local K is as global K (anyway, only global K is used in simulations)
					ComputeKRMmatricesLocal (Hl, Kfactor, Rfactor);
				}

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 60) && (Fi.GetColumns()==1));

						// set up vector of nodal displacements
					ChMatrixDynamic<> displ(60,1);
					displ.PasteVector(nodes[0]->pos - nodes[0]->GetX0(), 0, 0);
					displ.PasteVector(nodes[1]->pos - nodes[1]->GetX0(), 3, 0);
					displ.PasteVector(nodes[2]->pos - nodes[2]->GetX0(), 6, 0);
					displ.PasteVector(nodes[3]->pos - nodes[3]->GetX0(), 9, 0);
					displ.PasteVector(nodes[4]->pos - nodes[4]->GetX0(), 12, 0);
					displ.PasteVector(nodes[5]->pos - nodes[5]->GetX0(), 15, 0);
					displ.PasteVector(nodes[6]->pos - nodes[6]->GetX0(), 18, 0);
					displ.PasteVector(nodes[7]->pos - nodes[7]->GetX0(), 21, 0);
					displ.PasteVector(nodes[8]->pos - nodes[8]->GetX0(), 24, 0);
					displ.PasteVector(nodes[9]->pos - nodes[9]->GetX0(), 27, 0);
					displ.PasteVector(nodes[10]->pos - nodes[10]->GetX0(), 30, 0);
					displ.PasteVector(nodes[11]->pos - nodes[11]->GetX0(), 33, 0);
					displ.PasteVector(nodes[12]->pos - nodes[12]->GetX0(), 36, 0);
					displ.PasteVector(nodes[13]->pos - nodes[13]->GetX0(), 39, 0);
					displ.PasteVector(nodes[14]->pos - nodes[14]->GetX0(), 42, 0);
					displ.PasteVector(nodes[15]->pos - nodes[15]->GetX0(), 45, 0);
					displ.PasteVector(nodes[16]->pos - nodes[16]->GetX0(), 48, 0);
					displ.PasteVector(nodes[17]->pos - nodes[17]->GetX0(), 51, 0);
					displ.PasteVector(nodes[18]->pos - nodes[18]->GetX0(), 54, 0);
					displ.PasteVector(nodes[19]->pos - nodes[19]->GetX0(), 57, 0);

						// [Internal Forces] = [K] * [displ]
					Fi.MatrMultiply(StiffnessMatrix,displ);

				}

			//
			// Custom properties functions
			//

				/// Set the material of the element
	void   SetMaterial( ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
	ChSharedPtr<ChContinuumElastic> GetMaterial() {return Material;}

				/// Get the StiffnessMatrix
	ChMatrixDynamic<> GetStiffnessMatrix() {return StiffnessMatrix;}
				/// Get the Nth gauss point
	ChGaussPoint* GetGaussPoint(int N) {return GpVector[N];}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)




};

}//___end of namespace fem___
}//___end of namespace chrono___

#endif
