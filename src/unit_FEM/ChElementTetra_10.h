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
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHELEMENTTETRA10_H
#define CHELEMENTTETRA10_H


#include "ChElement3D.h"
#include "ChNodeFEMxyz.h"

namespace chrono
{
namespace fem
{

	/// Tetahedron FEM element with 10 nodes.
	/// This is a quadratic element for displacementes; stress and strain 
	/// are interpolated depending on Gauss points.

class ChApiFem ChElementTetra_10 : public ChTetrahedron
{
protected:
		std::vector<ChNodeFEMxyz*> nodes;
		ChSharedPtr<ChContinuumElastic> Material;
		std::vector<ChMatrixDynamic<> > MatrB;		// matrices of shape function's partial derivatives (one for each integration point)
													// we use a vector to keep in memory all the four matrices (-> 4 integr. point)
		ChMatrixDynamic<> StiffnessMatrix;

		ChMatrixNM<double,4,4> mM; // for speeding up corotational approach

public:

	ChElementTetra_10();
	virtual ~ChElementTetra_10();

	virtual int GetNcoords() {return 30;}
	virtual int GetNnodes()  {return 10;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB, ChNodeFEMxyz* nodeC, ChNodeFEMxyz* nodeD,
						ChNodeFEMxyz* nodeE, ChNodeFEMxyz* nodeF, ChNodeFEMxyz* nodeG, ChNodeFEMxyz* nodeH, ChNodeFEMxyz* nodeI, ChNodeFEMxyz* nodeJ) 
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
					Kmatr.SetVariables(mvars);
				}
			
			//
			// FEM functions
			//

				/// Approximation!! not the exact volume
				/// This returns an exact value only in case of Constant Metric Tetrahedron
	double ComputeVolume()
				{
					ChVector<> B1,C1,D1;
					B1.Sub(nodes[1]->pos,nodes[0]->pos);
					C1.Sub(nodes[2]->pos,nodes[0]->pos);
					D1.Sub(nodes[3]->pos,nodes[0]->pos);
					ChMatrixDynamic<> M(3,3);
					M.PasteVector(B1,0,0);
					M.PasteVector(C1,0,1);
					M.PasteVector(D1,0,2);
					M.MatrTranspose();
					Volume = abs(M.Det()/6);
					return Volume;
				}
			
				/// Puts inside 'Jacobian' the Jacobian matrix of the element
				/// zeta1,...,zeta4 are the four natural coordinates of the integration point
				/// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
	virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, double zeta1, double zeta2, double zeta3, double zeta4) 
				{
					Jacobian.FillElem(1);

					Jacobian.SetElement(1,0, 4*(nodes[0]->pos.x*(zeta1-1/4)+nodes[4]->pos.x*zeta2+nodes[6]->pos.x*zeta3+nodes[7]->pos.x*zeta4));
					Jacobian.SetElement(2,0, 4*(nodes[0]->pos.y*(zeta1-1/4)+nodes[4]->pos.y*zeta2+nodes[6]->pos.y*zeta3+nodes[7]->pos.y*zeta4));
					Jacobian.SetElement(3,0, 4*(nodes[0]->pos.z*(zeta1-1/4)+nodes[4]->pos.z*zeta2+nodes[6]->pos.z*zeta3+nodes[7]->pos.z*zeta4));
					Jacobian.SetElement(1,1, 4*(nodes[4]->pos.x*zeta1+nodes[1]->pos.x*(zeta2-1/4)+nodes[5]->pos.x*zeta3+nodes[8]->pos.x*zeta4));
					Jacobian.SetElement(2,1, 4*(nodes[4]->pos.y*zeta1+nodes[1]->pos.y*(zeta2-1/4)+nodes[5]->pos.y*zeta3+nodes[8]->pos.y*zeta4));
					Jacobian.SetElement(3,1, 4*(nodes[4]->pos.z*zeta1+nodes[1]->pos.z*(zeta2-1/4)+nodes[5]->pos.z*zeta3+nodes[8]->pos.z*zeta4));
					Jacobian.SetElement(1,2, 4*(nodes[6]->pos.x*zeta1+nodes[5]->pos.x*zeta2+nodes[2]->pos.x*(zeta3-1/4)+nodes[9]->pos.x*zeta4));
					Jacobian.SetElement(2,2, 4*(nodes[6]->pos.y*zeta1+nodes[5]->pos.y*zeta2+nodes[2]->pos.y*(zeta3-1/4)+nodes[9]->pos.y*zeta4));
					Jacobian.SetElement(3,2, 4*(nodes[6]->pos.z*zeta1+nodes[5]->pos.z*zeta2+nodes[2]->pos.z*(zeta3-1/4)+nodes[9]->pos.z*zeta4));
					Jacobian.SetElement(1,3, 4*(nodes[7]->pos.x*zeta1+nodes[8]->pos.x*zeta2+nodes[9]->pos.x*zeta3+nodes[3]->pos.x*(zeta4-1/4)));
					Jacobian.SetElement(2,3, 4*(nodes[7]->pos.y*zeta1+nodes[8]->pos.y*zeta2+nodes[9]->pos.y*zeta3+nodes[3]->pos.y*(zeta4-1/4)));
					Jacobian.SetElement(3,3, 4*(nodes[7]->pos.z*zeta1+nodes[8]->pos.z*zeta2+nodes[9]->pos.z*zeta3+nodes[3]->pos.z*(zeta4-1/4)));
				}

				/// Computes the matrix of partial derivatives and puts data in "A"
				///	ID (0,...,3) identifies the integration point; zeta1,...,zeta4 are its four natural coordinates
				/// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
	virtual void ComputeMatrB(int ID, double zeta1, double zeta2, double zeta3, double zeta4, double& JacobianDet) 
			{
				ChMatrixDynamic<> Jacobian(4,4);
				ComputeJacobian(Jacobian, zeta1, zeta2, zeta3, zeta4);
				
				double Jdet=Jacobian.Det();
				JacobianDet = Jdet;		// !!! store the Jacobian Determinant: needed for the integration
		
				MatrB[ID].SetElement(0,0,(4*zeta1-1)*((Jacobian(2,3)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,1))-(Jacobian(2,2)-Jacobian(2,1))*(Jacobian(3,3)-Jacobian(3,1)))/Jdet);
				MatrB[ID].SetElement(0,3,(4*zeta2-1)*((Jacobian(2,2)-Jacobian(2,0))*(Jacobian(3,3)-Jacobian(3,2))-(Jacobian(2,2)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,2)))/Jdet);
				MatrB[ID].SetElement(0,6,(4*zeta3-1)*((Jacobian(2,1)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,3))-(Jacobian(2,0)-Jacobian(2,3))*(Jacobian(3,1)-Jacobian(3,3)))/Jdet);
				MatrB[ID].SetElement(0,9,(4*zeta4-1)*((Jacobian(2,0)-Jacobian(2,2))*(Jacobian(3,1)-Jacobian(3,0))-(Jacobian(2,0)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,0)))/Jdet);
				MatrB[ID].SetElement(0,12,4*(zeta1*((Jacobian(2,2)-Jacobian(2,0))*(Jacobian(3,3)-Jacobian(3,2))-(Jacobian(2,2)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,2)))+zeta2*((Jacobian(2,3)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,1))-(Jacobian(2,2)-Jacobian(2,1))*(Jacobian(3,3)-Jacobian(3,1))))/Jdet);
				MatrB[ID].SetElement(0,15,4*(zeta2*((Jacobian(2,1)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,3))-(Jacobian(2,0)-Jacobian(2,3))*(Jacobian(3,1)-Jacobian(3,3)))+zeta3*((Jacobian(2,2)-Jacobian(2,0))*(Jacobian(3,3)-Jacobian(3,2))-(Jacobian(2,2)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,2))))/Jdet);
				MatrB[ID].SetElement(0,18,4*(zeta3*((Jacobian(2,3)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,1))-(Jacobian(2,2)-Jacobian(2,1))*(Jacobian(3,3)-Jacobian(3,1)))+zeta1*((Jacobian(2,1)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,3))-(Jacobian(2,0)-Jacobian(2,3))*(Jacobian(3,1)-Jacobian(3,3))))/Jdet);
				MatrB[ID].SetElement(0,21,4*(zeta1*((Jacobian(2,0)-Jacobian(2,2))*(Jacobian(3,1)-Jacobian(3,0))-(Jacobian(2,0)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,0)))+zeta4*((Jacobian(2,3)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,1))-(Jacobian(2,2)-Jacobian(2,1))*(Jacobian(3,3)-Jacobian(3,1))))/Jdet);
				MatrB[ID].SetElement(0,24,4*(zeta2*((Jacobian(2,0)-Jacobian(2,2))*(Jacobian(3,1)-Jacobian(3,0))-(Jacobian(2,0)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,0)))+zeta4*((Jacobian(2,2)-Jacobian(2,0))*(Jacobian(3,3)-Jacobian(3,2))-(Jacobian(2,2)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,2))))/Jdet);
				MatrB[ID].SetElement(0,27,4*(zeta3*((Jacobian(2,0)-Jacobian(2,2))*(Jacobian(3,1)-Jacobian(3,0))-(Jacobian(2,0)-Jacobian(2,1))*(Jacobian(3,2)-Jacobian(3,0)))+zeta4*((Jacobian(2,1)-Jacobian(2,3))*(Jacobian(3,0)-Jacobian(3,3))-(Jacobian(2,0)-Jacobian(2,3))*(Jacobian(3,1)-Jacobian(3,3))))/Jdet);

				MatrB[ID].SetElement(1,1,(4*zeta1-1)*((Jacobian(1,2)-Jacobian(1,1))*(Jacobian(3,3)-Jacobian(3,1))-(Jacobian(1,3)-Jacobian(1,1))*(Jacobian(3,2)-Jacobian(3,1)))/Jdet);
				MatrB[ID].SetElement(1,4,(4*zeta2-1)*((Jacobian(1,3)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,0))-(Jacobian(1,0)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,3)))/Jdet);
				MatrB[ID].SetElement(1,7,(4*zeta3-1)*((Jacobian(1,0)-Jacobian(1,3))*(Jacobian(3,1)-Jacobian(3,3))-(Jacobian(1,1)-Jacobian(1,3))*(Jacobian(3,0)-Jacobian(3,3)))/Jdet);
				MatrB[ID].SetElement(1,10,(4*zeta4-1)*((Jacobian(1,1)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,2))-(Jacobian(1,2)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,1)))/Jdet);
				MatrB[ID].SetElement(1,13,4*(zeta1*((Jacobian(1,3)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,0))-(Jacobian(1,0)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,3)))+zeta2*((Jacobian(1,2)-Jacobian(1,1))*(Jacobian(3,3)-Jacobian(3,1))-(Jacobian(1,3)-Jacobian(1,1))*(Jacobian(3,2)-Jacobian(3,1))))/Jdet);
				MatrB[ID].SetElement(1,16,4*(zeta2*((Jacobian(1,0)-Jacobian(1,3))*(Jacobian(3,1)-Jacobian(3,3))-(Jacobian(1,1)-Jacobian(1,3))*(Jacobian(3,0)-Jacobian(3,3)))+zeta3*((Jacobian(1,3)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,0))-(Jacobian(1,0)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,3))))/Jdet);
				MatrB[ID].SetElement(1,19,4*(zeta3*((Jacobian(1,2)-Jacobian(1,1))*(Jacobian(3,3)-Jacobian(3,1))-(Jacobian(1,3)-Jacobian(1,1))*(Jacobian(3,2)-Jacobian(3,1)))+zeta1*((Jacobian(1,0)-Jacobian(1,3))*(Jacobian(3,1)-Jacobian(3,3))-(Jacobian(1,1)-Jacobian(1,3))*(Jacobian(3,0)-Jacobian(3,3))))/Jdet);
				MatrB[ID].SetElement(1,22,4*(zeta1*((Jacobian(1,1)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,2))-(Jacobian(1,2)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,1)))+zeta4*((Jacobian(1,2)-Jacobian(1,1))*(Jacobian(3,3)-Jacobian(3,1))-(Jacobian(1,3)-Jacobian(1,1))*(Jacobian(3,2)-Jacobian(3,1))))/Jdet);
				MatrB[ID].SetElement(1,25,4*(zeta2*((Jacobian(1,1)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,2))-(Jacobian(1,2)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,1)))+zeta4*((Jacobian(1,3)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,0))-(Jacobian(1,0)-Jacobian(1,2))*(Jacobian(3,2)-Jacobian(3,3))))/Jdet);
				MatrB[ID].SetElement(1,28,4*(zeta3*((Jacobian(1,1)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,2))-(Jacobian(1,2)-Jacobian(1,0))*(Jacobian(3,0)-Jacobian(3,1)))+zeta4*((Jacobian(1,0)-Jacobian(1,3))*(Jacobian(3,1)-Jacobian(3,3))-(Jacobian(1,1)-Jacobian(1,3))*(Jacobian(3,0)-Jacobian(3,3))))/Jdet);

				MatrB[ID].SetElement(2,2,(4*zeta1-1)*((Jacobian(1,3)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,1))-(Jacobian(1,2)-Jacobian(1,1))*(Jacobian(2,3)-Jacobian(2,1)))/Jdet);
				MatrB[ID].SetElement(2,5,(4*zeta2-1)*((Jacobian(1,2)-Jacobian(1,0))*(Jacobian(2,3)-Jacobian(2,2))-(Jacobian(1,2)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,2)))/Jdet);
				MatrB[ID].SetElement(2,8,(4*zeta3-1)*((Jacobian(1,1)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,3))-(Jacobian(1,0)-Jacobian(1,3))*(Jacobian(2,1)-Jacobian(2,3)))/Jdet);
				MatrB[ID].SetElement(2,11,(4*zeta4-1)*((Jacobian(1,0)-Jacobian(1,2))*(Jacobian(2,1)-Jacobian(2,0))-(Jacobian(1,0)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,0)))/Jdet);
				MatrB[ID].SetElement(2,14,4*(zeta1*((Jacobian(1,2)-Jacobian(1,0))*(Jacobian(2,3)-Jacobian(2,2))-(Jacobian(1,2)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,2)))+zeta2*((Jacobian(1,3)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,1))-(Jacobian(1,2)-Jacobian(1,1))*(Jacobian(2,3)-Jacobian(2,1))))/Jdet);
				MatrB[ID].SetElement(2,17,4*(zeta2*((Jacobian(1,1)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,3))-(Jacobian(1,0)-Jacobian(1,3))*(Jacobian(2,1)-Jacobian(2,3)))+zeta3*((Jacobian(1,2)-Jacobian(1,0))*(Jacobian(2,3)-Jacobian(2,2))-(Jacobian(1,2)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,2))))/Jdet);
				MatrB[ID].SetElement(2,20,4*(zeta3*((Jacobian(1,3)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,1))-(Jacobian(1,2)-Jacobian(1,1))*(Jacobian(2,3)-Jacobian(2,1)))+zeta1*((Jacobian(1,1)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,3))-(Jacobian(1,0)-Jacobian(1,3))*(Jacobian(2,1)-Jacobian(2,3))))/Jdet);
				MatrB[ID].SetElement(2,23,4*(zeta1*((Jacobian(1,0)-Jacobian(1,2))*(Jacobian(2,1)-Jacobian(2,0))-(Jacobian(1,0)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,0)))+zeta4*((Jacobian(1,3)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,1))-(Jacobian(1,2)-Jacobian(1,1))*(Jacobian(2,3)-Jacobian(2,1))))/Jdet);
				MatrB[ID].SetElement(2,26,4*(zeta2*((Jacobian(1,0)-Jacobian(1,2))*(Jacobian(2,1)-Jacobian(2,0))-(Jacobian(1,0)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,0)))+zeta4*((Jacobian(1,2)-Jacobian(1,0))*(Jacobian(2,3)-Jacobian(2,2))-(Jacobian(1,2)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,2))))/Jdet);
				MatrB[ID].SetElement(2,29,4*(zeta3*((Jacobian(1,0)-Jacobian(1,2))*(Jacobian(2,1)-Jacobian(2,0))-(Jacobian(1,0)-Jacobian(1,1))*(Jacobian(2,2)-Jacobian(2,0)))+zeta4*((Jacobian(1,1)-Jacobian(1,3))*(Jacobian(2,0)-Jacobian(2,3))-(Jacobian(1,0)-Jacobian(1,3))*(Jacobian(2,1)-Jacobian(2,3))))/Jdet);
	
				MatrB[ID].SetElement(3,0,MatrB[ID](1,1));
				MatrB[ID].SetElement(3,1,MatrB[ID](0,0));
				MatrB[ID].SetElement(3,3,MatrB[ID](1,4));
				MatrB[ID].SetElement(3,4,MatrB[ID](0,3));
				MatrB[ID].SetElement(3,6,MatrB[ID](1,7));
				MatrB[ID].SetElement(3,7,MatrB[ID](0,6));
				MatrB[ID].SetElement(3,9,MatrB[ID](1,10));
				MatrB[ID].SetElement(3,10,MatrB[ID](0,9));
				MatrB[ID].SetElement(3,12,MatrB[ID](1,13));
				MatrB[ID].SetElement(3,13,MatrB[ID](0,12));
				MatrB[ID].SetElement(3,15,MatrB[ID](1,16));
				MatrB[ID].SetElement(3,16,MatrB[ID](0,15));
				MatrB[ID].SetElement(3,18,MatrB[ID](1,19));
				MatrB[ID].SetElement(3,19,MatrB[ID](0,18));
				MatrB[ID].SetElement(3,21,MatrB[ID](1,22));
				MatrB[ID].SetElement(3,22,MatrB[ID](0,21));
				MatrB[ID].SetElement(3,24,MatrB[ID](1,25));
				MatrB[ID].SetElement(3,25,MatrB[ID](0,24));
				MatrB[ID].SetElement(3,27,MatrB[ID](1,28));
				MatrB[ID].SetElement(3,28,MatrB[ID](0,27));

				MatrB[ID].SetElement(4,1,MatrB[ID](2,2));
				MatrB[ID].SetElement(4,2,MatrB[ID](1,1));
				MatrB[ID].SetElement(4,4,MatrB[ID](2,5));
				MatrB[ID].SetElement(4,5,MatrB[ID](1,4));
				MatrB[ID].SetElement(4,7,MatrB[ID](2,8));
				MatrB[ID].SetElement(4,8,MatrB[ID](1,7));
				MatrB[ID].SetElement(4,10,MatrB[ID](2,11));
				MatrB[ID].SetElement(4,11,MatrB[ID](1,10));
				MatrB[ID].SetElement(4,13,MatrB[ID](2,14));
				MatrB[ID].SetElement(4,14,MatrB[ID](1,13));
				MatrB[ID].SetElement(4,16,MatrB[ID](2,17));
				MatrB[ID].SetElement(4,17,MatrB[ID](1,16));
				MatrB[ID].SetElement(4,19,MatrB[ID](2,20));
				MatrB[ID].SetElement(4,20,MatrB[ID](1,19));
				MatrB[ID].SetElement(4,22,MatrB[ID](2,23));
				MatrB[ID].SetElement(4,23,MatrB[ID](1,22));
				MatrB[ID].SetElement(4,25,MatrB[ID](2,26));
				MatrB[ID].SetElement(4,26,MatrB[ID](1,25));
				MatrB[ID].SetElement(4,28,MatrB[ID](2,29));
				MatrB[ID].SetElement(4,29,MatrB[ID](1,28));

				MatrB[ID].SetElement(5,0,MatrB[ID](2,2));
				MatrB[ID].SetElement(5,2,MatrB[ID](0,0));
				MatrB[ID].SetElement(5,3,MatrB[ID](2,5));
				MatrB[ID].SetElement(5,5,MatrB[ID](0,3));
				MatrB[ID].SetElement(5,6,MatrB[ID](2,8));
				MatrB[ID].SetElement(5,8,MatrB[ID](0,6));
				MatrB[ID].SetElement(5,9,MatrB[ID](2,11));
				MatrB[ID].SetElement(5,11,MatrB[ID](0,9));
				MatrB[ID].SetElement(5,12,MatrB[ID](2,14));
				MatrB[ID].SetElement(5,14,MatrB[ID](0,12));
				MatrB[ID].SetElement(5,15,MatrB[ID](2,17));
				MatrB[ID].SetElement(5,17,MatrB[ID](0,15));
				MatrB[ID].SetElement(5,18,MatrB[ID](2,20));
				MatrB[ID].SetElement(5,20,MatrB[ID](0,18));
				MatrB[ID].SetElement(5,21,MatrB[ID](2,23));
				MatrB[ID].SetElement(5,23,MatrB[ID](0,21));
				MatrB[ID].SetElement(5,24,MatrB[ID](2,26));
				MatrB[ID].SetElement(5,26,MatrB[ID](0,24));
				MatrB[ID].SetElement(5,27,MatrB[ID](2,29));
				MatrB[ID].SetElement(5,29,MatrB[ID](0,27));
				MatrB[ID].MatrScale(2);
			}

				/// Computes the local STIFFNESS MATRIX of the element:    
				/// K = sum (w_i * [B]' * [D] * [B])
				/// 
	virtual void ComputeStiffnessMatrix() 
			{
				// for speeding up corotational, used later:
				// M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
				//     [ 1    1    1    1    ]
				mM.PasteVector(nodes[0]->GetX0(), 0,0);
				mM.PasteVector(nodes[1]->GetX0(), 0,1);
				mM.PasteVector(nodes[2]->GetX0(), 0,2);
				mM.PasteVector(nodes[3]->GetX0(), 0,3);
				mM(3,0)=1.0;
				mM(3,1)=1.0;
				mM(3,2)=1.0;
				mM(3,3)=1.0;
				mM.MatrInverse();

			//========================
			//Exact Integration (4 Gp)
			//========================
				double zeta1, zeta2, zeta3, zeta4;
				double JacobianDet;
				ChMatrixDynamic<> temp;
				ChMatrixDynamic<> BT;


				zeta1=0.58541020;
				zeta2=0.1381966;
				zeta3=0.1381966;
				zeta4=0.1381966;

				ComputeMatrB(0, zeta1, zeta2, zeta3, zeta4, JacobianDet);
				BT=MatrB[0];
				BT.MatrTranspose();
				temp = (BT * Material->Get_StressStrainMatrix() * MatrB[0]);
				temp.MatrScale(JacobianDet/6);
					//Gauss integration weight = 1*1/4*1/4*1/4
				temp.MatrDivScale(16);
				StiffnessMatrix = temp;

				zeta1=0.1381966;
				zeta2=0.58541020;
				zeta3=0.1381966;
				zeta4=0.1381966;

				ComputeMatrB(1, zeta1, zeta2, zeta3, zeta4, JacobianDet);
				BT=MatrB[1];
				BT.MatrTranspose();
				temp = (BT * Material->Get_StressStrainMatrix() * MatrB[1]);
				temp.MatrScale(JacobianDet/6);
					//Gauss integration weight = 1*1/4*1/4*1/4
				temp.MatrDivScale(16);
				StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);
	
				zeta1=0.1381966;
				zeta2=0.1381966;
				zeta3=0.58541020;
				zeta4=0.1381966;

				ComputeMatrB(2, zeta1, zeta2, zeta3, zeta4, JacobianDet);
				BT=MatrB[2];
				BT.MatrTranspose();
				temp = (BT * Material->Get_StressStrainMatrix() * MatrB[2]);
				temp.MatrScale(JacobianDet/6);
					//Gauss integration weight = 1*1/4*1/4*1/4
				temp.MatrDivScale(16);
				StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

				zeta1=0.1381966;
				zeta2=0.1381966;
				zeta3=0.1381966;
				zeta4=0.58541020;

				ComputeMatrB(3, zeta1, zeta2, zeta3, zeta4, JacobianDet);
				BT=MatrB[3];
				BT.MatrTranspose();
				temp = (BT * Material->Get_StressStrainMatrix() * MatrB[3]);
				temp.MatrScale(JacobianDet/6);
					//Gauss integration weight = 1*1/4*1/4*1/4
				temp.MatrDivScale(16);
				StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

				StiffnessMatrix.MatrDivScale(2);		//!!! => because the canonical interval is -1 ... +1,  but we want to integrate 
														//		 in 0 ... +1 -> we have to multiply by: b-a/2 ( = (1-0)/2 = 1/2)

			}

	virtual void SetupInitial() 
			{
				ComputeVolume();
				ComputeStiffnessMatrix();
			}

			// compute large rotation of element for corotational approach
	virtual void UpdateRotation()
			{
				// P = [ p_0  p_1  p_2  p_3 ]  
				//     [ 1    1    1    1   ]
				ChMatrixNM<double,4,4> P;
				P.PasteVector(nodes[0]->pos, 0,0);
				P.PasteVector(nodes[1]->pos, 0,1);
				P.PasteVector(nodes[2]->pos, 0,2);
				P.PasteVector(nodes[3]->pos, 0,3);
				P(3,0)=1.0;
				P(3,1)=1.0;
				P(3,2)=1.0;
				P(3,3)=1.0;

				ChMatrix33<double> F; 
				// F=P*mM (only upper-left 3x3 block!)
				double sum;
				for (int colres=0; colres < 3; ++colres)
					for (int row=0; row < 3; ++row)
					{
						sum = 0;
						for (int col=0; col < 4; ++col)
								sum+= (P(row,col))*(mM(col,colres));
						F(row, colres)= sum;
					}
				ChMatrix33<> S;
				double det = ChPolarDecomposition<double>::Compute(F, this->A, S, 1E-6);
				if (det <0)
					this->A.MatrScale(-1.0);

				//GetLog() << "FEM rotation: \n" << A << "\n" 
			}

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 30) && (H.GetColumns()==30));

					// warp the local stiffness matrix K in order to obtain global 
					// tangent stiffness CKCt:
					ChMatrixDynamic<> CK(30,30);
					ChMatrixDynamic<> CKCt(30,30); // the global, corotated, K matrix
					ChMatrixCorotation::ComputeCK(StiffnessMatrix, this->A, 10, CK);
					ChMatrixCorotation::ComputeKCt(CK, this->A, 10, CKCt);
					
					// For K stiffness matrix and R damping matrix:

					double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

					CKCt.MatrScale( mkfactor );

					H.PasteMatrix(&CKCt,0,0);

					// For M mass matrix:
					if (Mfactor)
					{
						double lumped_node_mass = (this->GetVolume() * this->Material->Get_density() ) / (double)this->GetNnodes();
						for (int id = 0; id < 30; id++)
						{
							double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
							H(id,id)+= amfactor * lumped_node_mass;
						}
					}
					//***TO DO*** better per-node lumping, or 30x30 consistent mass matrix.
				}


				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 30) && (Fi.GetColumns()==1));

						// set up vector of nodal displacements (in local element system) u_l = R*p - p0
					ChMatrixDynamic<> displ(30,1);
					displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos) - nodes[0]->GetX0(), 0, 0); // nodal displacements, local
					displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos) - nodes[1]->GetX0(), 3, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos) - nodes[2]->GetX0(), 6, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos) - nodes[3]->GetX0(), 9, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[4]->pos) - nodes[4]->GetX0(),12, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[5]->pos) - nodes[5]->GetX0(),15, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[6]->pos) - nodes[6]->GetX0(),18, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[7]->pos) - nodes[7]->GetX0(),21, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[8]->pos) - nodes[8]->GetX0(),24, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[9]->pos) - nodes[9]->GetX0(),27, 0);

						// [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
					ChMatrixDynamic<> FiK_local(30,1);
					FiK_local.MatrMultiply(StiffnessMatrix, displ);

					displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos_dt), 0, 0); // nodal speeds, local
					displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos_dt), 3, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos_dt), 6, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos_dt), 9, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[4]->pos_dt),12, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[5]->pos_dt),15, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[6]->pos_dt),18, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[7]->pos_dt),21, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[8]->pos_dt),24, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[9]->pos_dt),27, 0);
					ChMatrixDynamic<> FiR_local(30,1);
					FiR_local.MatrMultiply(StiffnessMatrix, displ);
					FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

					double lumped_node_mass = (this->GetVolume() * this->Material->Get_density() ) / (double)this->GetNnodes();
					displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM() ); // reuse 'displ' for performance 
					FiR_local.MatrInc(displ); 
					//***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

					FiK_local.MatrInc(FiR_local);

					FiK_local.MatrScale(-1.0);

						// Fi = C * Fi_local  with C block-diagonal rotations A
					ChMatrixCorotation::ComputeCK(FiK_local, this->A, 10, Fi);

				}

			//
			// Custom properties functions
			//

				/// Set the material of the element
	void   SetMaterial( ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
	ChSharedPtr<ChContinuumElastic> GetMaterial() {return Material;}

				/// Get the partial derivatives matrix MatrB and the StiffnessMatrix
	ChMatrixDynamic<>   GetMatrB(int n) { return MatrB[n];}
	ChMatrixDynamic<> GetStiffnessMatrix() {return StiffnessMatrix;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)




};

}//___end of namespace fem___
}//___end of namespace chrono___

#endif
