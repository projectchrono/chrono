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

#ifndef CHELEMENTTETRA4_H
#define CHELEMENTTETRA4_H


#include "ChElement3D.h"
#include "ChNodeFEMxyz.h"


namespace chrono
{
namespace fem
{

	/// Tetahedron FEM element with 4 nodes.
	/// This is a classical element with linear displacement, hence
	/// with constant stress, constant strain.
	/// It can be easily used for 3D FEM problems.

class ChApiFem ChElementTetra_4 : public ChTetrahedron
{
protected:
		std::vector<ChNodeFEMxyz*> nodes;
		ChSharedPtr<ChContinuumElastic> Material;
		ChMatrixDynamic<> MatrB;		// matrix of shape function's partial derivatives
		ChMatrixDynamic<> StiffnessMatrix; // undeformed local stiffness matrix
		
		ChMatrixNM<double,4,4> mM; // for speeding up corotational approach
	
public:

	ChElementTetra_4();
	virtual ~ChElementTetra_4();

	virtual int GetNcoords() {return 12;}
	virtual int GetNnodes()  {return 4;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB, ChNodeFEMxyz* nodeC, ChNodeFEMxyz* nodeD) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					nodes[2]=nodeC;
					nodes[3]=nodeD;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					mvars.push_back(&nodes[2]->Variables());
					mvars.push_back(&nodes[3]->Variables());
					Kmatr.SetVariables(mvars);
				}
			



			//
			// FEM functions
			//

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

	
				/// Computes the local STIFFNESS MATRIX of the element:    
				/// K = Volume * [B]' * [D] * [B]
	virtual void ComputeStiffnessMatrix()
				{
				/* OLD ALGO
					ChMatrixDynamic<> MatCoeff(3,3);
					MatCoeff.PasteVector(nodes[1]->pos,0,0);
					MatCoeff.PasteVector(nodes[2]->pos,0,1);
					MatCoeff.PasteVector(nodes[3]->pos,0,2); 
					MatCoeff.MatrTranspose();
					ChMatrixDynamic<> MatCoeffB = MatCoeff;
				MatCoeffB.PasteVector(ChVector<>(1,1,1),0,0);
					double b1=-MatCoeffB.Det();
					MatCoeffB.SetElement(0,1,nodes[0]->pos.y);
					MatCoeffB.SetElement(0,2,nodes[0]->pos.z);
					double b2=MatCoeffB.Det();
					MatCoeffB.SetElement(1,1,nodes[1]->pos.y);
					MatCoeffB.SetElement(1,2,nodes[1]->pos.z);
					double b3=-MatCoeffB.Det();
					MatCoeffB.SetElement(2,1,nodes[2]->pos.y);
					MatCoeffB.SetElement(2,2,nodes[2]->pos.z);
					double b4=MatCoeffB.Det();
					ChMatrixDynamic<> MatCoeffC = MatCoeff;
				MatCoeffC.PasteVector(ChVector<>(1,1,1),0,1);
					double c1=-MatCoeffC.Det();
					MatCoeffC.SetElement(0,0,nodes[0]->pos.x);
					MatCoeffC.SetElement(0,2,nodes[0]->pos.z);
					double c2=MatCoeffC.Det();
					MatCoeffC.SetElement(1,0,nodes[1]->pos.x);
					MatCoeffC.SetElement(1,2,nodes[1]->pos.z);
					double c3=-MatCoeffC.Det();
					MatCoeffC.SetElement(2,0,nodes[2]->pos.x);
					MatCoeffC.SetElement(2,2,nodes[2]->pos.z);
					double c4=MatCoeffC.Det();
					ChMatrixDynamic<> MatCoeffD = MatCoeff;
				MatCoeffD.PasteVector(ChVector<>(1,1,1),0,2);
					double d1=-MatCoeffD.Det();
					MatCoeffD.SetElement(0,0,nodes[0]->pos.x);
					MatCoeffD.SetElement(0,1,nodes[0]->pos.y);
					double d2=MatCoeffD.Det();
					MatCoeffD.SetElement(1,0,nodes[1]->pos.x);
					MatCoeffD.SetElement(1,1,nodes[1]->pos.y);
					double d3=-MatCoeffD.Det();
					MatCoeffD.SetElement(2,0,nodes[2]->pos.x);
					MatCoeffD.SetElement(2,1,nodes[2]->pos.y);
					double d4=MatCoeffD.Det();
					//setting matrix B
					MatrB.Resize(6,12);
					MatrB.SetElement(0,0,b1);	MatrB.SetElement(0,3,b2);	MatrB.SetElement(0,6,b3);	MatrB.SetElement(0,9,b4);	
					MatrB.SetElement(1,1,c1);	MatrB.SetElement(1,4,c2);	MatrB.SetElement(1,7,c3);	MatrB.SetElement(1,10,c4);
					MatrB.SetElement(2,2,d1);	MatrB.SetElement(2,5,d2);	MatrB.SetElement(2,8,d3);	MatrB.SetElement(2,11,d4);	
					MatrB.SetElement(3,0,c1);	MatrB.SetElement(3,1,b1);	MatrB.SetElement(3,3,c2);	MatrB.SetElement(3,4,b2);	MatrB.SetElement(3,6,c3);	MatrB.SetElement(3,7,b3);	MatrB.SetElement(3,9,c4);	MatrB.SetElement(3,10,b4);
					MatrB.SetElement(4,1,d1);	MatrB.SetElement(4,2,c1);	MatrB.SetElement(4,4,d2);	MatrB.SetElement(4,5,c2);	MatrB.SetElement(4,7,d3);	MatrB.SetElement(4,8,c3);	MatrB.SetElement(4,10,d4);	MatrB.SetElement(4,11,c4);
					MatrB.SetElement(5,0,d1);	MatrB.SetElement(5,2,b1);	MatrB.SetElement(5,3,d2);	MatrB.SetElement(5,5,b2);	MatrB.SetElement(5,6,d3);	MatrB.SetElement(5,8,b3);	MatrB.SetElement(5,9,d4);	MatrB.SetElement(5,11,b4);
					MatrB.MatrDivScale(6*Volume);
					ChMatrixDynamic<> temp(12,6);
					temp.MatrTMultiply(MatrB, Material->Get_StressStrainMatrix());
					StiffnessMatrix.Resize(12,12);
					StiffnessMatrix.MatrMultiply(temp,MatrB);
					StiffnessMatrix.MatrScale(Volume);
				*/
					// new algo for stiffness matrix computation:
					
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

					MatrB.Reset(6,12);
					MatrB(0)  = mM(0); MatrB(3)  = mM(4); MatrB(6)  = mM(8);  MatrB(9)  = mM(12);
					MatrB(13) = mM(1); MatrB(16) = mM(5); MatrB(19) = mM(9);  MatrB(22) = mM(13);
					MatrB(26) = mM(2); MatrB(29) = mM(6); MatrB(32) = mM(10); MatrB(35) = mM(14);
					MatrB(36) = mM(1); MatrB(37) = mM(0); MatrB(39) = mM(5);  MatrB(40) = mM(4); MatrB(42) = mM(9);  MatrB(43) = mM(8); MatrB(45) = mM(13); MatrB(46) = mM(12);
					MatrB(49) = mM(2); MatrB(50) = mM(1); MatrB(52) = mM(6);  MatrB(53) = mM(5); MatrB(55) = mM(10); MatrB(56) = mM(9); MatrB(58) = mM(14); MatrB(59) = mM(13);
					MatrB(60) = mM(2); MatrB(62) = mM(0); MatrB(63) = mM(6);  MatrB(65) = mM(4); MatrB(66) = mM(10); MatrB(68) = mM(8); MatrB(69) = mM(14); MatrB(71) = mM(12);

					ChMatrixNM<double,6,12> EB;
					EB.MatrMultiply(this->Material->Get_StressStrainMatrix(), MatrB);

					StiffnessMatrix.MatrTMultiply(MatrB,EB);

					StiffnessMatrix.MatrScale(Volume);
				}

				/// set up the element's parameters and matrices 
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
					double det = ChPolarDecomposition::Compute(F, this->A, S, 1E-6);
					if (det <0)
						this->A.MatrScale(-1.0);

					GetLog() << "FEM rotation: \n" << A << "\n" ;
				}


				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 12) && (H.GetColumns()==12));

					// warp the local stiffness matrix K in order to obtain global 
					// tangent stiffness CKCt:
					ChMatrixDynamic<> CK(12,12);
					ChMatrixDynamic<> CKCt(12,12); // the global, corotated, K matrix
					ChMatrixCorotation<>::ComputeCK(StiffnessMatrix, this->A, 4, CK);
					ChMatrixCorotation<>::ComputeKCt(CK, this->A, 4, CKCt);

					// DEBUG
					/*
					ChMatrixDynamic<> Ctest(12,12);
					Ctest.PasteMatrix(&A,0,0);
					Ctest.PasteMatrix(&A,3,3);
					Ctest.PasteMatrix(&A,6,6);
					Ctest.PasteMatrix(&A,9,9);
					ChMatrixDynamic<> CKtest(12,12);
					CKtest.MatrMultiply(Ctest,StiffnessMatrix);
					ChMatrixDynamic<> CKCttest(12,12);
					CKCttest.MatrMultiplyT(CKtest,Ctest);
					GetLog() << "CKCt difference \n" << CKCt-CKCttest << "\n";
					*/

					// For K stiffness matrix and R damping matrix:

					double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

					CKCt.MatrScale( mkfactor );

					H.PasteMatrix(&CKCt,0,0);


					// For M mass matrix:
					if (Mfactor)
					{
						double lumped_node_mass = (this->GetVolume() * this->Material->Get_density() ) / 4.0;
						for (int id = 0; id < 12; id++)
						{
							double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
							H(id,id)+= amfactor * lumped_node_mass;
						}
					}
					//***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
				}


				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 12) && (Fi.GetColumns()==1));

						// set up vector of nodal displacements (in local element system) u_l = R*p - p0
					ChMatrixDynamic<> displ(12,1);
					displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos) - nodes[0]->GetX0(), 0, 0); // nodal displacements, local
					displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos) - nodes[1]->GetX0(), 3, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos) - nodes[2]->GetX0(), 6, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos) - nodes[3]->GetX0(), 9, 0);

						// [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
					ChMatrixDynamic<> FiK_local(12,1);
					FiK_local.MatrMultiply(StiffnessMatrix, displ);

					displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos_dt), 0, 0); // nodal speeds, local
					displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos_dt), 3, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos_dt), 6, 0);
					displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos_dt), 9, 0);
					ChMatrixDynamic<> FiR_local(12,1);
					FiR_local.MatrMultiply(StiffnessMatrix, displ);
					FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

					double lumped_node_mass = (this->GetVolume() * this->Material->Get_density() ) / 4.0;
					displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM() ); // reuse 'displ' for performance 
					FiR_local.MatrInc(displ); 
					//***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

					FiK_local.MatrInc(FiR_local);

					FiK_local.MatrScale(-1.0);

						// Fi = C * Fi_local  with C block-diagonal rotations A
					ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 4, Fi);

				}

			//
			// Custom properties functions
			//

				/// Set the material of the element
	void   SetMaterial( ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
	ChSharedPtr<ChContinuumElastic> GetMaterial() {return Material;}

				/// Get the partial derivatives matrix MatrB and the StiffnessMatrix
	ChMatrixDynamic<>   GetMatrB() { return MatrB;}
	ChMatrixDynamic<> GetStiffnessMatrix() {return StiffnessMatrix;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)




};

}//___end of namespace fem___
}//___end of namespace chrono___

#endif
