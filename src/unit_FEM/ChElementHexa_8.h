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
// File authors: Andrea Favali

#ifndef CHELEMENTHEXA8_H
#define CHELEMENTHEXA8_H


#include "ChElementHexahedron.h"
#include "ChNodeFEMxyz.h"

namespace chrono
{
namespace fem
{

	/// Class for FEM elements of hexahedron type (isoparametric 3D bricks) 
	/// with 8 nodes. This element has a linear displacement field.

class ChApiFem ChElementHexa_8 : public ChElementHexahedron
{
protected:
		std::vector< ChSharedPtr<ChNodeFEMxyz> > nodes;
		ChSharedPtr<ChContinuumElastic> Material;
		//std::vector< ChMatrixDynamic<> > MatrB;	// matrices of shape function's partial derivatives (one for each integration point)
													// we use a vector to keep in memory all the 8 matrices (-> 8 integr. point)
													// NO! each matrix is stored in the respective gauss point

		ChMatrixDynamic<> StiffnessMatrix;

public:

	ChElementHexa_8();
	virtual ~ChElementHexa_8();

	virtual int GetNnodes()  {return 8;}
	virtual int GetNcoords() {return 8*3;}
	virtual int GetNdofs()   {return 8*3;}

	virtual ChSharedPtr<ChNodeFEMbase> GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(ChSharedPtr<ChNodeFEMxyz> nodeA, ChSharedPtr<ChNodeFEMxyz> nodeB, ChSharedPtr<ChNodeFEMxyz> nodeC, ChSharedPtr<ChNodeFEMxyz> nodeD,
						  ChSharedPtr<ChNodeFEMxyz> nodeE, ChSharedPtr<ChNodeFEMxyz> nodeF, ChSharedPtr<ChNodeFEMxyz> nodeG, ChSharedPtr<ChNodeFEMxyz> nodeH) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					nodes[2]=nodeC;
					nodes[3]=nodeD;
					nodes[4]=nodeE;
					nodes[5]=nodeF;
					nodes[6]=nodeG;
					nodes[7]=nodeH;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					mvars.push_back(&nodes[2]->Variables());
					mvars.push_back(&nodes[3]->Variables());
					mvars.push_back(&nodes[4]->Variables());
					mvars.push_back(&nodes[5]->Variables());
					mvars.push_back(&nodes[6]->Variables());
					mvars.push_back(&nodes[7]->Variables());
					Kmatr.SetVariables(mvars);
				}
			

			//
			// QUADRATURE functions
			//

	virtual void SetDefaultIntegrationRule()
			{
				this->ir->SetIntOnCube(8, &this->GpVector);
			}

	virtual void SetReducedIntegrationRule()
			{
				this->ir->SetIntOnCube(1, &this->GpVector);
			}

	virtual void SetIntegrationRule(int nPoints)
			{
				this->ir->SetIntOnCube(nPoints, &this->GpVector);
			}

	
			//
			// FEM functions
			//

	
				/// Fills the D vector (displacement) column matrix with the current 
				/// field values at the nodes of the element, with proper ordering.
				/// If the D vector has not the size of this->GetNdofs(), it will be resized.
				/// For corotational elements, field is assumed in local reference!
	virtual void GetField(ChMatrixDynamic<>& mD)
				{
					mD.Reset(this->GetNdofs(),1);
					
					for(int i=0; i<GetNnodes(); i++)
						mD.PasteVector(A.MatrT_x_Vect(this->nodes[i]->GetPos())-nodes[i]->GetX0(),i*3,0);
				}

				/// Puts inside 'Jacobian' and 'J1' the Jacobian matrix and the shape functions derivatives matrix of the element.
				/// The vector "coord" contains the natural coordinates of the integration point.
				/// in case of hexahedral elements natural coords vary in the classical range -1 ... +1.
	virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, ChMatrixDynamic<>& J1, ChVector<> coord) 
				{
					ChMatrixDynamic<> J2(8,3);

					J1.SetElement(0,0,-(1-coord.y)*(1-coord.z)/8);
					J1.SetElement(0,1,+(1-coord.y)*(1-coord.z)/8);
					J1.SetElement(0,2,+(1+coord.y)*(1-coord.z)/8);
					J1.SetElement(0,3,-(1+coord.y)*(1-coord.z)/8);
					J1.SetElement(0,4,-(1-coord.y)*(1+coord.z)/8);
					J1.SetElement(0,5,+(1-coord.y)*(1+coord.z)/8);
					J1.SetElement(0,6,+(1+coord.y)*(1+coord.z)/8);
					J1.SetElement(0,7,-(1+coord.y)*(1+coord.z)/8);

					J1.SetElement(1,0,-(1-coord.x)*(1-coord.z)/8);
					J1.SetElement(1,1,-(1+coord.x)*(1-coord.z)/8);
					J1.SetElement(1,2,+(1+coord.x)*(1-coord.z)/8);
					J1.SetElement(1,3,+(1-coord.x)*(1-coord.z)/8);
					J1.SetElement(1,4,-(1-coord.x)*(1+coord.z)/8);
					J1.SetElement(1,5,-(1+coord.x)*(1+coord.z)/8);
					J1.SetElement(1,6,+(1+coord.x)*(1+coord.z)/8);
					J1.SetElement(1,7,+(1-coord.x)*(1+coord.z)/8);

					J1.SetElement(2,0,-(1-coord.x)*(1-coord.y)/8);
					J1.SetElement(2,1,-(1+coord.x)*(1-coord.y)/8);
					J1.SetElement(2,2,-(1+coord.x)*(1+coord.y)/8);
					J1.SetElement(2,3,-(1-coord.x)*(1+coord.y)/8);
					J1.SetElement(2,4,+(1-coord.x)*(1-coord.y)/8);
					J1.SetElement(2,5,+(1+coord.x)*(1-coord.y)/8);
					J1.SetElement(2,6,+(1+coord.x)*(1+coord.y)/8);
					J1.SetElement(2,7,+(1-coord.x)*(1+coord.y)/8);

					J2.SetElement(0,0,nodes[0]->GetX0().x);
					J2.SetElement(1,0,nodes[1]->GetX0().x);
					J2.SetElement(2,0,nodes[2]->GetX0().x);
					J2.SetElement(3,0,nodes[3]->GetX0().x);
					J2.SetElement(4,0,nodes[4]->GetX0().x);
					J2.SetElement(5,0,nodes[5]->GetX0().x);
					J2.SetElement(6,0,nodes[6]->GetX0().x);
					J2.SetElement(7,0,nodes[7]->GetX0().x);
	
					J2.SetElement(0,1,nodes[0]->GetX0().y);
					J2.SetElement(1,1,nodes[1]->GetX0().y);
					J2.SetElement(2,1,nodes[2]->GetX0().y);
					J2.SetElement(3,1,nodes[3]->GetX0().y);
					J2.SetElement(4,1,nodes[4]->GetX0().y);
					J2.SetElement(5,1,nodes[5]->GetX0().y);
					J2.SetElement(6,1,nodes[6]->GetX0().y);
					J2.SetElement(7,1,nodes[7]->GetX0().y);

					J2.SetElement(0,2,nodes[0]->GetX0().z);
					J2.SetElement(1,2,nodes[1]->GetX0().z);
					J2.SetElement(2,2,nodes[2]->GetX0().z);
					J2.SetElement(3,2,nodes[3]->GetX0().z);
					J2.SetElement(4,2,nodes[4]->GetX0().z);
					J2.SetElement(5,2,nodes[5]->GetX0().z);
					J2.SetElement(6,2,nodes[6]->GetX0().z);
					J2.SetElement(7,2,nodes[7]->GetX0().z);

					Jacobian.MatrMultiply(J1,J2);				
				}


				/// Computes the matrix of partial derivatives and puts data in "MatrB"
				///	evaluated at natural coordinates zeta1,...,zeta4 . Also computes determinant of jacobian.
				/// note: in case of hexahedral elements natural coord. vary in the range -1 ... +1
	virtual void ComputeMatrB(ChMatrixDynamic<>& MatrB, double zeta1, double zeta2, double zeta3, double& JacobianDet) 
				{
					ChMatrixDynamic<> Jacobian(3,3);
					ChMatrixDynamic<> J1(3,8);
					ComputeJacobian(Jacobian, J1, ChVector<>(zeta1,zeta2,zeta3));
				
					double Jdet=Jacobian.Det();
					JacobianDet = Jdet;		// !!! store the Jacobian Determinant: needed for the integration

					ChMatrixDynamic<> Jinv = Jacobian;
					Jinv.MatrInverse();

					ChMatrixDynamic<> Btemp(3,8);
					Btemp.MatrMultiply(Jinv,J1);
					MatrB.Resize(6,24);	// Remember to resize the matrix!

					MatrB.SetElement(0,0,Btemp(0,0));
					MatrB.SetElement(0,3,Btemp(0,1));
					MatrB.SetElement(0,6,Btemp(0,2));
					MatrB.SetElement(0,9,Btemp(0,3));
					MatrB.SetElement(0,12,Btemp(0,4));
					MatrB.SetElement(0,15,Btemp(0,5));
					MatrB.SetElement(0,18,Btemp(0,6));
					MatrB.SetElement(0,21,Btemp(0,7));

					MatrB.SetElement(1,1,Btemp(1,0));
					MatrB.SetElement(1,4,Btemp(1,1));
					MatrB.SetElement(1,7,Btemp(1,2));
					MatrB.SetElement(1,10,Btemp(1,3));
					MatrB.SetElement(1,13,Btemp(1,4));
					MatrB.SetElement(1,16,Btemp(1,5));
					MatrB.SetElement(1,19,Btemp(1,6));
					MatrB.SetElement(1,22,Btemp(1,7));

					MatrB.SetElement(2,2,Btemp(2,0));
					MatrB.SetElement(2,5,Btemp(2,1));
					MatrB.SetElement(2,8,Btemp(2,2));
					MatrB.SetElement(2,11,Btemp(2,3));
					MatrB.SetElement(2,14,Btemp(2,4));
					MatrB.SetElement(2,17,Btemp(2,5));
					MatrB.SetElement(2,20,Btemp(2,6));
					MatrB.SetElement(2,23,Btemp(2,7));

					MatrB.SetElement(3,0,Btemp(1,0));
					MatrB.SetElement(3,1,Btemp(0,0));
					MatrB.SetElement(3,3,Btemp(1,1));
					MatrB.SetElement(3,4,Btemp(0,1));
					MatrB.SetElement(3,6,Btemp(1,2));
					MatrB.SetElement(3,7,Btemp(0,2));
					MatrB.SetElement(3,9,Btemp(1,3));
					MatrB.SetElement(3,10,Btemp(0,3));
					MatrB.SetElement(3,12,Btemp(1,4));
					MatrB.SetElement(3,13,Btemp(0,4));
					MatrB.SetElement(3,15,Btemp(1,5));
					MatrB.SetElement(3,16,Btemp(0,5));
					MatrB.SetElement(3,18,Btemp(1,6));
					MatrB.SetElement(3,19,Btemp(0,6));
					MatrB.SetElement(3,21,Btemp(1,7));
					MatrB.SetElement(3,22,Btemp(0,7));

					MatrB.SetElement(4,1,Btemp(2,0));
					MatrB.SetElement(4,2,Btemp(1,0));
					MatrB.SetElement(4,4,Btemp(2,1));
					MatrB.SetElement(4,5,Btemp(1,1));
					MatrB.SetElement(4,7,Btemp(2,2));
					MatrB.SetElement(4,8,Btemp(1,2));
					MatrB.SetElement(4,10,Btemp(2,3));
					MatrB.SetElement(4,11,Btemp(1,3));
					MatrB.SetElement(4,13,Btemp(2,4));
					MatrB.SetElement(4,14,Btemp(1,4));
					MatrB.SetElement(4,16,Btemp(2,5));
					MatrB.SetElement(4,17,Btemp(1,5));
					MatrB.SetElement(4,19,Btemp(2,6));
					MatrB.SetElement(4,20,Btemp(1,6));
					MatrB.SetElement(4,22,Btemp(2,7));
					MatrB.SetElement(4,23,Btemp(1,7));

					MatrB.SetElement(5,0,Btemp(2,0));
					MatrB.SetElement(5,2,Btemp(0,0));
					MatrB.SetElement(5,3,Btemp(2,1));
					MatrB.SetElement(5,5,Btemp(0,1));
					MatrB.SetElement(5,6,Btemp(2,2));
					MatrB.SetElement(5,8,Btemp(0,2));
					MatrB.SetElement(5,9,Btemp(2,3));
					MatrB.SetElement(5,11,Btemp(0,3));
					MatrB.SetElement(5,12,Btemp(2,4));
					MatrB.SetElement(5,14,Btemp(0,4));
					MatrB.SetElement(5,15,Btemp(2,5));
					MatrB.SetElement(5,17,Btemp(0,5));
					MatrB.SetElement(5,18,Btemp(2,6));
					MatrB.SetElement(5,20,Btemp(0,6));
					MatrB.SetElement(5,21,Btemp(2,7));
					MatrB.SetElement(5,23,Btemp(0,7));
				}


				/// Computes the matrix of partial derivatives and puts data in "GaussPt".
				///	Stores the determinant of the jacobian in "JacobianDet".
	virtual void ComputeMatrB(ChGaussPoint* GaussPt, double& JacobianDet) 
				{
					this->ComputeMatrB( *(GaussPt->MatrB), 
						(*GaussPt).GetLocalCoordinates().x, 
						(*GaussPt).GetLocalCoordinates().y, 
						(*GaussPt).GetLocalCoordinates().z,
						JacobianDet);
				}

				
				/// Computes the global STIFFNESS MATRIX of the element:    
				/// K = sum (w_i * [B]' * [D] * [B])
				/// The number of Gauss Point is defined by SetIntegrationRule function (default: 8 Gp).
	virtual void ComputeStiffnessMatrix() 
		{
			double Jdet;
			ChMatrixDynamic<> *temp = new ChMatrixDynamic<>;
			ChMatrixDynamic<> BT;
			this->Volume = 0;

			for(unsigned int i=0; i < GpVector.size(); i++)
			{
				ComputeMatrB(GpVector[i], Jdet);
				BT = *GpVector[i]->MatrB;
				BT.MatrTranspose();
				*temp = (BT * Material->Get_StressStrainMatrix() * *(GpVector[i]->MatrB));
				temp->MatrScale(GpVector[i]->GetWeight());
				temp->MatrScale(Jdet);
				StiffnessMatrix.MatrAdd(StiffnessMatrix,*temp);
				
				// by the way also computes volume:
				this->Volume  += GpVector[i]->GetWeight() * Jdet;
			}
			
			delete temp;
		}


	virtual void SetupInitial() 
			{
				ComputeStiffnessMatrix();

			}

					// compute large rotation of element for corotational approach
	virtual void UpdateRotation()
			{
				ChVector<> avgX1;
				avgX1 = this->nodes[0]->GetX0() + 
						this->nodes[1]->GetX0() + 
						this->nodes[2]->GetX0() + 
						this->nodes[3]->GetX0();
				ChVector<> avgX2;
				avgX2 = this->nodes[4]->GetX0() + 
						this->nodes[5]->GetX0() + 
						this->nodes[6]->GetX0() + 
						this->nodes[7]->GetX0();
				ChVector<> Xdir = avgX2 - avgX1;

				ChVector<> avgY1;
				avgY1 = this->nodes[0]->GetX0() + 
						this->nodes[1]->GetX0() + 
						this->nodes[4]->GetX0() + 
						this->nodes[5]->GetX0();
				ChVector<> avgY2;
				avgY2 = this->nodes[2]->GetX0() + 
						this->nodes[3]->GetX0() + 
						this->nodes[6]->GetX0() + 
						this->nodes[7]->GetX0();
				ChVector<> Ydir = avgY2 - avgY1;
				ChMatrix33<> rotX0;
				rotX0.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

				avgX1 = this->nodes[0]->pos + 
						this->nodes[1]->pos + 
						this->nodes[2]->pos + 
						this->nodes[3]->pos;
				avgX2 = this->nodes[4]->pos + 
						this->nodes[5]->pos + 
						this->nodes[6]->pos + 
						this->nodes[7]->pos;
				Xdir = avgX2 - avgX1;

				avgY1 = this->nodes[0]->pos + 
						this->nodes[1]->pos + 
						this->nodes[4]->pos + 
						this->nodes[5]->pos;
				avgY2 = this->nodes[2]->pos + 
						this->nodes[3]->pos + 
						this->nodes[6]->pos + 
						this->nodes[7]->pos;
				Ydir = avgY2 - avgY1;
				ChMatrix33<> rotXcurrent;
				rotXcurrent.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

				this->A.MatrMultiplyT(rotXcurrent,rotX0);
			}

				/// Returns the strain tensor at given parameters. 
				/// The tensor is in the original undeformed unrotated reference.
	ChStrainTensor<> GetStrain(double z1, double z2,  double z3) 
				{
					// set up vector of nodal displacements (in local element system) u_l = R*p - p0
					ChMatrixDynamic<> displ(GetNdofs(),1);
					this->GetField(displ);
					
					double JacobianDet;
					ChMatrixDynamic<> amatrB(6, GetNdofs()); 
					ComputeMatrB(amatrB, z1, z2, z3, JacobianDet);
					
					ChStrainTensor<> mstrain;
					mstrain.MatrMultiply(amatrB, displ);
					return mstrain;
				}

				/// Returns the stress tensor at given parameters.  
				/// The tensor is in the original undeformed unrotated reference.
	ChStressTensor<> GetStress(double z1, double z2,  double z3)
				{
					ChStressTensor<> mstress;
					mstress.MatrMultiply(this->Material->Get_StressStrainMatrix(), this->GetStrain(z1,z2,z3));
					return mstress;
				}

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == GetNdofs()) && (H.GetColumns()==GetNdofs()));

					// warp the local stiffness matrix K in order to obtain global 
					// tangent stiffness CKCt:
					ChMatrixDynamic<> CK(GetNdofs(), GetNdofs());
					ChMatrixDynamic<> CKCt(GetNdofs(), GetNdofs()); // the global, corotated, K matrix, for 8 nodes
					ChMatrixCorotation<>::ComputeCK(StiffnessMatrix, this->A, 8, CK);
					ChMatrixCorotation<>::ComputeKCt(CK, this->A, 8, CKCt);

					// For K stiffness matrix and R damping matrix:

					double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

					CKCt.MatrScale( mkfactor );

					H.PasteMatrix(&CKCt,0,0);


					// For M mass matrix:
					if (Mfactor)
					{
						double lumped_node_mass = (this->Volume * this->Material->Get_density() ) / 8.0;
						for (int id = 0; id < GetNdofs(); id++)
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
					assert((Fi.GetRows() == GetNdofs()) && (Fi.GetColumns()==1));

						// set up vector of nodal displacements (in local element system) u_l = R*p - p0
					ChMatrixDynamic<> displ(GetNdofs(),1);
					this->GetField(displ);

						// [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
					ChMatrixDynamic<> FiK_local(GetNdofs(),1);
					FiK_local.MatrMultiply(StiffnessMatrix, displ);

					for (int in=0; in < 8; ++in)
					{
						displ.PasteVector(A.MatrT_x_Vect(nodes[in]->pos_dt), in*3, 0); // nodal speeds, local
					}
					ChMatrixDynamic<> FiR_local(GetNdofs(),1);
					FiR_local.MatrMultiply(StiffnessMatrix, displ);
					FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

					double lumped_node_mass = (this->Volume * this->Material->Get_density() ) / 8.0;
					displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM() ); // reuse 'displ' for performance 
					FiR_local.MatrInc(displ); 
					//***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

					FiK_local.MatrInc(FiR_local);

					FiK_local.MatrScale(-1.0);

						// Fi = C * Fi_local  with C block-diagonal rotations A
					ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 8, Fi);
				}

			//
			// Custom properties functions
			//

				/// Set the material of the element
	void   SetMaterial( ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
	ChSharedPtr<ChContinuumElastic> GetMaterial() {return Material;}

				/// Get the StiffnessMatrix
	ChMatrix<>& GetStiffnessMatrix() {return StiffnessMatrix;}
				/// Get the Nth gauss point
	ChGaussPoint* GetGaussPoint(int N) {return GpVector[N];}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)




};

}//___end of namespace fem___
}//___end of namespace chrono___

#endif
