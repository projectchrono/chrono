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
// File authors: Alessandro Tasora

#ifndef CHELEMENTSPRING_H
#define CHELEMENTSPRING_H



#include "ChElementGeneric.h"
#include "ChNodeFEMxyz.h"


namespace chrono
{
namespace fem
{



/// Simple finite element with two nodes and a spring/damper
/// between the two nodes.
/// This element is mass-less, so if used in dynamic analysis,
/// the two nodes must be set with non-zero point mass.

class ChApiFem ChElementSpring : public ChElementGeneric
{
protected:
	std::vector<ChNodeFEMxyz*> nodes;
	double spring_k;
	double damper_r;
public:

	ChElementSpring();
	virtual ~ChElementSpring();

	virtual int GetNcoords() {return 6;}
	virtual int GetNnodes()  {return 2;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}
	

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					Kmatr.SetVariables(mvars);
				}

			//
			// FEM functions
			//

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
				/// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 6) && (H.GetColumns()==6));

					// compute stiffness matrix (this is already the explicit
					// formulation of the corotational stiffness matrix in 3D)
					
					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					ChMatrixNM<double,3,1> dircolumn; 
					dircolumn.PasteVector(dir, 0,0);

					ChMatrix33<> submatr;
					submatr.MatrMultiplyT(dircolumn, dircolumn);

						// note that stiffness and damping matrices are the same, so join stuff here
					double commonfactor = this->spring_k * Kfactor + 
										  this->damper_r * Rfactor ;
					submatr.MatrScale(commonfactor);
					H.PasteMatrix(&submatr,0,0);
					H.PasteMatrix(&submatr,3,3);
					submatr.MatrNeg();
					H.PasteMatrix(&submatr,0,3);
					H.PasteMatrix(&submatr,3,0);
					 
					  // finally, do nothing about mass matrix because this element is mass-less 
				}


				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 6) && (Fi.GetColumns()==1));

					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					double L_ref = (nodes[1]->GetX0()  - nodes[0]->GetX0() ).Length();
					double L     = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
					double L_dt  = Vdot((nodes[1]->GetPos_dt() - nodes[0]->GetPos_dt()), dir);
					double internal_Kforce_local = this->spring_k * (L - L_ref); 
					double internal_Rforce_local = this->damper_r * L_dt;
					double internal_force_local = internal_Kforce_local + internal_Rforce_local;
					ChMatrixDynamic<> displacements(6,1);
					ChVector<> int_forceA =  dir * internal_force_local;
					ChVector<> int_forceB = -dir * internal_force_local;
					Fi.PasteVector(int_forceA, 0,0);
					Fi.PasteVector(int_forceB, 3,0);
				}

				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
				/// (**Not needed for the spring element because global K is computed on-the-fly in ComputeAddKRmatricesGlobal() )
	virtual void SetupInitial() {}


			//
			// Custom properties functions
			//

				/// Set the stiffness of the spring that connects the two nodes (N/m)
	virtual void   SetSpringK(double ms) { spring_k = ms;}
	virtual double GetSpringK() {return spring_k;}

				/// Set the damping of the damper that connects the two nodes (Ns/M)
	virtual void   SetDamperR(double md) { damper_r = md;}
	virtual double GetDamperR() {return damper_r;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






