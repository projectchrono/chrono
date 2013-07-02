#ifndef CHELEMENTSPRING_H
#define CHELEMENTSPRING_H

//////////////////////////////////////////////////
//
//   ChElementSpring.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "fem/ChElementGeneric.h"
#include "fem/ChNodeFEMxyz.h"


namespace chrono
{
namespace fem
{



/// Simple finite element with two nodes and a spring/damper
/// between the two nodes.

class ChApi ChElementSpring : public ChElementGeneric
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
				/// superimposes global damping matrix R, scaled by Rfactor.
				/// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
	virtual void ComputeKRmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0) 
				{
					assert((H.GetRows() == 6) && (H.GetColumns()==6));

					// compute stiffness matrix (this is already the explicit
					// formulation of the corotational stiffness matrix in 3D)
					
					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					ChMatrixDynamic<> dircolumn; 
					dircolumn.PasteVector(dir, 0,0);

					ChMatrix33<> submatr;
					submatr.MatrTMultiply(dircolumn, dircolumn);

						// note that stiffness and damping matrices are the same, so join stuff here
					double commonfactor = this->spring_k * Kfactor + 
										  this->damper_r * Rfactor ;
					submatr.MatrScale(commonfactor);
					H.PasteMatrix(&submatr,0,0);
					H.PasteMatrix(&submatr,3,3);
					submatr.MatrNeg();
					H.PasteMatrix(&submatr,0,3);
					H.PasteMatrix(&submatr,3,0);
				}

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor.
				/// This is usually called only once in the simulation. 
	virtual void ComputeKRmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0)
				{
					assert((Hl.GetRows() == 6) && (Hl.GetColumns() == 6));

					// to keep things short, here local K is as global K (anyway, only global K is used in simulations)
					ComputeKRmatricesLocal (Hl, Kfactor, Rfactor);
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
					double internal_Kforce_local = this->spring_k * (L - L_ref); 
					double internal_Rforce_local = this->spring_k * (L - L_ref);
					double internal_force_local = internal_Kforce_local + internal_Rforce_local;
					ChMatrixDynamic<> displacements(6,1);
					ChVector<> int_forceA =  dir * internal_force_local;
					ChVector<> int_forceB = -dir * internal_force_local;
					Fi.PasteVector(int_forceA, 0,0);
					Fi.PasteVector(int_forceB, 3,0);
				}

				/// Setup. Precompute matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
				/// (**Not needed for the spring element because global K is computed on-the-fly in ComputeAddKRmatricesGlobal() )
	virtual void Setup() {}


			//
			// Custom properties functions
			//

				/// Set the stiffness of the spring that connects the two nodes (N/m)
	void   SetSpringK(double ms) { spring_k = ms;}
	double GetSpringK() {return spring_k;}

				/// Set the damping of the damper that connects the two nodes (Ns/M)
	void   SetDamperR(double md) { damper_r = md;}
	double GetDamperR() {return damper_r;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






