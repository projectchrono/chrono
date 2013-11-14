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
// File author: Alessandro Tasora


#ifndef CHELEMENTBAR_H
#define CHELEMENTBAR_H


#include "ChElementSpring.h"


namespace chrono
{
namespace fem
{



/// Simple finite element with two nodes and a bar that
/// connect them, without bending and torsion stiffness,
/// just like a bar with two spherical joints. 
/// In practical terms, it works a bit like the 
/// class ChElementSpring, but also adds mass along the
/// element, hence point-like mass in the two nodes is not
/// needed.

class ChApiFem ChElementBar : public ChElementGeneric
{
protected:
	std::vector<ChNodeFEMxyz*> nodes;
	double area;
	double density;
	double E;
	double rdamping;
	double mass;
	double length;
public:

	ChElementBar();
	virtual ~ChElementBar();

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

					// For K stiffness matrix and R damping matrix:
					// compute stiffness matrix (this is already the explicit
					// formulation of the corotational stiffness matrix in 3D)
					
					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					ChMatrixNM<double,3,1> dircolumn; 
					dircolumn.PasteVector(dir, 0,0);

					ChMatrix33<> submatr;
					submatr.MatrMultiplyT(dircolumn, dircolumn);
	
					double Kstiffness= ((this->area*this->E)/this->length);
					double Rdamping = this->rdamping * Kstiffness;

						// note that stiffness and damping matrices are the same, so join stuff here
					double commonfactor = Kstiffness * Kfactor + 
										  Rdamping   * Rfactor ;
					submatr.MatrScale(commonfactor);
					H.PasteMatrix(&submatr,0,0);
					H.PasteMatrix(&submatr,3,3);
					submatr.MatrNeg();
					H.PasteMatrix(&submatr,0,3);
					H.PasteMatrix(&submatr,3,0);
						
					// For M mass matrix, do mass lumping:
					H(0,0) += Mfactor* mass*0.5; //node A x,y,z
					H(1,1) += Mfactor* mass*0.5;
					H(2,2) += Mfactor* mass*0.5;
					H(3,3) += Mfactor* mass*0.5; //node B x,y,z
					H(4,4) += Mfactor* mass*0.5;
					H(5,5) += Mfactor* mass*0.5;
				}


				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
	virtual void SetupInitial() 
				{
					// Compute rest length, mass:
					this->length = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
					this->mass   = this->length * this->area * this->density;
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
					double Kstiffness= ((this->area*this->E)/this->length);
					double Rdamping = this->rdamping * Kstiffness;
					double internal_Kforce_local = Kstiffness * (L - L_ref); 
					double internal_Rforce_local = Rdamping * L_dt;
					double internal_force_local = internal_Kforce_local + internal_Rforce_local;
					ChMatrixDynamic<> displacements(6,1);
					ChVector<> int_forceA =  dir * internal_force_local;
					ChVector<> int_forceB = -dir * internal_force_local;
					Fi.PasteVector(int_forceA, 0,0);
					Fi.PasteVector(int_forceB, 3,0);
				}

			//
			// Custom properties functions
			//

				/// Set the cross sectional area of the bar (m^2) (also changes stiffness keeping same E modulus)
	void   SetBarArea(double ma) { this->area = ma;  }
	double GetBarArea() {return this->area;}

				/// Set the density of the bar (kg/m^3)
	void   SetBarDensity(double md) { this->density = md;  }
	double GetBarDensity() {return this->density;}

				/// Set the Young elastic modulus (N/m^2) (also sets stiffness)
	void   SetBarYoungModulus(double mE) { this->E = mE; }
	double GetBarYoungModulus() {return this->E;}

				/// Set the Raleygh damping ratio r (as in: R = r * K )
	void   SetBarRaleyghDamping(double mr) { this->rdamping = mr; }
	double GetBarRaleyghDamping() {return this->rdamping;}

				/// The full mass of the bar
	double  GetMass() {return this->mass;}

				/// The rest length of the bar
	double  GetRestLength() {return this->length;}

				/// The current length of the bar (might be after deformation)
	double  GetCurrentLength() {return (nodes[1]->GetPos()-nodes[0]->GetPos()).Length(); }
		
				/// Get the strain epsilon, after deformation.
	double  GetStrain() { return (GetCurrentLength()-GetRestLength())/GetRestLength();}

				/// Get the stress sigma, after deformation.
	double  GetStress() { return GetBarYoungModulus()*GetStrain();}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






