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

#ifndef CHELEMENTBEAMEULER_H
#define CHELEMENTBEAMEULER_H


#include "ChElementGeneric.h"
#include "ChElement3D.h"
#include "ChNodeFEMxyzrot.h"


namespace chrono
{
namespace fem
{



/// Simple beam element with two nodes and Euler-Bernoulli 
/// formulation.
/// For this 'basic' implementation, constant section and 
/// constant material are assumed.

class ChApiFem ChElementBeamEuler : public ChElementGeneric,
								    public ChElementCorotational
{
protected:
	std::vector< ChSharedPtr<ChNodeFEMxyzrot> > nodes;
	double Area;
	double Iyy;
	double Izz;
	double G;
	double E;
	double density;
	double rdamping;
	double mass;
	double length;
	ChMatrixDynamic<> StiffnessMatrix; // undeformed local stiffness matrix
	ChQuaternion<> q_refrot1;

public:

	ChElementBeamEuler()
				{
					nodes.resize(2);

					SetAsRectangularSection(0.01, 0.01); // defaults Area, Ixx, Iyy
				
					E = 0.01e9;		  // default stiffness: rubber
					G = 0.3 * E;

					density = 1000;   // default density: water
					rdamping = 0.01;  // default raleygh damping.

					length = 0;		// will be computed by Setup(), later
					mass = 0;		// will be computed by Setup(), later

					this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());

					q_refrot1 = QUNIT;
				}

	virtual ~ChElementBeamEuler() {}

	virtual int GetNnodes()  {return 2;}
	virtual int GetNcoords() {return 2*6;}
	virtual int GetNdofs()   {return 2*6;}

	virtual ChSharedPtr<ChNodeFEMbase> GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes( ChSharedPtr<ChNodeFEMxyzrot> nodeA, ChSharedPtr<ChNodeFEMxyzrot> nodeB) 
				{
					assert(!nodeA.IsNull());
					assert(!nodeB.IsNull());

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

				/// Fills the N matrix (single row, 12 columns) with the
				/// values of shape functions at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
	virtual void ShapeFunctions(ChMatrix<>& N, double eta)
				{
					double Nx1 = (1./2.)*(1-eta);
					double Nx2 = (1./2.)*(1+eta);
					double Ny1 = (1./4.)*pow((1-eta),2)*(2+eta);
					double Ny2 = (1./4.)*pow((1+eta),2)*(2-eta);
					double Nr1 = (this->length/8.)*pow((1-eta),2)*(1+eta);
					double Nr2 = (this->length/8.)*pow((1+eta),2)*(eta-1);
					N(0) =Nx1;
					N(1) =Ny1;
					N(2) =Ny1;
					N(3) =Nx1;
					N(4) =-Nr1;
					N(5) =Nr1;
					N(6) =Nx2;
					N(7) =Ny2;
					N(8) =Ny2;
					N(9) =Nx2;
					N(10)=-Nr2;
					N(11)=Nr2;
				};

	virtual void Update() 
				{
					// parent class update:
					ChElementGeneric::Update();

					// always keep updated the rotation matrix A:
					this->UpdateRotation();
				};

				/// Compute large rotation of element for corotational approach
				/// The reference frame of this Euler-Bernoulli beam has X aligned to two nodes and Y parallel to Y of 1st node
	virtual void UpdateRotation()
				{
					ChVector<> mXele = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
					ChVector<> myele = nodes[0]->Frame().GetA()->Get_A_Yaxis();
					A.Set_A_Xdir(mXele, myele);
				}

				/// Fills the D vector (column matrix) with the current 
				/// field values at the nodes of the element, with proper ordering.
				/// If the D vector has not the size of this->GetNdofs(), it will be resized.
				/// For corotational elements, field is assumed in local reference!
				/// Give that this element includes rotations at nodes, this gives: 
				///  {x_a y_a z_a Rx_a Ry_a Rz_a x_b y_b z_b Rx_b Ry_b Rz_b}
	virtual void GetField(ChMatrixDynamic<>& mD)
				{
					mD.Reset(12,1);

					ChVector<> delta_rot_dir;
					double     delta_rot_angle;

					// corotational frame 
					ChQuaternion<> quatA = A.Get_A_quaternion();

					// Node 0, displacement (in local element frame, corotated back by A' )
					mD.PasteVector(A.MatrT_x_Vect(nodes[0]->Frame().GetPos()) - nodes[0]->GetX0().GetPos(), 0, 0);

					// Node 0, x,y,z small rotations (in local element frame, corotated back by A' )
					//  with quaternion algebra: q_delta = q_corotation' * q_node  
					//  with linear algebra in SO(3): [A_delta] = [A_coro.]'[A_node]
					ChQuaternion<> q_delta0 =  quatA.GetConjugate() % nodes[0]->Frame().GetRot();
						// note, for small incremental rotations this is opposite of ChNodeFEMxyzrot::VariablesQbIncrementPosition 
					q_delta0.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); 
					mD.PasteVector(delta_rot_dir*delta_rot_angle, 3, 0);

					// Node 1, displacement (in local element frame, corotated back by A' )
					mD.PasteVector(A.MatrT_x_Vect(nodes[1]->Frame().GetPos()) - nodes[1]->GetX0().GetPos(), 6, 0);

					// Node 1, x,y,z small rotations (in local element frame, corotated back by A' )
					//  with quaternion algebra: q_delta = q_refrot1' * q_corotation' * q_node  
					//  with linear algebra in SO(3): [A_delta] = [A_refrot1]'[A_coro.]'[A_node]
					ChQuaternion<> q_delta1 = this->q_refrot1.GetConjugate() % (quatA.GetConjugate() % nodes[1]->Frame().GetRot());
						// note, for small incremental rotations this is opposite of ChNodeFEMxyzrot::VariablesQbIncrementPosition 
					q_delta1.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); 
					mD.PasteVector(delta_rot_dir*delta_rot_angle, 9, 0);

				}

				/// Fills the Ddt vector (column matrix) with the current 
				/// tme derivatives of field values at the nodes of the element, with proper ordering.
				/// If the D vector has not the size of this->GetNdofs(), it will be resized.
				/// For corotational elements, field is assumed in local reference!
				/// Give that this element includes rotations at nodes, this gives: 
				///  {v_a v_a v_a wx_a wy_a wz_a v_b v_b v_b wx_b wy_b wz_b}
	virtual void GetField_dt(ChMatrixDynamic<>& mD_dt)
				{
					mD_dt.Reset(12,1);

					// corotational frame 
					ChQuaternion<> quatA = A.Get_A_quaternion();

					// Node 0, velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(A.MatrT_x_Vect(nodes[0]->Frame().GetPos_dt()),    0, 0);

					// Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(A.MatrT_x_Vect(nodes[0]->Frame().GetWvel_par() ), 3, 0);

					// Node 1, velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(A.MatrT_x_Vect(nodes[1]->Frame().GetPos_dt()),    6, 0);

					// Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(A.MatrT_x_Vect(nodes[1]->Frame().GetWvel_par() ), 9, 0);

				}
				
				/// Computes the local STIFFNESS MATRIX of the element:    
				/// K = integral( [B]' * [D] * [B] ),
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed, so the explicit result of quadrature is used. 
	virtual void ComputeStiffnessMatrix()
				{	
					double om_xz = 0; // For Euler-Bernoulli
					double om_xy = 0; // For Euler-Bernoulli
					// double om_xz = ...; // For Reddy's RBT 
					// double om_xy = ...; // For Reddy's RBT 
					double u_xz = 1+12*om_xz;
					double l_xz = 1+3*om_xz;
					double e_xz = 1-6*om_xz;
					double u_xy = 1+12*om_xy;
					double l_xy = 1+3*om_xy;
					double e_xy = 1-6*om_xy;

					double h = this->length;
					double Jpolar = Izz+Iyy;
					double k_u = E*Area/h;
					double k_f = G*Jpolar/h;
					
					double k_w = 12*E*Iyy / (u_xz*h*h*h);
					double k_v = 12*E*Izz / (u_xy*h*h*h);

					double k_t = 4*E*Iyy*l_xz / (u_xz*h);
					double k_p = 4*E*Izz*l_xy / (u_xy*h);

					double k_wt = 6*E*Iyy / (u_xz*h*h);
					double k_vp = 6*E*Izz / (u_xy*h*h);

					double k_tt = 2*E*Iyy*e_xz / (u_xz*h);
					double k_pp = 2*E*Izz*e_xy / (u_xy*h);

					StiffnessMatrix(0,0)  = k_u;
					StiffnessMatrix(1,1)  = k_v;
					StiffnessMatrix(2,2)  = k_w;
					StiffnessMatrix(3,3)  = k_f;
					StiffnessMatrix(4,4)  = k_t;
					StiffnessMatrix(5,5)  = k_p;
					StiffnessMatrix(6,6)  = k_u;
					StiffnessMatrix(7,7)  = k_v;
					StiffnessMatrix(8,8)  = k_w;
					StiffnessMatrix(9,9)  = k_f;
					StiffnessMatrix(10,10)= k_t;
					StiffnessMatrix(11,11)= k_p;
	
					StiffnessMatrix(0,6)  = -k_u;
					StiffnessMatrix(1,7)  = -k_v;
					StiffnessMatrix(2,8)  = -k_w;
					StiffnessMatrix(3,9)  = -k_f;
					StiffnessMatrix(4,10) =  k_tt;
					StiffnessMatrix(5,11) =  k_pp;

					StiffnessMatrix(4,8)  =  k_wt;
					StiffnessMatrix(5,7)  = -k_vp;
					StiffnessMatrix(1,11) =  k_vp;
					StiffnessMatrix(2,10) = -k_wt;

					StiffnessMatrix(1,5)  =  k_vp;
					StiffnessMatrix(2,4)  = -k_wt;
					StiffnessMatrix(7,11) = -k_vp;
					StiffnessMatrix(8,10) =  k_wt;

					// symm part;
					for (int r = 0; r < 12; r++)
						for (int c = r+1; c < 12; c++)
							StiffnessMatrix(c,r) = StiffnessMatrix(r,c);
				}


				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.

	virtual void SetupInitial() 
				{
					// Compute rest length, mass:
					this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
					this->mass   = this->length * this->Area * this->density;

					// Compute local stiffness matrix:
					ComputeStiffnessMatrix();
				}


				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 12) && (H.GetColumns()==12));

					// Corotational K stiffness:
					ChMatrixDynamic<> CK(12,12);
					ChMatrixDynamic<> CKCt(12,12); // the global, corotated, K matrix
					ChMatrixCorotation<>::ComputeCK(this->StiffnessMatrix, this->A, 4, CK);
					ChMatrixCorotation<>::ComputeKCt(CK, this->A, 4, CKCt);


					// For K stiffness matrix and R matrix: scale by factors

					double mkrfactor = Kfactor + Rfactor * this->rdamping;  

					CKCt.MatrScale( mkrfactor );

					H.PasteMatrix(&CKCt,0,0); // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K] 


						
					// For M mass matrix, do mass lumping:

					double lmass = mass*0.5;
					double liner = mass*0.5* pow((this->length* 0.25),2); //***APPROX just for test 
					
					H(0,0) += Mfactor* lmass; //node A x,y,z
					H(1,1) += Mfactor* lmass;
					H(2,2) += Mfactor* lmass;

					H(3,3) += Mfactor* liner; //node A Ixx,Iyy,Izz
					H(4,4) += Mfactor* liner;
					H(5,5) += Mfactor* liner;

					H(6,6) += Mfactor* lmass; //node B x,y,z
					H(7,7) += Mfactor* lmass;
					H(8,8) += Mfactor* lmass;

					H(9,9) +=  Mfactor* liner; //node B Ixx,Iyy,Izz
					H(10,10)+= Mfactor* liner;
					H(11,11)+= Mfactor* liner;
					
					//***TO DO*** better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform materials.
				}


				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 12) && (Fi.GetColumns()==12));

						// set up vector of nodal displacements and small rotations (in local element system) 
					ChMatrixDynamic<> displ(12,1);
					this->GetField(displ);
					
						// [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
					ChMatrixDynamic<> FiK_local(12,1);
					FiK_local.MatrMultiply(StiffnessMatrix, displ);

						// set up vector of nodal velocities (in local element system) 
					ChMatrixDynamic<> displ_dt(12,1);
					this->GetField_dt(displ_dt);

					ChMatrixDynamic<> FiR_local(12,1);
					FiR_local.MatrMultiply(StiffnessMatrix, displ_dt);
					FiR_local.MatrScale(this->GetBeamRaleyghDamping());

					FiK_local.MatrInc(FiR_local);


					FiK_local.MatrScale(-1.0);

						// Fi = C * Fi_local  with C block-diagonal rotations A  , for nodal forces in abs. frame 
					ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 4, Fi);
				}

			//
			// Custom properties functions
			//

				/// Set the cross sectional area A of the beam (m^2) 
	void   SetArea(double ma) { this->Area = ma;  }
	double GetArea() {return this->Area;}

				/// Set the Iyy moment of inertia of the beam 
	void   SetIyy(double ma) { this->Iyy = ma;  }
	double GetIyy() {return this->Iyy;}

				/// Set the Iyy moment of inertia of the beam 
	void   SetIzz(double ma) { this->Izz = ma;  }
	double GetIzz() {return this->Izz;}

				/// Shortcut: set area, Ixx and Iyy moment of inertia  
				/// at once, given the y and z widths of the beam assumed
				/// with rectangular shape.
	void   SetAsRectangularSection(double width_y, double width_z) 
				{ 
					this->Area = width_y * width_z; 
					this->Izz = (1.0/12.0)*width_z*pow(width_y,3);
					this->Iyy = (1.0/12.0)*width_y*pow(width_z,3);
				}


				/// Set the density of the beam (kg/m^3)
	void   SetDensity(double md) { this->density = md;  }
	double GetDensity() {return this->density;}

				/// Set E, the Young elastic modulus (N/m^2) 
	void   SetYoungModulus(double mE) { this->E = mE; }
	double GetYoungModulus() {return this->E;}
				/// Set G, the shear modulus 
	void   SetGshearModulus(double mG) { this->G = mG; }
	double GetGshearModulus() {return this->G;}

				/// Set the Raleygh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
	void   SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
	double GetBeamRaleyghDamping() {return this->rdamping;}

				/// The full mass of the beam, (with const. section, density, etc.)
	double  GetMass() {return this->mass;}

				/// The rest length of the bar
	double  GetRestLength() {return this->length;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






