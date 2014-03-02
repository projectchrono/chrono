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

//#define BEAM_VERBOSE

#include "ChElementBeam.h"
#include "ChBeamSection.h"


namespace chrono
{
namespace fem
{



/// Simple beam element with two nodes and Euler-Bernoulli 
/// formulation.
/// For this 'basic' implementation, constant section and 
/// constant material are assumed.

class ChApiFem ChElementBeamEuler : public ChElementBeam
{
protected:
	std::vector< ChSharedPtr<ChNodeFEMxyzrot> > nodes;
	
	ChSharedPtr<ChBeamSectionAdvanced> section;

	ChMatrixDynamic<> StiffnessMatrix; // undeformed local stiffness matrix

	ChQuaternion<> q_refrot1;

	ChQuaternion<> q_element_abs_rot;
	ChQuaternion<> q_element_ref_rot;

	bool disable_corotate;

public:

	ChElementBeamEuler()
				{
					nodes.resize(2);

					this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());

					q_refrot1 = QUNIT;
					q_element_abs_rot = QUNIT;
					q_element_ref_rot = QUNIT;
					disable_corotate = false;
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

				/// Set the section & material of beam element .
				/// It is a shared property, so it can be shared between other beams.
	void   SetSection( ChSharedPtr<ChBeamSectionAdvanced> my_material) { section = my_material; }
				/// Get the section & material of the element
	ChSharedPtr<ChBeamSectionAdvanced> GetSection() {return section;}

				/// Get the absolute rotation of element in space 
				/// This is not the same of Rotation() , that expresses 
				/// the accumulated rotation from starting point.
	ChQuaternion<> GetAbsoluteRotation() {return q_element_abs_rot;}

				/// Get the original reference rotation of element in space 
	ChQuaternion<> GetRefRotation() {return q_element_ref_rot;}

				/// Set this as true to have the beam behave like a non-corotated beam
				/// (i.e. only for small deformations).
	void SetDisableCorotate(bool md) {disable_corotate = md;}

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
					ChMatrix33<> A0(this->q_element_ref_rot); 

					ChMatrix33<> Aabs;
					if (this->disable_corotate)
					{
						Aabs = A0;
						q_element_abs_rot = q_element_ref_rot;
					}
					else
					{
						ChVector<> mXele = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
						ChVector<> myele = nodes[0]->Frame().GetA()->Get_A_Yaxis();
						Aabs.Set_A_Xdir(mXele, myele);
						q_element_abs_rot = Aabs.Get_A_quaternion(); 
					}

					this->A.MatrTMultiply(A0,Aabs);
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

					//ChMatrix33<> A0  (this->q_element_ref_rot);
					//ChMatrix33<> Aabs(this->q_element_abs_rot);

					ChVector<> delta_rot_dir;
					double     delta_rot_angle;

					// Node 0, displacement (in local element frame, corotated back)
					//     d = [Atw]' Xt - [A0w]'X0   
					ChVector<> displ = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos()) - 
									   this->q_element_ref_rot.RotateBack(nodes[0]->GetX0().GetPos());
					mD.PasteVector(displ, 0, 0);

					// Node 0, x,y,z small rotations (in local element frame)
					ChQuaternion<> q_delta0 =   q_element_abs_rot.GetConjugate() % 
												nodes[0]->Frame().GetRot();
						// note, for small incremental rotations this is opposite of ChNodeFEMxyzrot::VariablesQbIncrementPosition 
					q_delta0.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); 
					mD.PasteVector(delta_rot_dir*delta_rot_angle, 3, 0);
 
					// Node 1, displacement (in local element frame, corotated back)
					//     d = [Atw]' Xt - [A0w]'X0 
					displ			 = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos()) - 
									   this->q_element_ref_rot.RotateBack(nodes[1]->GetX0().GetPos());
					mD.PasteVector(displ, 6, 0);

					// Node 1, x,y,z small rotations (in local element frame)
					ChQuaternion<> q_delta1 =   // this->q_refrot1.GetConjugate() %  
												q_element_abs_rot.GetConjugate() % 
												nodes[1]->Frame().GetRot();
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

					// Node 0, velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos_dt()),    0, 0);

					// Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetWvel_par() ), 3, 0);

					// Node 1, velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos_dt()),    6, 0);

					// Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
					mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetWvel_par() ), 9, 0);

				}
				
				/// Computes the local STIFFNESS MATRIX of the element:    
				/// K = integral( [B]' * [D] * [B] ),
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed, so the explicit result of quadrature is used. 
	virtual void ComputeStiffnessMatrix()
				{	
					assert (!section.IsNull());

					double Area = section->Area;
					double E    = section->E;
					double Izz  = section->Izz;
					double Iyy  = section->Iyy;
					double G    = section->G;

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

					// In case the section is rotated:
					if (this->section->alpha)
					{
						// Do [K]^ = [R][K][R]'
						ChMatrix33<> Rotsect;
						Rotsect.Set_A_Rxyz(ChVector<>(-section->alpha,0,0));
						ChMatrixDynamic<> CKtemp(12,12);
						ChMatrixCorotation<>::ComputeCK (this->StiffnessMatrix, Rotsect, 4, CKtemp);
						ChMatrixCorotation<>::ComputeKCt(CKtemp, Rotsect, 4, this->StiffnessMatrix);
					}
					// In case the section has a centroid displacement:
					if (this->section->Cy || this->section->Cz)
					{
						// Do [K]" = [T_c][K]^[T_c]'

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(4,i) +=  this->section->Cz * this->StiffnessMatrix(0,i);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(5,i) += -this->section->Cy * this->StiffnessMatrix(0,i);

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(10,i) +=  this->section->Cz * this->StiffnessMatrix(6,i);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(11,i) += -this->section->Cy * this->StiffnessMatrix(6,i);

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,4) +=  this->section->Cz * this->StiffnessMatrix(i,0);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,5) += -this->section->Cy * this->StiffnessMatrix(i,0);

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,10) +=  this->section->Cz * this->StiffnessMatrix(i,6);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,11) += -this->section->Cy * this->StiffnessMatrix(i,6);
					}

					// In case the section has a shear center displacement:
					if (this->section->Sy || this->section->Sz)
					{
						// Do [K]° = [T_s][K]"[T_s]'

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(3,i) +=  this->section->Sz * this->StiffnessMatrix(1,i)  
														  -this->section->Sy * this->StiffnessMatrix(2,i);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(9,i) +=  this->section->Sz * this->StiffnessMatrix(7,i)  
														  -this->section->Sy * this->StiffnessMatrix(8,i);

						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,3) +=  this->section->Sz * this->StiffnessMatrix(i,1)  
														  -this->section->Sy * this->StiffnessMatrix(i,2);
						for (int i = 0; i<12; ++i)
							this->StiffnessMatrix(i,9) +=  this->section->Sz * this->StiffnessMatrix(i,7)  
														  -this->section->Sy * this->StiffnessMatrix(i,8);
					}

				}


				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.

	virtual void SetupInitial() 
				{
					assert (!section.IsNull());

					// Compute rest length, mass:
					this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
					this->mass   = this->length * this->section->Area * this->section->density;

					// Compute initial rotation
					ChMatrix33<> A0;
					ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
					ChVector<> myele = nodes[0]->GetX0().GetA()->Get_A_Yaxis();
					A0.Set_A_Xdir(mXele, myele);
					q_element_ref_rot = A0.Get_A_quaternion(); 

					// Compute local stiffness matrix:
					ComputeStiffnessMatrix();
				}


				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 12) && (H.GetColumns()==12));
					assert (!section.IsNull());

					// Corotational K stiffness:
					ChMatrixDynamic<> CK(12,12);
					ChMatrixDynamic<> CKCt(12,12); // the global, corotated, K matrix
					
					ChMatrix33<> Atoabs(this->q_element_abs_rot);
					ChMatrix33<> Atolocwel; Atolocwel.Set33Identity();
					ChMatrixCorotation<>::ComputeCK(this->StiffnessMatrix, Atoabs, Atolocwel, 4, CK);
					ChMatrixCorotation<>::ComputeKCt(CK, Atoabs, Atolocwel, 4, CKCt);


					// For K stiffness matrix and R matrix: scale by factors

					double mkrfactor = Kfactor + Rfactor * this->section->rdamping;  

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
					assert (!section.IsNull());

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
					FiR_local.MatrScale(this->section->GetBeamRaleyghDamping());

					FiK_local.MatrInc(FiR_local);


					FiK_local.MatrScale(-1.0);

						// Fi = C * Fi_local  with C block-diagonal rotations A  , for nodal forces in abs. frame 
					ChMatrix33<> Atoabs(this->q_element_abs_rot);
					ChMatrix33<> Atolocwel; Atolocwel.Set33Identity();
					ChMatrixCorotation<>::ComputeCK(FiK_local, Atoabs, Atolocwel, 4, Fi);
#ifdef BEAM_VERBOSE
GetLog() << "\nInternal forces (local): \n";
for (int c = 0; c<6; c++)  GetLog() << FiK_local(c) << "  "; 
GetLog() << "\n";
for (int c = 6; c<12; c++) GetLog() << FiK_local(c) << "  ";
GetLog() << "\n\nInternal forces (ABS) : \n";
for (int c = 0; c<6; c++)  GetLog() << Fi(c) << "  "; 
GetLog() << "\n";
for (int c = 6; c<12; c++) GetLog() << Fi(c) << "  ";
GetLog() << "\n";
#endif
				}


			//
			// Beam-specific functions
			//

				/// Gets the xyz displacement of a point on the beam line, 
				/// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are not corotated.
	virtual void EvaluateSectionDisplacement(const double eta, const ChMatrix<>& displ, ChVector<>& u_displ, ChVector<>& u_rotaz)
				{
					ChMatrixNM<double,1,12> N;

					this->ShapeFunctions(N, eta); // Evaluate shape functions

					u_displ.x = N(0)*displ(0)+N(6)*displ(6);   // x_a   x_b
					u_displ.y = N(1)*displ(1)+N(7)*displ(7)    // y_a   y_b
							   +N(5)*displ(5)+N(11)*displ(11); // Rz_a  Rz_b
					u_displ.z = N(2)*displ(2)+N(8)*displ(8)    // z_a   z_b
							   +N(4)*displ(4)+N(10)*displ(10); // Ry_a  Ry_b 

					u_rotaz.x = N(3)*displ(3)+N(9)*displ(9);   // Rx_a  Rx_b
					
					double dN_ua = (1./(2.*this->GetRestLength()))*(-3. +3*eta*eta);  // slope shape functions are computed here on-the-fly
					double dN_ub = (1./(2.*this->GetRestLength()))*( 3. -3*eta*eta);
					double dN_ra =  (1./4.)*(-1. -2*eta + 3*eta*eta);
					double dN_rb = -(1./4.)*( 1. -2*eta - 3*eta*eta);
					u_rotaz.y = dN_ua*displ(2)+dN_ub*displ(8)+   // z_a   z_b
								dN_ra*displ(4)+dN_rb*displ(10);  // Ry_a  Ry_b
					u_rotaz.z = dN_ua*displ(1)+dN_ub*displ(7)+   // y_a   y_b
								dN_ra*displ(5)+dN_rb*displ(11);  // Rz_a  Rz_b    
				}
	
				/// Gets the absolute xyz position of a point on the beam line, 
				/// and the absolute rotation of section plane, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are corotated (expressed in world reference)
	virtual void EvaluateSectionFrame(const double eta, const ChMatrix<>& displ, ChVector<>& point, ChQuaternion<>& rot)
				{
					ChVector<> u_displ;
					ChVector<> u_rotaz;
					double Nx1 = (1./2.)*(1-eta);
					double Nx2 = (1./2.)*(1+eta);

					this->EvaluateSectionDisplacement(eta, displ, u_displ, u_rotaz);

					// Since   d = [Atw]' Xt - [A0w]'X0   , so we have
					//        Xt = [Atw] (d +  [A0w]'X0)

					point = this->q_element_abs_rot.Rotate 
							( u_displ +
							  this->q_element_ref_rot.RotateBack
							  (Nx1 * this->nodes[0]->GetX0().GetPos() + 
							   Nx2 * this->nodes[1]->GetX0().GetPos() ) 
							 );

					ChQuaternion<> msectionrot;
					msectionrot.Q_from_AngAxis(u_rotaz.Length(), u_rotaz.GetNormalized());
					rot = this->q_element_abs_rot % msectionrot;
				}


				/// Gets the force (traction x, shear y, shear z) and the
				/// torque (torsion on x, bending on y, on bending on z) at a section along 
				/// the beam line, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField().
				/// Results are not corotated, and are expressed in the reference system of beam.
	virtual void EvaluateSectionForceTorque(const double eta, const ChMatrix<>& displ, ChVector<>& Fforce, ChVector<>& Mtorque)
				{
					assert (!section.IsNull());

					double Jpolar = section->Izz + section->Iyy;

					ChMatrixNM<double,1,12> N;

					this->ShapeFunctions(N, eta); // Evaluate shape functions

					// shape function derivatives are computed here on-the-fly
					double dN_xa = -(1./length);
					double dN_xb =  (1./length);
					double ddN_ua = (6./(length*length))*(eta);  
					double ddN_ub =-(6./(length*length))*(eta);
					double ddN_ra = -(1./length) + ((3.0/length)*eta);
					double ddN_rb =  (1./length) + ((3.0/length)*eta);
					double dddN_ua = (12./(length*length*length));  
					double dddN_ub =-(12./(length*length*length));
					double dddN_ra =  (6.0/(length*length));
					double dddN_rb =  (6.0/(length*length));

					
					

					ChMatrixNM<double,6,1> sect_ek; // generalized strains/curvatures;

					// e_x
					sect_ek(0) = (dN_xa*displ(0)+dN_xb*displ(6));      // x_a   x_b
					// e_y
					sect_ek(2) = (dddN_ua*displ(1)+dddN_ub*displ(7)+   // y_a   y_b
								  dddN_ra*displ(5)+dddN_rb*displ(11)); // Rz_a  Rz_b 
					// e_z
					sect_ek(1) = (dddN_ua*displ(2)+dddN_ub*displ(8)+   // z_a   z_b
								  dddN_ra*displ(4)+dddN_rb*displ(10)); // Ry_a  Ry_b
					
					// k_x
					sect_ek(3) = (dN_xa*displ(3)+dN_xb*displ(9));	 // Rx_a  Rx_b
					// k_y
					sect_ek(4) = (ddN_ua*displ(2)+ddN_ub*displ(8)+   // z_a   z_b
								  ddN_ra*displ(4)+ddN_rb*displ(10));  // Ry_a  Ry_b
					// k_z 
					sect_ek(5) = (ddN_ua*displ(1)+ddN_ub*displ(7)+   // y_a   y_b
								  ddN_ra*displ(5)+ddN_rb*displ(11));  // Rz_a  Rz_b 

					Fforce.x = this->section->E * this->section->Area* sect_ek(0);
					Fforce.y = this->section->E * this->section->Iyy * sect_ek(1);
					Fforce.z = this->section->E * this->section->Izz * sect_ek(2);		
								 
					Mtorque.x = this->section->G * Jpolar			  * sect_ek(3);
					Mtorque.y = this->section->E * this->section->Iyy * sect_ek(4);	
					Mtorque.z = this->section->E * this->section->Izz * sect_ek(5);
					
					///***TO DO*** case of displaced shear center or centroid or rotated section axes
				}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






