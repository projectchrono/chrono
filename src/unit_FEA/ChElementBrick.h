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

#ifndef CHELEMENTBRICK_H   //brick with EAS
#define CHELEMENTBRICK_H

//#define BEAM_VERBOSE


#include "ChElementGeneric.h"
#include "physics\ChContinuumMaterial.h"
#include "ChNodeFEAxyz.h"
#include "core/ChQuadrature.h"



namespace chrono
{
namespace fea
{



/// Simple beam element with two nodes and ANCF gradient-deficient
/// formulation.
/// For this 'basic' implementation, constant section and 
/// constant material are assumed along the beam coordinate.
/// Torsional stiffness is impossible because of the formulation.
/// Based on the formulation in:
///  "Analysis of Thin Beams and Cables Using the Absolute Nodal Co-ordinate Formulation"
///  J.GERSTMAYR, A.SHABANA
///  Nonlinear Dynamics (2006) 45: 109–130
///  DOI: 10.1007/s11071-006-1856-1 
/// and in:  
/// "On the Validation and Applications of a Parallel Flexible Multi-body 
///  Dynamics Implementation"
///  D. MELANZ

class  ChElementBrick : public ChElementGeneric
{
protected:
	std::vector< ChSharedPtr<ChNodeFEAxyz> > nodes;
	
	double thickness;
	ChSharedPtr<ChContinuumElastic> Material;

	ChMatrixNM<double,24,24> StiffnessMatrix; // stiffness matrix
	ChMatrixNM<double,24,24> MassMatrix;	   // mass matrix
	ChMatrixNM<double,3,1> InertFlexVec; //for element size (EL,EW,EH)
	//EAS
	int elementnumber;
    ChMatrixNM<double,24,24> stock_jac_EAS; // EAS per elmeent
	ChMatrixNM<double,9,1> stock_alpha_EAS; // EAS per element
	ChMatrixNM<double, 24,24> stock_KTE;
	ChMatrixNM<double,24,1> initialpos; // Initial Coordinate per element
	int flag_HE;
	
	//EAS
public:

	ChElementBrick()
				{
					nodes.resize(8);

					//this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());
					//this->MassMatrix.Resize(this->GetNdofs(), this->GetNdofs());

				}

	virtual ~ChElementBrick() {}

	virtual int GetNnodes()  {return 8;}
	virtual int GetNcoords() {return 8*3;}
	virtual int GetNdofs()   {return 8*3;}

	virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes( ChSharedPtr<ChNodeFEAxyz> nodeA, ChSharedPtr<ChNodeFEAxyz> nodeB, ChSharedPtr<ChNodeFEAxyz> nodeC, ChSharedPtr<ChNodeFEAxyz> nodeD,ChSharedPtr<ChNodeFEAxyz> nodeE,ChSharedPtr<ChNodeFEAxyz> nodeF,ChSharedPtr<ChNodeFEAxyz> nodeG,ChSharedPtr<ChNodeFEAxyz> nodeH) 
				{
					assert(!nodeA.IsNull());
					assert(!nodeB.IsNull());
					assert(!nodeC.IsNull());
					assert(!nodeD.IsNull());
                    assert(!nodeE.IsNull());
					assert(!nodeF.IsNull());
					assert(!nodeG.IsNull());
					assert(!nodeH.IsNull());

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
				//EAS
					// Initial position 
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> pD = this->nodes[3]->GetPos();
					ChVector<> pE = this->nodes[4]->GetPos();
					ChVector<> pF = this->nodes[5]->GetPos();
					ChVector<> pG = this->nodes[6]->GetPos();
					ChVector<> pH = this->nodes[7]->GetPos();
					initialpos(0,0)=pA(0); initialpos(1,0)=pA(1); initialpos(2,0)=pA(2);
					initialpos(3,0)=pB(0); initialpos(4,0)=pB(1); initialpos(5,0)=pB(2);
					initialpos(6,0)=pC(0); initialpos(7,0)=pC(1); initialpos(8,0)=pC(2);
					initialpos(9,0)=pD(0); initialpos(10,0)=pD(1); initialpos(11,0)=pD(2);
					initialpos(12,0)=pE(0); initialpos(13,0)=pE(1); initialpos(14,0)=pE(2);
					initialpos(15,0)=pF(0); initialpos(16,0)=pF(1); initialpos(17,0)=pF(2);
					initialpos(18,0)=pG(0); initialpos(19,0)=pG(1); initialpos(20,0)=pG(2);
					initialpos(21,0)=pH(0); initialpos(22,0)=pH(1); initialpos(23,0)=pH(2);
				//EAS
				}

				

			//
			// FEM functions
			//

				/// Set the section & material of beam element .
				/// It is a shared property, so it can be shared between other beams.
	//void   SetThickness( double th) { thickness = th;}
				/// Get the section & material of the element
	//double GetThickness() {return thickness;}
	//EAS
	void   SetElemNum(int kb){ elementnumber = kb;}         //// 2015/5/23 for EAS

	void   SetStockAlpha(double a1,double a2,double a3,double a4,double a5,double a6,double a7,double a8,double a9){ stock_alpha_EAS(0,0)=a1;  //// 2015/5/23  only for 10by1 bench mark
																			 stock_alpha_EAS(1,0)=a2;
																			 stock_alpha_EAS(2,0)=a3;
																			 stock_alpha_EAS(3,0)=a4;
																			 stock_alpha_EAS(4,0)=a5;
	                                                                         stock_alpha_EAS(5,0)=a6;
	                                                                         stock_alpha_EAS(6,0)=a7;
	                                                                         stock_alpha_EAS(7,0)=a8;
	                                                                         stock_alpha_EAS(8,0)=a9;}

	void   SetStockJac(ChMatrixNM<double,24,24> a){stock_jac_EAS=a;	} //// 2015/5/23  for EAS

	void   SetStockKTE(ChMatrixNM<double,24,24> a){stock_KTE=a;	} //// 2015/5/23  for EAS

	void   SetInertFlexVec(ChMatrixNM<double,3,1> a){InertFlexVec = a;	} // for read element size

	int GetElemNum() {return elementnumber;}                //// 2015/5/23  for EAS

	ChMatrixNM<double,9,1> GetStockAlpha() {return stock_alpha_EAS;} //// 2015/5/23  for EAS

	ChMatrixNM<double,24,24> GetStockJac() { return stock_jac_EAS;} //// 2015/5/23  for EAS

	ChMatrixNM<double,24,24> GetStockKTE() { return stock_KTE;}

	ChMatrixNM<double,24,1> GetInitialPos() {return initialpos;} //// 2015/5/23  for Initial position
	//EAS

				/// Get the first node (beginning) 
	ChSharedPtr<ChNodeFEAxyz> GetNodeA() {return nodes[0];}

				/// Get the second node (ending)
	ChSharedPtr<ChNodeFEAxyz> GetNodeB() {return nodes[1];}

	ChSharedPtr<ChNodeFEAxyz> GetNodeC() {return nodes[2];}

	ChSharedPtr<ChNodeFEAxyz> GetNodeD() {return nodes[3];}

    ChSharedPtr<ChNodeFEAxyz> GetNodeE() {return nodes[4];}

	ChSharedPtr<ChNodeFEAxyz> GetNodeF() {return nodes[5];}

	ChSharedPtr<ChNodeFEAxyz> GetNodeG() {return nodes[6];}

    ChSharedPtr<ChNodeFEAxyz> GetNodeH() {return nodes[7];}

	//double GetLengthX() {return nodes[1]->GetX0().x - nodes[0]->GetX0().x;}
	//double GetLengthY() {return nodes[2]->GetX0().y - nodes[0]->GetX0().y;}
	//double GetLengthZ() {return nodes[4]->GetX0().z - nodes[0]->GetX0().z;}

	double GetLengthX() {return InertFlexVec(0);} 
	double GetLengthY() {return InertFlexVec(1);}
	double GetLengthZ() {return InertFlexVec(2);}

	void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
    ChSharedPtr<ChContinuumElastic> GetMaterial() { return Material; }

	
	

				/// Fills the N shape function matrix with the
				/// values of shape functions at abscyssa 'xi'.
				/// Note, xi=0 at node1, xi=+1 at node2.
				/// NOTE! actually N should be a 3row, 12 column sparse matrix,
				/// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)]; ,
				/// but to avoid wasting zero and repeated elements, here
				/// it stores only the s1 s2 s3 s4 values in a 1 row, 4 columns matrix!
	virtual void ShapeFunctions(ChMatrix<>& N, double x, double y, double z)
				{
					//double a = this->GetLengthX();
					//double b = this->GetLengthY();
					//double c = this->GetThickness();

					N(0) = 0.125*(1.0-x)*(1.0-y)*(1.0-z);
					N(1) = 0.125*(1.0+x)*(1.0-y)*(1.0-z);
					N(2) = 0.125*(1.0-x)*(1.0+y)*(1.0-z);
					N(3) = 0.125*(1.0+x)*(1.0+y)*(1.0-z);
					N(4) = 0.125*(1.0-x)*(1.0-y)*(1.0+z);
					N(5) = 0.125*(1.0+x)*(1.0-y)*(1.0+z);
					N(6) = 0.125*(1.0-x)*(1.0+y)*(1.0+z);
					N(7) = 0.125*(1.0+x)*(1.0+y)*(1.0+z);
				};

				/// Fills the N shape function derivative matrix with the
				/// values of shape function derivatives at abscyssa 'xi'.
				/// Note, xi=0 at node1, xi=+1 at node2.
				/// NOTE! to avoid wasting zero and repeated elements, here
				/// it stores only the four values in a 1 row, 4 columns matrix!
	virtual void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z)
				{
					double a = this->GetLengthX();
					//double b = this->GetLengthY();
					//double c = this->GetLengthZ();

					//double a = GetLengthX();
					//double b = GetLengthY();
					//double c = GetThickness();
					//
					//GetLog()<<a<<"\n"<<b<<"\n"<<c<<"\n";


					Nx(0) = 2.0/a*0.125 * (-1.0) * (1.0-y)*(1.0-z);
					Nx(1) = 2.0/a*0.125 * (1.0) * (1.0-y)*(1.0-z);
					Nx(2) = 2.0/a*0.125 * (-1.0) * (1.0+y)*(1.0-z);
					Nx(3) = 2.0/a*0.125 * (1.0) * (1.0+y)*(1.0-z);
					Nx(4) = 2.0/a*0.125 * (-1.0) * (1.0-y)*(1.0+z);
					Nx(5) = 2.0/a*0.125 * (1.0) * (1.0-y)*(1.0+z);
					Nx(6) = 2.0/a*0.125 * (-1.0) * (1.0+y)*(1.0+z);
					Nx(7) = 2.0/a*0.125 * (1.0) * (1.0+y)*(1.0+z);

				};

	virtual void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z)
				{
					//double a = this->GetLengthX();
					double b = this->GetLengthY();
					//double c = this->GetThickness();


					Ny(0) = 2.0/b*0.125 * (1.0-x)*(-1.0)*(1.0-z);
					Ny(1) = 2.0/b*0.125 * (1.0+x)*(-1.0)*(1.0-z);
					Ny(2) = 2.0/b*0.125 * (1.0-x)*(1.0)*(1.0-z);
					Ny(3) = 2.0/b*0.125 * (1.0+x)*(1.0)*(1.0-z);
					Ny(4) = 2.0/b*0.125 * (1.0-x)*(-1.0)*(1.0+z);
					Ny(5) = 2.0/b*0.125 * (1.0+x)*(-1.0)*(1.0+z);
					Ny(6) = 2.0/b*0.125 * (1.0-x)*(1.0)*(1.0+z);
					Ny(7) = 2.0/b*0.125 * (1.0+x)*(1.0)*(1.0+z);

				};

	virtual void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z)
				{
					//double a = this->GetLengthX();
					//double b = this->GetLengthY();
					double c = this->GetLengthZ();

					Nz(0) = 2.0/c*0.125 * (1.0-x)*(1.0-y)*(-1.0);
					Nz(1) = 2.0/c*0.125 * (1.0+x)*(1.0-y)*(-1.0);
					Nz(2) = 2.0/c*0.125 * (1.0-x)*(1.0+y)*(-1.0);
					Nz(3) = 2.0/c*0.125 * (1.0+x)*(1.0+y)*(-1.0);
					Nz(4) = 2.0/c*0.125 * (1.0-x)*(1.0-y)*(1.0);
					Nz(5) = 2.0/c*0.125 * (1.0+x)*(1.0-y)*(1.0);
					Nz(6) = 2.0/c*0.125 * (1.0-x)*(1.0+y)*(1.0);
					Nz(7) = 2.0/c*0.125 * (1.0+x)*(1.0+y)*(1.0);
				};



	virtual void Update() 
				{
					// parent class update:
					ChElementGeneric::Update();

				};

				/// Fills the D vector (column matrix) with the current 
				/// field values at the nodes of the element, with proper ordering.
				/// If the D vector has not the size of this->GetNdofs(), it will be resized.
				///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
	virtual void GetField(ChMatrixDynamic<>& mD)
				{
					mD.Reset(24,1);

					mD.PasteVector(this->nodes[0]->GetPos(),0,0);
					mD.PasteVector(this->nodes[1]->GetPos(),3,0);
					mD.PasteVector(this->nodes[2]->GetPos(),6,0);
					mD.PasteVector(this->nodes[3]->GetPos(),9,0);
					mD.PasteVector(this->nodes[4]->GetPos(),12,0);
					mD.PasteVector(this->nodes[5]->GetPos(),15,0);
					mD.PasteVector(this->nodes[6]->GetPos(),18,0);
					mD.PasteVector(this->nodes[7]->GetPos(),21,0);
				}

				
				/// Computes the STIFFNESS MATRIX of the element:    
				/// K = integral( .... ),
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed 
	virtual void ComputeStiffnessMatrix()
				{	
					
					bool use_numerical_differentiation = false;

					// Option: compute the stiffness matrix by doing a numerical differentiation
					// of the internal forces. This fixes a problem with the code by D.Melanz, that 
					// produces a rank deficient matrix for straight beams.
					if (use_numerical_differentiation)
					{
						double diff = 1e-8;
						ChMatrixDynamic<> Kcolumn(24,1);
						ChMatrixDynamic<> F0(24,1);
						ChMatrixDynamic<> F1(24,1);
						//EAS
						flag_HE=0; // flag_HE is defineded in  [class  ChElementBrick : public ChElementGeneric]
						//EAS
						this->ComputeInternalForces(F0);
						
						// the rest could be in a for loop, if implementing a  ComputeInternalForces that
						// accepts the DOFs values as a 12-vector state where one increments a value at a time,
						// but ComputeInternalForces avoids moving that data by using nodes[0]->pos.blabla directly, 
						// so here we do a sort of 'unrolled' loop:

						//EAS
						flag_HE=1; // flag_HE is defineded in  [class  ChElementBrick : public ChElementGeneric]
						//EAS
						for (int inode = 0; inode <8; ++inode)
						{
							
							this->nodes[inode]->pos.x +=diff;
							this->ComputeInternalForces(F1); // Flag=1 > Jacobian of internal force calculation
							Kcolumn = (F0-F1)*(1.0/diff);
							this->StiffnessMatrix.PasteClippedMatrix(&Kcolumn,0,0,24,1,0, 0+inode*3);
							this->nodes[inode]->pos.x -=diff;

							//for(int i=0;i<24;i++)
							//{
							//	GetLog() << F0(i) << "\n";
							//}
							//system("pause");
							//for(int i=0;i<24;i++)
							//{
							//	GetLog() << F1(i) << "\n";
							//}
							//system("pause");

							this->nodes[inode]->pos.y +=diff;
							this->ComputeInternalForces(F1);
							Kcolumn = (F0-F1)*(1.0/diff);
							this->StiffnessMatrix.PasteClippedMatrix(&Kcolumn,0,0,24,1,0, 1+inode*3);
							this->nodes[inode]->pos.y -=diff;

							this->nodes[inode]->pos.z +=diff;
							this->ComputeInternalForces(F1);
							Kcolumn = (F0-F1)*(1.0/diff);
							this->StiffnessMatrix.PasteClippedMatrix(&Kcolumn,0,0,24,1,0, 2+inode*3);
							this->nodes[inode]->pos.z -=diff;
						}
						//for(int k=0;k<24;k++)
						//{
						//	for(int i=0;i<24;i++)
						//	{
						//		GetLog() << StiffnessMatrix(k,i) << "\n";
						//	}
						//	system("pause");
						//}
						//system("pause");

						//EAS
						flag_HE=0; // flag_HE=0 is default
						ChMatrixNM<double, 24,24>  stock_jac_EAS_elem;
						stock_jac_EAS_elem=this->GetStockJac();
						StiffnessMatrix -= stock_jac_EAS_elem; //For Enhanced Assumed Strain
						//GetLog() << "stock_jac" << stock_jac_EAS_elem;
						//system("pause");


					    //GetLog() << this->StiffnessMatrix;
					    //ChStreamOutAsciiFile mdump("dump_stiff.txt");
					    //this->StiffnessMatrix.StreamOUTdenseMatlabFormat(mdump);
					    //system("pause");
					}else{
						flag_HE=0;
						///  Recover stored Jacobian
						ChMatrixNM<double, 24,24>  stock_KTE_elem;
						stock_KTE_elem=this->GetStockKTE();
						StiffnessMatrix = stock_KTE_elem;

						/// Recover stored EAS Jacobian
						ChMatrixNM<double, 24,24>  stock_jac_EAS_elem;
						stock_jac_EAS_elem=this->GetStockJac();

						StiffnessMatrix -= stock_jac_EAS_elem;
					    }
				}

				/// Computes the MASS MATRIX of the element
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed 
	virtual void ComputeMassMatrix()
				{	
					
					double rho		= Material->Get_density();

					ChMatrixNM<double, 24,1> InitialCoord;
					ChMatrixNM<double, 8,3>   d0;
					InitialCoord=this->GetInitialPos();
					d0(0,0) = InitialCoord(0,0);	d0(0,1) = InitialCoord(1,0);	d0(0,2) = InitialCoord(2,0);
					d0(1,0) = InitialCoord(3,0);	d0(1,1) = InitialCoord(4,0);	d0(1,2) = InitialCoord(5,0);
					d0(2,0) = InitialCoord(6,0);	d0(2,1) = InitialCoord(7,0);	d0(2,2) = InitialCoord(8,0);
					d0(3,0) = InitialCoord(9,0);	d0(3,1) = InitialCoord(10,0);	d0(3,2) = InitialCoord(11,0);
					d0(4,0) = InitialCoord(12,0);	d0(4,1) = InitialCoord(13,0);	d0(4,2) = InitialCoord(14,0);
					d0(5,0) = InitialCoord(15,0);	d0(5,1) = InitialCoord(16,0);	d0(5,2) = InitialCoord(17,0);
					d0(6,0) = InitialCoord(18,0);	d0(6,1) = InitialCoord(19,0);	d0(6,2) = InitialCoord(20,0);
					d0(7,0) = InitialCoord(21,0);	d0(7,1) = InitialCoord(22,0);	d0(7,2) = InitialCoord(23,0);

					/// Integrate  Area*rho*(S'*S)
					/// where S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]

					class MyMass : public ChIntegrable3D< ChMatrixNM<double,24,24> >
					{
					public:
						ChElementBrick* element;
						ChMatrixNM<double, 8,3> *d0;
						ChMatrixNM<double, 3,24> S;
						ChMatrixNM<double, 3,24> Sx;
						ChMatrixNM<double, 3,24> Sy;
						ChMatrixNM<double, 3,24> Sz;
						ChMatrixNM<double, 1,8> N;
						ChMatrixNM<double, 1,8> Nx;
						ChMatrixNM<double, 1,8> Ny;
						ChMatrixNM<double, 1,8> Nz;

						/// Evaluate the S'*S  at point x 
						virtual void Evaluate(ChMatrixNM<double,24,24>& result, const double x, const double y, const double z)
						{							
							element->ShapeFunctions(N, x, y, z);
							element->ShapeFunctionsDerivativeX(Nx, x, y, z);
							element->ShapeFunctionsDerivativeY(Ny, x, y, z);
							element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

							// S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]
							ChMatrix33<> Si;
							Si.FillDiag(N(0));
							S.PasteMatrix(&Si, 0,0);
							Si.FillDiag(N(1));
							S.PasteMatrix(&Si, 0,3);
							Si.FillDiag(N(2));
							S.PasteMatrix(&Si, 0,6);
							Si.FillDiag(N(3));
							S.PasteMatrix(&Si, 0,9);
							Si.FillDiag(N(4));
							S.PasteMatrix(&Si, 0,12);
							Si.FillDiag(N(5));
							S.PasteMatrix(&Si, 0,15);
							Si.FillDiag(N(6));
							S.PasteMatrix(&Si, 0,18);
							Si.FillDiag(N(7));
							S.PasteMatrix(&Si, 0,21);

							//ChVector<> pA = this->element->GetNodeA()->GetX0();
							//ChVector<> pB = this->element->GetNodeB()->GetX0();
							//ChVector<> pC = this->element->GetNodeC()->GetX0();
							//ChVector<> pD = this->element->GetNodeD()->GetX0();
						 //   ChVector<> pE = this->element->GetNodeE()->GetX0();
							//ChVector<> pF = this->element->GetNodeF()->GetX0();
							//ChVector<> pG = this->element->GetNodeG()->GetX0();
							//ChVector<> pH = this->element->GetNodeH()->GetX0();

							//ChMatrixNM<double, 8,3>   d;
							//d(0,0) = pA.x;	d(0,1) = pA.y;	d(0,2) = pA.z;
							//d(1,0) = pB.x;	d(1,1) = pB.y;	d(1,2) = pB.z;
							//d(2,0) = pC.x;	d(2,1) = pC.y;	d(2,2) = pC.z;
							//d(3,0) = pD.x;	d(3,1) = pD.y;	d(3,2) = pD.z;
							//d(4,0) = pE.x;	d(4,1) = pE.y;	d(4,2) = pE.z;
							//d(5,0) = pF.x;	d(5,1) = pF.y;	d(5,2) = pF.z;
							//d(6,0) = pG.x;	d(6,1) = pG.y;	d(6,2) = pG.z;
							//d(7,0) = pH.x;	d(7,1) = pH.y;	d(7,2) = pH.z;

							ChMatrixNM<double, 1,3> Nx_d0;
							Nx_d0.MatrMultiply(Nx,*d0);

							ChMatrixNM<double, 1,3> Ny_d0;
							Ny_d0.MatrMultiply(Ny,*d0);

							ChMatrixNM<double, 1,3> Nz_d0;
							Nz_d0.MatrMultiply(Nz,*d0);

							ChMatrixNM<double, 3,3> rd0;
							rd0(0,0) = Nx_d0(0,0); rd0(1,0) = Nx_d0(0,1); rd0(2,0) = Nx_d0(0,2);
							rd0(0,1) = Ny_d0(0,0); rd0(1,1) = Ny_d0(0,1); rd0(2,1) = Ny_d0(0,2);
							rd0(0,2) = Nz_d0(0,0); rd0(1,2) = Nz_d0(0,1); rd0(2,2) = Nz_d0(0,2);

							double detJ0 = rd0.Det();//if not flat,it's not 1

							// perform  r = S'*S
							result.MatrTMultiply(S,S); 

							result *= detJ0*(element->GetLengthX()/2)*(element->GetLengthY()/2)*(element->GetLengthZ()/2);
						}
					};

					MyMass myformula;
					myformula.d0 = &d0;
					myformula.element = this;

					ChQuadrature::Integrate3D< ChMatrixNM<double,24,24> >(
									this->MassMatrix,	// result of integration will go there
									myformula,			// formula to integrate
									-1,					// start of x
									1,					// end of x
									-1,					// start of y
									1,					// end of y
									-1,					// start of z
									1,					// end of z
									2					// order of integration
									);
					
					this->MassMatrix *= rho;
					//check mass matrix
					//for(int i=0;i<24;i++)
					//{
					//	for(int j=0;j<24;j++)
					//	{
					//	    GetLog() << MassMatrix(i,j) <<"\n";
					//	}
					//	system("pause");
					//}
					//system("pause");
				}

				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, ex. the mass matrix in ANCF is constant

	virtual void SetupInitial() 
				{

					// Compute rest length, mass:
					//this->length = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
					//this->mass   = this->GetThickness() * this->GetLengthY() * this->GetLengthX() * this->Material->Get_density();

					// Compute mass matrix
					ComputeMassMatrix();

					//initial EAS parameters  @@@@@@@@
					stock_jac_EAS.Reset();

					// Compute stiffness matrix 
					// (this is not constant in ANCF and will be called automatically many times by ComputeKRMmatricesGlobal() 
					// when the solver will run, yet maybe nice to privide an initial nonzero value)
					ComputeStiffnessMatrix();
				}

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 24) && (H.GetColumns()==24));
					assert (!section.IsNull());
					
					// Compute global stiffness matrix:
					ComputeStiffnessMatrix();

					//
					// 1) Store  +kf*[K] +rf*[R]
					//

					// For K stiffness matrix and R matrix: scale by factors
					// because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K] 
					double kr_factor = Kfactor + Rfactor * this->Material->Get_RayleighDampingK();  

					ChMatrixDynamic<> temp(this->StiffnessMatrix);
					temp.MatrScale( kr_factor );

					// Paste scaled K stiffness matrix and R matrix in resulting H:
					H.PasteMatrix(&temp,0,0); 

					//
					// 2) Store  +mf*[M] 
					//
	
					temp = this->MassMatrix;
					temp.MatrScale( Mfactor );

					// Paste scaled M mass matrix in resulting H:
					H.PasteSumMatrix(&temp,0,0); 

				}

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 24) && (Fi.GetColumns()==1));
					//assert (!section.IsNull());

					/*double Area = section->Area;
					double E    = section->E;
					double I    = section->I;

					double l	= this->length;*/

					//EAS
					int i=this ->GetElemNum();
					//					GetLog() <<i<<"\n";
					//system("pause");
					//EAS
 
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> pD = this->nodes[3]->GetPos();
				    ChVector<> pE = this->nodes[4]->GetPos();
					ChVector<> pF = this->nodes[5]->GetPos();
					ChVector<> pG = this->nodes[6]->GetPos();
					ChVector<> pH = this->nodes[7]->GetPos();

					
					// this matrix will be used in both MyForcesAxial and MyForcesCurv integrators (Current shape)
					ChMatrixNM<double, 8,3>   d;
					d(0,0) = pA.x;	d(0,1) = pA.y;	d(0,2) = pA.z;
					d(1,0) = pB.x;	d(1,1) = pB.y;	d(1,2) = pB.z;
					d(2,0) = pC.x;	d(2,1) = pC.y;	d(2,2) = pC.z;
					d(3,0) = pD.x;	d(3,1) = pD.y;	d(3,2) = pD.z;
					d(4,0) = pE.x;	d(4,1) = pE.y;	d(4,2) = pE.z;
					d(5,0) = pF.x;	d(5,1) = pF.y;	d(5,2) = pF.z;
					d(6,0) = pG.x;	d(6,1) = pG.y;	d(6,2) = pG.z;
					d(7,0) = pH.x;	d(7,1) = pH.y;	d(7,2) = pH.z;

					/// Initial nodal coordinates
					ChMatrixNM<double, 24,1> InitialCoord;
					ChMatrixNM<double, 8,3>   d0;
					InitialCoord=this->GetInitialPos();
					d0(0,0) = InitialCoord(0,0);	d0(0,1) = InitialCoord(1,0);	d0(0,2) = InitialCoord(2,0);
					d0(1,0) = InitialCoord(3,0);	d0(1,1) = InitialCoord(4,0);	d0(1,2) = InitialCoord(5,0);
					d0(2,0) = InitialCoord(6,0);	d0(2,1) = InitialCoord(7,0);	d0(2,2) = InitialCoord(8,0);
					d0(3,0) = InitialCoord(9,0);	d0(3,1) = InitialCoord(10,0);	d0(3,2) = InitialCoord(11,0);
					d0(4,0) = InitialCoord(12,0);	d0(4,1) = InitialCoord(13,0);	d0(4,2) = InitialCoord(14,0);
					d0(5,0) = InitialCoord(15,0);	d0(5,1) = InitialCoord(16,0);	d0(5,2) = InitialCoord(17,0);
					d0(6,0) = InitialCoord(18,0);	d0(6,1) = InitialCoord(19,0);	d0(6,2) = InitialCoord(20,0);
					d0(7,0) = InitialCoord(21,0);	d0(7,1) = InitialCoord(22,0);	d0(7,2) = InitialCoord(23,0);
					
					double v = Material->Get_v();
					double E = Material->Get_E();
									
					//@@@@@@@@@this part is added by refering ChElementBeamANCF.h (Chrono 6-1) 
					
					Fi.Reset();

					// @@@@@@@@ from below, is modified from ChElementBeamANCF.h with EAS (5-27) @@@@@@@@
                    
					/// If numerical differentiation is used, only the internal force and EAS stiffness
					/// will be calculated. If the numerical differentiation is not used, the jacobian
					/// will also be calculated.
					bool use_numerical_differentiation = false;

                    /// Internal force and EAS parameters are caulculated for numerical differentiation. 
					if(use_numerical_differentiation){
						class MyForce : public ChIntegrable3D< ChMatrixNM<double,330,1> >
						{
						public:
							ChElementBrick* element;
							/// Pointers used for external values
							ChMatrixNM<double, 8,3>  *d;
							ChMatrixNM<double, 8,3>  *d0;
							ChMatrixNM<double, 6,6>  *T0;
							ChMatrixNM<double, 9,1>  *alpha_eas;
							double *detJ0C;
							double *E;
							double *v;

							ChMatrixNM<double, 24,1>  Fint;
							ChMatrixNM<double, 6,6>  E_eps;// same as D in FORTRAN
							ChMatrixNM<double, 3,24>  Sx;
							ChMatrixNM<double, 3,24>  Sy;
							ChMatrixNM<double, 3,24>  Sz;
							ChMatrixNM<double, 1,8>   Nx;
							ChMatrixNM<double, 1,8>   Ny;
							ChMatrixNM<double, 1,8>   Nz;
							ChMatrixNM<double, 6,24>  strainD;
							ChMatrixNM<double, 6,1>   strain;
							ChMatrixNM<double, 8,8>   d_d;
							ChMatrixNM<double, 8,1>   ddNx;
							ChMatrixNM<double, 8,1>   ddNy;
							ChMatrixNM<double, 8,1>   ddNz;
							ChMatrixNM<double, 1,3>  Nxd;
							ChMatrixNM<double, 1,3>  Nyd;
							ChMatrixNM<double, 1,3>  Nzd;
							ChMatrixNM<double, 1,1>   tempA;
							ChMatrixNM<double, 1,24>  tempB;
							ChMatrixNM<double, 24,6>  tempC;
							ChMatrixNM<double, 1,1>   tempA1;//for strain incase of initial curved
							ChMatrixNM<double, 8,8>   d0_d0;//for strain incase of initial curved
						    ChMatrixNM<double, 8,1>   d0d0Nx;//for strain incase of initial curved
							ChMatrixNM<double, 8,1>   d0d0Ny;//for strain incase of initial curved
							ChMatrixNM<double, 8,1>   d0d0Nz;//for strain incase of initial curved
							double detJ0;
							//EAS
							ChMatrixNM<double, 6,9>   M;
							ChMatrixNM<double, 6,9>   G;
							ChMatrixNM<double, 9,6>   GT;
							ChMatrixNM<double, 6,1> strain_EAS;

							/// Gaussian integration to calculate internal forces and EAS matrices  
							virtual void Evaluate(ChMatrixNM<double,330,1>& result, const double x, const double y, const double z)
							{

								element->ShapeFunctionsDerivativeX(Nx, x, y, z);
								element->ShapeFunctionsDerivativeY(Ny, x, y, z);
								element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
							
								element->Basis_M(M,x,y,z); // EAS

								int flag_Mooney=1; // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material

								if(flag_Mooney==0){                                        // 0 means use linear material
									double DD = (*E)*(1.0-(*v))/((1.0+(*v))*(1.0-2.0*(*v)));
									E_eps.FillDiag(1.0);
									E_eps(0,1) = (*v)/(1.0-(*v));
									E_eps(0,3) = (*v)/(1.0-(*v));
									E_eps(1,0) = (*v)/(1.0-(*v)); 
									E_eps(1,3) = (*v)/(1.0-(*v));
									E_eps(2,2) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps(3,0) = (*v)/(1.0-(*v));
									E_eps(3,1) = (*v)/(1.0-(*v));                              
									E_eps(4,4) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps(5,5) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps *= DD;
								}							
								// Expand shape functions Sx, Sy, Sz 
								// Sx=[Nx1*eye(3) Nx2*eye(3) Nx3*eye(3) Nx4*eye(3) Nx5*eye(3) Nx6*eye(3) Nx7*eye(3) Nx8*eye(3)]
								ChMatrix33<> Sxi;
								Sxi.FillDiag(Nx(0));
								Sx.PasteMatrix(&Sxi, 0,0);
								Sxi.FillDiag(Nx(1));
								Sx.PasteMatrix(&Sxi, 0,3);
								Sxi.FillDiag(Nx(2)); 
								Sx.PasteMatrix(&Sxi, 0,6);
								Sxi.FillDiag(Nx(3));
								Sx.PasteMatrix(&Sxi, 0,9);
								Sxi.FillDiag(Nx(4));
								Sx.PasteMatrix(&Sxi, 0,12);
								Sxi.FillDiag(Nx(5));
								Sx.PasteMatrix(&Sxi, 0,15);
								Sxi.FillDiag(Nx(6));
								Sx.PasteMatrix(&Sxi, 0,18);
								Sxi.FillDiag(Nx(7));
								Sx.PasteMatrix(&Sxi, 0,21);

								ChMatrix33<> Syi;
								Syi.FillDiag(Ny(0));
								Sy.PasteMatrix(&Syi, 0,0);
								Syi.FillDiag(Ny(1));
								Sy.PasteMatrix(&Syi, 0,3);
								Syi.FillDiag(Ny(2)); 
								Sy.PasteMatrix(&Syi, 0,6);
								Syi.FillDiag(Ny(3));
								Sy.PasteMatrix(&Syi, 0,9);
								Syi.FillDiag(Ny(4));
								Sy.PasteMatrix(&Syi, 0,12);
								Syi.FillDiag(Ny(5));
								Sy.PasteMatrix(&Syi, 0,15);
								Syi.FillDiag(Ny(6));
								Sy.PasteMatrix(&Syi, 0,18);
								Syi.FillDiag(Ny(7));
								Sy.PasteMatrix(&Syi, 0,21);

								ChMatrix33<> Szi;
								Szi.FillDiag(Nz(0));
								Sz.PasteMatrix(&Szi, 0,0);
								Szi.FillDiag(Nz(1));
								Sz.PasteMatrix(&Szi, 0,3);
								Szi.FillDiag(Nz(2)); 
								Sz.PasteMatrix(&Szi, 0,6);
								Szi.FillDiag(Nz(3));
								Sz.PasteMatrix(&Szi, 0,9);
								Szi.FillDiag(Nz(4));
								Sz.PasteMatrix(&Szi, 0,12);
								Szi.FillDiag(Nz(5));
								Sz.PasteMatrix(&Szi, 0,15);
								Szi.FillDiag(Nz(6));
								Sz.PasteMatrix(&Szi, 0,18);
								Szi.FillDiag(Nz(7));
								Sz.PasteMatrix(&Szi, 0,21);

								//==EAS and Initial Shape==//					
								ChMatrixNM<double, 3,3>   rd0;
								ChMatrixNM<double, 3,3>   temp33;
								temp33.Reset();
								temp33=(Nx*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,0);
								temp33=(Ny*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,1);
								temp33=(Nz*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,2);
								detJ0=rd0.Det();

								//////////////////////////////////////////////////////////////
								//// Transformation : Orthogonal transformation (A and J) ////
								//////////////////////////////////////////////////////////////
								ChVector<double> G1;
								ChVector<double> G2;
								ChVector<double> G3;
								ChVector<double> G1xG2;
								double G1norm;
								double G1dotG1;
								G1(0)=rd0(0,0); G2(0)=rd0(0,1); G3(0)=rd0(0,2);
								G1(1)=rd0(1,0); G2(1)=rd0(1,1); G3(1)=rd0(1,2);
								G1(2)=rd0(2,0); G2(2)=rd0(2,1); G3(2)=rd0(2,2);
								G1xG2.Cross(G1,G2);
								G1dotG1=Vdot(G1,G1);

								////Tangent Frame
								ChVector<double> A1;
								ChVector<double> A2;
								ChVector<double> A3;
								A1=G1/sqrt(G1(0)*G1(0)+G1(1)*G1(1)+G1(2)*G1(2));
								A3=G1xG2/sqrt(G1xG2(0)*G1xG2(0)+G1xG2(1)*G1xG2(1)+G1xG2(2)*G1xG2(2));
								A2.Cross(A3,A1);

								////Direction for orthotropic material//
								double theta = 0.0;
								ChVector<double> AA1;
								ChVector<double> AA2;
								ChVector<double> AA3;
								AA1 = A1*cos(theta)+A2*sin(theta);
								AA2 = -A1*sin(theta)+A2*cos(theta);
								AA3 = A3;

								////Beta
								ChMatrixNM<double, 3,3> j0;
								ChVector<double> j01;
								ChVector<double> j02;
								ChVector<double> j03;
								ChMatrixNM<double, 9,1> beta;
								double temp;
								j0 = rd0;
								j0.MatrInverse();
								j01(0)=j0(0,0); j02(0)=j0(1,0); j03(0)=j0(2,0);
					            j01(1)=j0(0,1); j02(1)=j0(1,1); j03(1)=j0(2,1);
					            j01(2)=j0(0,2); j02(2)=j0(1,2); j03(2)=j0(2,2);
								temp = Vdot(AA1,j01); beta(0,0) = temp;
								temp = Vdot(AA2,j01); beta(1,0) = temp;
								temp = Vdot(AA3,j01); beta(2,0) = temp;
								temp = Vdot(AA1,j02); beta(3,0) = temp;
								temp = Vdot(AA2,j02); beta(4,0) = temp;
								temp = Vdot(AA3,j02); beta(5,0) = temp;
								temp = Vdot(AA1,j03); beta(6,0) = temp;
								temp = Vdot(AA2,j03); beta(7,0) = temp;
								temp = Vdot(AA3,j03); beta(8,0) = temp;

								//////////////////////////////////////////////////
								//// Enhanced Assumed Strain /////////////////////
								//////////////////////////////////////////////////
								G = (*T0) * M * ((*detJ0C) / (detJ0));
								strain_EAS = G*(*alpha_eas);

								//////////////////////////////////////////////////

								d_d.MatrMultiplyT(*d,*d);
								ddNx.MatrMultiplyT(d_d,Nx);
								ddNy.MatrMultiplyT(d_d,Ny);
								ddNz.MatrMultiplyT(d_d,Nz);

								d0_d0.MatrMultiplyT(*d0,*d0);
								d0d0Nx.MatrMultiplyT(d0_d0,Nx);
								d0d0Ny.MatrMultiplyT(d0_d0,Ny);
								d0d0Nz.MatrMultiplyT(d0_d0,Nz);

								///////////////////////////
								/// Strain component //////
								///////////////////////////
								ChMatrixNM<double, 6,1> strain_til;
								tempA= Nx*ddNx;
								tempA1= Nx*d0d0Nx;
								strain_til(0,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Ny*ddNy;
								tempA1=Ny*d0d0Ny;
								strain_til(1,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Nx*ddNy;
								tempA1=Nx*d0d0Ny;
								strain_til(2,0) = tempA(0,0)-tempA1(0,0);
								//== Compatible strain (No ANS) ==//
								tempA=Nz*ddNz;
								tempA1= Nz*d0d0Nz;
								strain_til(3,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Nx*ddNz;
								tempA1= Nx*d0d0Nz;
								strain_til(4,0) = tempA(0,0)-tempA1(0,0);
								tempA=Ny*ddNz;
								tempA1= Ny*d0d0Nz;
								strain_til(5,0) = tempA(0,0)-tempA1(0,0);
								//// For orthotropic material ///
								strain(0,0)=strain_til(0,0)*beta(0)*beta(0)+strain_til(1,0)*beta(3)*beta(3)+strain_til(2,0)*beta(0)*beta(3)+strain_til(3,0)*beta(6)*beta(6)+strain_til(4,0)*beta(0)*beta(6)+strain_til(5,0)*beta(3)*beta(6);
								strain(1,0)=strain_til(0,0)*beta(1)*beta(1)+strain_til(1,0)*beta(4)*beta(4)+strain_til(2,0)*beta(1)*beta(4)+strain_til(3,0)*beta(7)*beta(7)+strain_til(4,0)*beta(1)*beta(7)+strain_til(5,0)*beta(4)*beta(7);
								strain(2,0)=strain_til(0,0)*2.0*beta(0)*beta(1)+strain_til(1,0)*2.0*beta(3)*beta(4)+strain_til(2,0)*(beta(1)*beta(3)+beta(0)*beta(4))+strain_til(3,0)*2.0*beta(6)*beta(7)+strain_til(4,0)*(beta(1)*beta(6)+beta(0)*beta(7))+strain_til(5,0)*(beta(4)*beta(6)+beta(3)*beta(7));
								strain(3,0)=strain_til(0,0)*beta(2)*beta(2)+strain_til(1,0)*beta(5)*beta(5)+strain_til(2,0)*beta(2)*beta(5)+strain_til(3,0)*beta(8)*beta(8)+strain_til(4,0)*beta(2)*beta(8)+strain_til(5,0)*beta(5)*beta(8);
								strain(4,0)=strain_til(0,0)*2.0*beta(0)*beta(2)+strain_til(1,0)*2.0*beta(3)*beta(5)+strain_til(2,0)*(beta(2)*beta(3)+beta(0)*beta(5))+strain_til(3,0)*2.0*beta(6)*beta(8)+strain_til(4,0)*(beta(2)*beta(6)+beta(0)*beta(8))+strain_til(5,0)*(beta(5)*beta(6)+beta(3)*beta(8));
								strain(5,0)=strain_til(0,0)*2.0*beta(1)*beta(2)+strain_til(1,0)*2.0*beta(4)*beta(5)+strain_til(2,0)*(beta(2)*beta(4)+beta(1)*beta(5))+strain_til(3,0)*2.0*beta(7)*beta(8)+strain_til(4,0)*(beta(2)*beta(7)+beta(1)*beta(8))+strain_til(5,0)*(beta(5)*beta(7)+beta(4)*beta(8));

								////////////////////////////////////
								/// Straint derivative component ///
								////////////////////////////////////
								ChMatrixNM<double, 6,24> strainD_til;
								strainD_til.Reset();
								tempB = Nx*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,0,0);
								tempB = Ny*(*d)*Sy;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,1,0);
								tempB = Nx*(*d)*Sy + Ny*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,2,0);
								//== Compatible strain (No ANS)==//
								tempB = Nz*(*d)*Sz;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,3,0);
								tempB = Nx*(*d)*Sz + Nz*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,4,0);
								tempB = Ny*(*d)*Sz + Nz*(*d)*Sy;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,5,0);

							    //// For orthotropic material ///
								for(int ii=0;ii<24;ii++){
									strainD(0,ii)=strainD_til(0,ii)*beta(0)*beta(0)+strainD_til(1,ii)*beta(3)*beta(3)+strainD_til(2,ii)*beta(0)*beta(3)+strainD_til(3,ii)*beta(6)*beta(6)+strainD_til(4,ii)*beta(0)*beta(6)+strainD_til(5,ii)*beta(3)*beta(6);
									strainD(1,ii)=strainD_til(0,ii)*beta(1)*beta(1)+strainD_til(1,ii)*beta(4)*beta(4)+strainD_til(2,ii)*beta(1)*beta(4)+strainD_til(3,ii)*beta(7)*beta(7)+strainD_til(4,ii)*beta(1)*beta(7)+strainD_til(5,ii)*beta(4)*beta(7);
									strainD(2,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(1)+strainD_til(1,ii)*2.0*beta(3)*beta(4)+strainD_til(2,ii)*(beta(1)*beta(3)+beta(0)*beta(4))+strainD_til(3,ii)*2.0*beta(6)*beta(7)+strainD_til(4,ii)*(beta(1)*beta(6)+beta(0)*beta(7))+strainD_til(5,ii)*(beta(4)*beta(6)+beta(3)*beta(7));
									strainD(3,ii)=strainD_til(0,ii)*beta(2)*beta(2)+strainD_til(1,ii)*beta(5)*beta(5)+strainD_til(2,ii)*beta(2)*beta(5)+strainD_til(3,ii)*beta(8)*beta(8)+strainD_til(4,ii)*beta(2)*beta(8)+strainD_til(5)*beta(5)*beta(8);
									strainD(4,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(2)+strainD_til(1,ii)*2.0*beta(3)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(3)+beta(0)*beta(5))+strainD_til(3,ii)*2.0*beta(6)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(6)+beta(0)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(6)+beta(3)*beta(8));
									strainD(5,ii)=strainD_til(0,ii)*2.0*beta(1)*beta(2)+strainD_til(1,ii)*2.0*beta(4)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(4)+beta(1)*beta(5))+strainD_til(3,ii)*2.0*beta(7)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(7)+beta(1)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(7)+beta(4)*beta(8));
								}
								//for(int i=0;i<6;i++){
								//	for(int j=0; j<24; j++){
								//	GetLog() <<strainD(i,j)<<"\n";
								//	}
								//	system("pause");
								//}
								//system("pause");
								///////////////////////////////////
								/// Enhanced Assumed Strain 2nd ///
								///////////////////////////////////
								strain += strain_EAS;//same as EPS in FORTRAN

								ChMatrixNM<double, 9,6>  temp56;// same as TEMP1 in FORTRAN
								ChMatrixNM<double, 9,1> HE1;
								ChMatrixNM<double, 9,24> GDEPSP;
								ChMatrixNM<double, 9,9>  KALPHA;
	
								for(int ii=0;ii<9;ii++){
									for(int jj=0;jj<6;jj++){
										GT(ii,jj)=G(jj,ii);
									}
								}

								// 1=use Iso_Nonlinear_Mooney-Rivlin Material (2-parameters=> 3 inputs)
								if(flag_Mooney==1){
									ChMatrixNM<double, 3,3> CG;//CG: Right Cauchy-Green tensor  C=trans(F)*F
									ChMatrixNM<double, 3,3> INVCG;
									ChMatrixNM<double, 3,3> IMAT;
									ChMatrixNM<double, 3,3> I1PC;
									ChMatrixNM<double, 3,3> I2PC;
									ChMatrixNM<double, 3,3> JPC;
									ChMatrixNM<double, 3,3> STR;

									ChMatrixNM<double, 3,3> CGN;
									ChMatrixNM<double, 3,3> INVCGN;
									ChMatrixNM<double, 3,3> I1PCN;
									ChMatrixNM<double, 3,3> I2PCN;
									ChMatrixNM<double, 3,3> JPCN;
									ChMatrixNM<double, 3,3> STRN;

									ChMatrixNM<double, 6,1> strain_1;// same as EPS1 in FORTRAN
								    ChMatrixNM<double, 6,1> TEMP5;
									ChMatrixNM<double, 6,1> TEMP5N;
									TEMP5.Reset();
									TEMP5N.Reset();

									CG(0,0)=2.0*strain(0,0)+1.0;
                                    CG(1,1)=2.0*strain(1,0)+1.0;
                                    CG(2,2)=2.0*strain(3,0)+1.0;
                                    CG(1,0)=strain(2,0);
                                    CG(0,1)=CG(1,0);
                                    CG(2,0)=strain(4,0);
                                    CG(0,2)=CG(2,0);
                                    CG(2,1)=strain(5,0);
                                    CG(1,2)=CG(2,1);

									INVCG=CG;
									INVCG.MatrInverse();

									double Deld=0.000001;// same as DD in FORTRAN
									double I1 =CG(0,0)+CG(1,1)+CG(2,2);
									double I2=0.5*(pow(I1,2)-(pow(CG(0,0),2)+pow(CG(1,0),2)+pow(CG(2,0),2)+pow(CG(0,1),2)+pow(CG(1,1),2)+pow(CG(2,1),2)+pow(CG(0,2),2)+pow(CG(1,2),2)+pow(CG(2,2),2)));
									double I3=CG(0,0)*CG(1,1)*CG(2,2)-CG(0,0)*CG(1,2)*CG(2,1)+CG(0,1)*CG(1,2)*CG(2,0)-CG(0,1)*CG(1,0)*CG(2,2)+CG(0,2)*CG(1,0)*CG(2,1)-CG(2,0)*CG(1,1)*CG(0,2);
									double I1BAR=I1/(pow(I3,1.0/3.0));
									double I2BAR=I2/(pow(I3,2.0/3.0));
									double J=sqrt(I3);
									double CCOM1=551584.0; // C10   not 0.551584
									double CCOM2=137896.0; // C01   not 0.137896
									double CCOM3=2.0*(CCOM1+CCOM2)/(1.0-2.0*0.49); //K:bulk modulus
									double StockEPS;// same as StockEPS in FORTRAN

									IMAT.Reset();
                                    IMAT(0,0)=1.0;
                                    IMAT(1,1)=1.0;
                                    IMAT(2,2)=1.0;

									I1PC=(IMAT-INVCG*(1.0/3.0*I1))*pow(I3,-1.0/3.0);
									I2PC=(((IMAT*I1)-CG)-(INVCG*(2.0/3.0)*I2))*pow(I3,-2.0/3.0);
									JPC=INVCG*(J/2.0);

									STR=I1PC*(CCOM1*2.0)+I2PC*(CCOM2*2.0)+JPC*(CCOM3*(J-1.0)*2.0);

									TEMP5(0,0)=STR(0,0);TEMP5(1,0)=STR(1,1);TEMP5(2,0)=STR(0,1);
									TEMP5(3,0)=STR(2,2);TEMP5(4,0)=STR(0,2);TEMP5(5,0)=STR(1,2);

									E_eps.Reset();

									strain_1=strain;
									for(int JJJ=0;JJJ<6;JJJ++){
										StockEPS=strain_1(JJJ,0);
										strain_1(JJJ,0)=StockEPS+Deld;
										CGN(0,0)=2.0*strain_1(0,0)+1.0;
										CGN(1,1)=2.0*strain_1(1,0)+1.0;
                                        CGN(2,2)=2.0*strain_1(3,0)+1.0;
                                        CGN(1,0)=strain_1(2,0);
                                        CGN(0,1)=CGN(1,0);
                                        CGN(2,0)=strain_1(4,0);
										CGN(0,2)=CGN(2,0);
										CGN(2,1)=strain_1(5,0);
										CGN(1,2)=CGN(2,1);
										INVCGN=CGN;
									    INVCGN.MatrInverse();
										I1 =CGN(0,0)+CGN(1,1)+CGN(2,2);
										I2=0.5*(pow(I1,2)-(pow(CGN(0,0),2)+pow(CGN(1,0),2)+pow(CGN(2,0),2)+pow(CGN(0,1),2)+pow(CGN(1,1),2)+pow(CGN(2,1),2)+pow(CGN(0,2),2)+pow(CGN(1,2),2)+pow(CGN(2,2),2)));
										I3=CGN(0,0)*CGN(1,1)*CGN(2,2)-CGN(0,0)*CGN(1,2)*CGN(2,1)+CGN(0,1)*CGN(1,2)*CGN(2,0)-CGN(0,1)*CGN(1,0)*CGN(2,2)+CGN(0,2)*CGN(1,0)*CGN(2,1)-CGN(2,0)*CGN(1,1)*CGN(0,2);
										J=sqrt(I3);
										I1BAR=I1/(pow(I3,1.0/3.0));
										I2BAR=I2/(pow(I3,2.0/3.0));
										I1PCN=(IMAT-INVCGN*(1.0/3.0*I1))*pow(I3,-1.0/3.0);
										I2PCN=(((IMAT*I1)-CGN)-(INVCGN*(2.0/3.0)*I2))*pow(I3,-2.0/3.0);
										JPCN=INVCGN*(J/2.0);
										STRN=I1PCN*(CCOM1*2.0)+I2PCN*(CCOM2*2.0)+JPCN*(CCOM3*(J-1.0)*2.0);
								        TEMP5N(0,0)=STRN(0,0);TEMP5N(1,0)=STRN(1,1);TEMP5N(2,0)=STRN(0,1);
									    TEMP5N(3,0)=STRN(2,2);TEMP5N(4,0)=STRN(0,2);TEMP5N(5,0)=STRN(1,2);
										strain_1(JJJ,0)=StockEPS;
										E_eps(JJJ,0)=(TEMP5N(0,0)-TEMP5(0,0))/Deld;E_eps(JJJ,1)=(TEMP5N(1,0)-TEMP5(1,0))/Deld;E_eps(JJJ,2)=(TEMP5N(2,0)-TEMP5(2,0))/Deld;
										E_eps(JJJ,3)=(TEMP5N(3,0)-TEMP5(3,0))/Deld;E_eps(JJJ,4)=(TEMP5N(4,0)-TEMP5(4,0))/Deld;E_eps(JJJ,5)=(TEMP5N(5,0)-TEMP5(5,0))/Deld;
										//for(int i=0;i<6;i++){
										//	GetLog() <<TEMP5N(i)<<"\n";}
										//system("pause");
										//for(int i=0;i<6;i++){
										//	GetLog() <<TEMP5(i)<<"\n";}
										//system("pause");
									   }
									// check E_eps
									//for(int i=0;i<6;i++){
									//	for(int j=0; j<6; j++){
									//	GetLog() <<E_eps(i,j)<<"\n";
									//	}
									//	system("pause");
									//}
									//system("pause");
									temp56.MatrMultiply(GT,E_eps);
									Fint.MatrTMultiply(strainD,TEMP5);
									Fint *= detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
									HE1.MatrMultiply(GT,TEMP5);
									HE1*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								}else{

									temp56.MatrMultiply(GT,E_eps);
									tempC.MatrTMultiply(strainD,E_eps);
									Fint.MatrMultiply(tempC,strain);
									Fint *= detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
									HE1.MatrMultiply(temp56,strain);
									HE1*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								}//end of   if(*flag_Mooney==1)

								KALPHA.MatrMultiply(temp56,G);
								KALPHA*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								GDEPSP.MatrMultiply(temp56,strainD);
								GDEPSP*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								
								result.Reset();

								ChMatrixNM<double, 216,1> GDEPSPVec;
								ChMatrixNM<double, 81,1>  KALPHAVec;
								GDEPSPVec = GDEPSP;
								KALPHAVec = KALPHA;
								result.PasteClippedMatrix(&Fint,0,0,24,1,0,0);
								result.PasteClippedMatrix(&HE1,0,0,9,1,24,0);   
								result.PasteClippedMatrix(&GDEPSPVec,0,0,216,1,33,0); 
								result.PasteClippedMatrix(&KALPHAVec,0,0,81,1,249,0); 
							}
						};
						//////////////////////////////////////////////////////////////////////////////////////////////////////////
						ChMatrixNM<double, 330,1> TempIntegratedResult;
						ChMatrixNM<double, 24,1> Finternal;
						
						ChMatrixNM<double, 6,6>   T0;
						ChMatrixNM<double, 9,1>   HE;
						ChMatrixNM<double, 9,24>  GDEPSP;
						ChMatrixNM<double, 9,9>   KALPHA;
						ChMatrixNM<double, 9,9>   KALPHA1;
						ChMatrixNM<double, 9,1>   ResidHE;
						double detJ0C;
						ChMatrixNM<double, 9,1>   alpha_eas;
						ChMatrixNM<double, 9,1>   renewed_alpha_eas;
						ChMatrixNM<double, 9,1> previous_alpha;
					
						previous_alpha=GetStockAlpha();
						alpha_eas = previous_alpha;
						ResidHE.Reset();
						int flag_M=1; // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material
						int count=0;
						int fail=1;
						/// Begin EAS loop
						while(fail==1)
						{
							/// Update alpha EAS
							alpha_eas = alpha_eas - ResidHE;
							renewed_alpha_eas = alpha_eas;

							Finternal.Reset();
							HE.Reset();
							GDEPSP.Reset();
							KALPHA.Reset();

							// Enhanced Assumed Strain (EAS)
							T0.Reset();
							detJ0C=0.0;
							T0DetJElementCenterForEAS(d0,T0,detJ0C);
							//== F_internal ==//
							MyForce myformula;
							myformula.d = &d;
							myformula.d0 = &d0;

							if(flag_M==0){
							myformula.E = &E;
							myformula.v = &v;
							}

							myformula.element = this;
							//EAS
							myformula.T0 = &T0;
							myformula.detJ0C = &detJ0C;
							myformula.alpha_eas = &alpha_eas;
							ChQuadrature::Integrate3D< ChMatrixNM<double,330,1> >(
											TempIntegratedResult,				// result of integration will go there
											myformula,			// formula to integrate
											-1,					// start of x
											1,					// end of x
											-1,					// start of y
											1,					// end of y
											-1,					// start of z
											1,					// end of z
											2					// order of integration
											);
							///===============================================================//
							///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
							///===TempIntegratedResult(24:32,1) -> HE(9x1)   brick   =========//
							///===TempIntegratedResult(33:248,1) -> GDEPSP(9x24)     =========//
							///===TempIntegratedResult(249:329,1) -> KALPHA(9x9)     =========//
							///===============================================================//
							ChMatrixNM<double, 216,1> GDEPSPvec;
							ChMatrixNM<double, 81,1> KALPHAvec;
							Finternal.PasteClippedMatrix(&TempIntegratedResult,0,0,24,1,0,0); // 
							HE.PasteClippedMatrix(&TempIntegratedResult,24,0,9,1,0,0); // 
							GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult,33,0,216,1,0,0); //
							KALPHAvec.PasteClippedMatrix(&TempIntegratedResult,249,0,81,1,0,0); //
							GDEPSP=GDEPSPvec;
							KALPHA=KALPHAvec;
							KALPHA1=KALPHA;

							if(flag_HE==1) break; // When numerical jacobian loop, no need to calculate HE
							count = count + 1;
							double norm_HE = HE.NormTwo();

							if(norm_HE < 0.00001) 
							{fail=0;
							}else{
							ChMatrixNM<int, 9,1> INDX;
							double DAMMY = 0.0;
							ResidHE=HE;
							LUDCMP55(KALPHA1,9,9,INDX,DAMMY);
							LUBKSB55(KALPHA1,9,9,INDX,ResidHE); //Linear solver for alpha EAS convergence
							}
							if(flag_HE==0&&count>2){
							GetLog() << i << "  count " << count << "  NormHE "  << norm_HE << "\n";
							}
						}
						Fi = -Finternal;
									//for(int j=0; j<24; j++){
									//GetLog() <<Fi(j)<<"\n";
									//}
									//system("pause");
						//===============================//
						//== Stock_Alpha=================//
						//===============================//
						if(flag_HE==0){
							SetStockAlpha(renewed_alpha_eas(0,0),renewed_alpha_eas(1,0),renewed_alpha_eas(2,0),renewed_alpha_eas(3,0),renewed_alpha_eas(4,0),renewed_alpha_eas(5,0),renewed_alpha_eas(6,0),renewed_alpha_eas(7,0),renewed_alpha_eas(8,0)); //this->
						}
						//===============================//
						//== Jacobian Matrix for alpha ==//
						//===============================//
						if(flag_HE==0){
							ChMatrixNM<double, 9,9> INV_KALPHA;
							ChMatrixNM<double, 9,24> TEMP_GDEPSP;
							ChMatrixNM<double, 9,9> INV_KALPHA_Temp;
							ChMatrixNM<double, 24,24> stock_jac_EAS_elem;

							for(int ii=0;ii<9;ii++){
								double DAMMY = 0.0;
								INV_KALPHA_Temp=KALPHA;
								ChMatrixNM<int, 9,1> INDX;
								ChMatrixNM<double, 9,1> DAMMY_vec;
								DAMMY_vec.Reset();
								DAMMY_vec(ii)=1.0;
								LUDCMP55(INV_KALPHA_Temp,9,9,INDX,DAMMY);
								LUBKSB55(INV_KALPHA_Temp,9,9,INDX,DAMMY_vec);
								INV_KALPHA.PasteClippedMatrix(&DAMMY_vec,0,0,9,1,0,ii); //
							}
							TEMP_GDEPSP.MatrMultiply(INV_KALPHA,GDEPSP);
							stock_jac_EAS_elem.MatrTMultiply(GDEPSP,TEMP_GDEPSP);
							this->SetStockJac(stock_jac_EAS_elem);
						}
					}else{
						/// Internal force, EAS stiffness, and analytical jacobian are calculated
						class MyForce : public ChIntegrable3D< ChMatrixNM<double,906,1> >
						{
						public:
							ChElementBrick* element;
							ChMatrixNM<double, 8,3>  *d; //this is an external matrix, use pointer
							ChMatrixNM<double, 8,3>  *d0;
							ChMatrixNM<double, 6,6>  *T0;
							ChMatrixNM<double, 9,1>  *alpha_eas;
							double *detJ0C;
							double *E;
							double *v;
						
							ChMatrixNM<double, 24,1>  Fint;
							ChMatrixNM<double, 24,24>  JAC11;
							ChMatrixNM<double, 9,24>  Gd;
							ChMatrixNM<double, 6,1>  stress;
							ChMatrixNM<double, 9,9>  Sigm;
							ChMatrixNM<double, 24,6>  temp246;
							ChMatrixNM<double, 24,9>  temp249;
							ChMatrixNM<double, 6,6>  E_eps;
							ChMatrixNM<double, 3,24>  Sx;
							ChMatrixNM<double, 3,24>  Sy;
							ChMatrixNM<double, 3,24>  Sz;
							ChMatrixNM<double, 1,8>   Nx;
							ChMatrixNM<double, 1,8>   Ny;
							ChMatrixNM<double, 1,8>   Nz;
							ChMatrixNM<double, 6,24>  strainD;
							ChMatrixNM<double, 6,1>   strain;
							ChMatrixNM<double, 8,8>   d_d;
							ChMatrixNM<double, 8,1>   ddNx;
							ChMatrixNM<double, 8,1>   ddNy;
							ChMatrixNM<double, 8,1>   ddNz;
							ChMatrixNM<double, 1,3>  Nxd;
							ChMatrixNM<double, 1,3>  Nyd;
							ChMatrixNM<double, 1,3>  Nzd;
							ChMatrixNM<double, 1,1>   tempA;
							ChMatrixNM<double, 1,24>  tempB;
							ChMatrixNM<double, 24,6>  tempC;
						    ChMatrixNM<double, 1,1>   tempA1;//for strain incase of initial curved
							ChMatrixNM<double, 8,8>   d0_d0;//for strain incase of initial curved
						    ChMatrixNM<double, 8,1>   d0d0Nx;//for strain incase of initial curved
							ChMatrixNM<double, 8,1>   d0d0Ny;//for strain incase of initial curved
							ChMatrixNM<double, 8,1>   d0d0Nz;//for strain incase of initial curved
							double detJ0;
						//	//EAS
							ChMatrixNM<double, 6,9>   M;
							ChMatrixNM<double, 6,9>   G;
							ChMatrixNM<double, 9,6>   GT;
							ChMatrixNM<double, 6,1> strain_EAS;



						//	/// Evaluate (strainD'*strain)  at point x 
							virtual void Evaluate(ChMatrixNM<double,906,1>& result, const double x, const double y, const double z)
							{
								element->ShapeFunctionsDerivativeX(Nx, x, y, z);
								element->ShapeFunctionsDerivativeY(Ny, x, y, z);
								element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
							
								element->Basis_M(M,x,y,z); // EAS

								int flag_Mooney=1; // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material

								if(flag_Mooney==0){                                        // 0 means use linear material
									double DD = (*E)*(1.0-(*v))/((1.0+(*v))*(1.0-2.0*(*v)));
									E_eps.FillDiag(1.0);
									E_eps(0,1) = (*v)/(1.0-(*v));
									E_eps(0,3) = (*v)/(1.0-(*v));
									E_eps(1,0) = (*v)/(1.0-(*v)); 
									E_eps(1,3) = (*v)/(1.0-(*v));
									E_eps(2,2) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps(3,0) = (*v)/(1.0-(*v));
									E_eps(3,1) = (*v)/(1.0-(*v));                              
									E_eps(4,4) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps(5,5) = (1.0-2.0*(*v))/(2.0*(1.0-(*v)));
									E_eps *= DD;
								}							
						//		// Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
								ChMatrix33<> Sxi;
								Sxi.FillDiag(Nx(0));
								Sx.PasteMatrix(&Sxi, 0,0);
								Sxi.FillDiag(Nx(1));
								Sx.PasteMatrix(&Sxi, 0,3);
								Sxi.FillDiag(Nx(2)); 
								Sx.PasteMatrix(&Sxi, 0,6);
								Sxi.FillDiag(Nx(3));
								Sx.PasteMatrix(&Sxi, 0,9);
								Sxi.FillDiag(Nx(4));
								Sx.PasteMatrix(&Sxi, 0,12);
								Sxi.FillDiag(Nx(5));
								Sx.PasteMatrix(&Sxi, 0,15);
								Sxi.FillDiag(Nx(6));
								Sx.PasteMatrix(&Sxi, 0,18);
								Sxi.FillDiag(Nx(7));
								Sx.PasteMatrix(&Sxi, 0,21);

								ChMatrix33<> Syi;
								Syi.FillDiag(Ny(0));
								Sy.PasteMatrix(&Syi, 0,0);
								Syi.FillDiag(Ny(1));
								Sy.PasteMatrix(&Syi, 0,3);
								Syi.FillDiag(Ny(2)); 
								Sy.PasteMatrix(&Syi, 0,6);
								Syi.FillDiag(Ny(3));
								Sy.PasteMatrix(&Syi, 0,9);
								Syi.FillDiag(Ny(4));
								Sy.PasteMatrix(&Syi, 0,12);
								Syi.FillDiag(Ny(5));
								Sy.PasteMatrix(&Syi, 0,15);
								Syi.FillDiag(Ny(6));
								Sy.PasteMatrix(&Syi, 0,18);
								Syi.FillDiag(Ny(7));
								Sy.PasteMatrix(&Syi, 0,21);

								ChMatrix33<> Szi;
								Szi.FillDiag(Nz(0));
								Sz.PasteMatrix(&Szi, 0,0);
								Szi.FillDiag(Nz(1));
								Sz.PasteMatrix(&Szi, 0,3);
								Szi.FillDiag(Nz(2)); 
								Sz.PasteMatrix(&Szi, 0,6);
								Szi.FillDiag(Nz(3));
								Sz.PasteMatrix(&Szi, 0,9);
								Szi.FillDiag(Nz(4));
								Sz.PasteMatrix(&Szi, 0,12);
								Szi.FillDiag(Nz(5));
								Sz.PasteMatrix(&Szi, 0,15);
								Szi.FillDiag(Nz(6));
								Sz.PasteMatrix(&Szi, 0,18);
								Szi.FillDiag(Nz(7));
								Sz.PasteMatrix(&Szi, 0,21);

						//		//==EAS and Initial Shape==//					
								ChMatrixNM<double, 3,3>   rd0;
								ChMatrixNM<double, 3,3>   temp33;
								temp33.Reset();
								temp33=(Nx*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,0);
								temp33=(Ny*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,1);
								temp33=(Nz*(*d0));
								temp33.MatrTranspose();
								rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,2);
								detJ0=rd0.Det();

						//		//////////////////////////////////////////////////////////////
						//		//// Transformation : Orthogonal transformation (A and J) ////
						//		//////////////////////////////////////////////////////////////
								ChVector<double> G1;
								ChVector<double> G2;
								ChVector<double> G3;
								ChVector<double> G1xG2;
								double G1norm;
								double G1dotG1;
								G1(0)=rd0(0,0); G2(0)=rd0(0,1); G3(0)=rd0(0,2);
								G1(1)=rd0(1,0); G2(1)=rd0(1,1); G3(1)=rd0(1,2);
								G1(2)=rd0(2,0); G2(2)=rd0(2,1); G3(2)=rd0(2,2);
								G1xG2.Cross(G1,G2);
								G1dotG1=Vdot(G1,G1);

						//		////Tangent Frame
								ChVector<double> A1;
								ChVector<double> A2;
								ChVector<double> A3;
								A1=G1/sqrt(G1(0)*G1(0)+G1(1)*G1(1)+G1(2)*G1(2));
								A3=G1xG2/sqrt(G1xG2(0)*G1xG2(0)+G1xG2(1)*G1xG2(1)+G1xG2(2)*G1xG2(2));
								A2.Cross(A3,A1);
								////Direction for orthotropic material//
								double theta = 0.0;
								ChVector<double> AA1;
								ChVector<double> AA2;
								ChVector<double> AA3;
								AA1 = A1*cos(theta)+A2*sin(theta);
								AA2 = -A1*sin(theta)+A2*cos(theta);
								AA3 = A3;

								////Beta
								ChMatrixNM<double, 3,3> j0;
								ChVector<double> j01;
								ChVector<double> j02;
								ChVector<double> j03;
								ChMatrixNM<double, 9,1> beta;
								double temp;
								j0 = rd0;
								j0.MatrInverse();
								j01(0)=j0(0,0); j02(0)=j0(1,0); j03(0)=j0(2,0);
								j01(1)=j0(0,1); j02(1)=j0(1,1); j03(1)=j0(2,1);
								j01(2)=j0(0,2); j02(2)=j0(1,2); j03(2)=j0(2,2);
								temp = Vdot(AA1,j01); beta(0,0) = temp;
								temp = Vdot(AA2,j01); beta(1,0) = temp;
								temp = Vdot(AA3,j01); beta(2,0) = temp;
								temp = Vdot(AA1,j02); beta(3,0) = temp;
								temp = Vdot(AA2,j02); beta(4,0) = temp;
								temp = Vdot(AA3,j02); beta(5,0) = temp;
								temp = Vdot(AA1,j03); beta(6,0) = temp;
								temp = Vdot(AA2,j03); beta(7,0) = temp;
								temp = Vdot(AA3,j03); beta(8,0) = temp;
						//		//////////////////////////////////////////////////
						//		//// Enhanced Assumed Strain /////////////////////
						//		//////////////////////////////////////////////////
								G = (*T0) * M * ((*detJ0C) / (detJ0));
								strain_EAS = G*(*alpha_eas);
						//		//////////////////////////////////////////////////

								d_d.MatrMultiplyT(*d,*d);
								ddNx.MatrMultiplyT(d_d,Nx);
								ddNy.MatrMultiplyT(d_d,Ny);
								ddNz.MatrMultiplyT(d_d,Nz);

								d0_d0.MatrMultiplyT(*d0,*d0);
								d0d0Nx.MatrMultiplyT(d0_d0,Nx);
								d0d0Ny.MatrMultiplyT(d0_d0,Ny);
								d0d0Nz.MatrMultiplyT(d0_d0,Nz);
						//		///////////////////////////
						//		/// Strain component //////
						//		///////////////////////////
								ChMatrixNM<double, 6,1> strain_til;
								tempA= Nx*ddNx;
								tempA1= Nx*d0d0Nx;
								strain_til(0,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Ny*ddNy;
								tempA1=Ny*d0d0Ny;
								strain_til(1,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Nx*ddNy;
								tempA1=Nx*d0d0Ny;
								strain_til(2,0) = tempA(0,0)-tempA1(0,0);
								//== Compatible strain (No ANS) ==//
								tempA=Nz*ddNz;
								tempA1= Nz*d0d0Nz;
								strain_til(3,0) = 0.5*(tempA(0,0)-tempA1(0,0));
								tempA=Nx*ddNz;
								tempA1= Nx*d0d0Nz;
								strain_til(4,0) = tempA(0,0)-tempA1(0,0);
								tempA=Ny*ddNz;
								tempA1= Ny*d0d0Nz;
								strain_til(5,0) = tempA(0,0)-tempA1(0,0);
						//		//// For orthotropic material ///
								strain(0,0)=strain_til(0,0)*beta(0)*beta(0)+strain_til(1,0)*beta(3)*beta(3)+strain_til(2,0)*beta(0)*beta(3)+strain_til(3,0)*beta(6)*beta(6)+strain_til(4,0)*beta(0)*beta(6)+strain_til(5,0)*beta(3)*beta(6);
								strain(1,0)=strain_til(0,0)*beta(1)*beta(1)+strain_til(1,0)*beta(4)*beta(4)+strain_til(2,0)*beta(1)*beta(4)+strain_til(3,0)*beta(7)*beta(7)+strain_til(4,0)*beta(1)*beta(7)+strain_til(5,0)*beta(4)*beta(7);
								strain(2,0)=strain_til(0,0)*2.0*beta(0)*beta(1)+strain_til(1,0)*2.0*beta(3)*beta(4)+strain_til(2,0)*(beta(1)*beta(3)+beta(0)*beta(4))+strain_til(3,0)*2.0*beta(6)*beta(7)+strain_til(4,0)*(beta(1)*beta(6)+beta(0)*beta(7))+strain_til(5,0)*(beta(4)*beta(6)+beta(3)*beta(7));
								strain(3,0)=strain_til(0,0)*beta(2)*beta(2)+strain_til(1,0)*beta(5)*beta(5)+strain_til(2,0)*beta(2)*beta(5)+strain_til(3,0)*beta(8)*beta(8)+strain_til(4,0)*beta(2)*beta(8)+strain_til(5,0)*beta(5)*beta(8);
								strain(4,0)=strain_til(0,0)*2.0*beta(0)*beta(2)+strain_til(1,0)*2.0*beta(3)*beta(5)+strain_til(2,0)*(beta(2)*beta(3)+beta(0)*beta(5))+strain_til(3,0)*2.0*beta(6)*beta(8)+strain_til(4,0)*(beta(2)*beta(6)+beta(0)*beta(8))+strain_til(5,0)*(beta(5)*beta(6)+beta(3)*beta(8));
								strain(5,0)=strain_til(0,0)*2.0*beta(1)*beta(2)+strain_til(1,0)*2.0*beta(4)*beta(5)+strain_til(2,0)*(beta(2)*beta(4)+beta(1)*beta(5))+strain_til(3,0)*2.0*beta(7)*beta(8)+strain_til(4,0)*(beta(2)*beta(7)+beta(1)*beta(8))+strain_til(5,0)*(beta(5)*beta(7)+beta(4)*beta(8));

						//		////////////////////////////////////
						//		/// Straint derivative component ///
						//		////////////////////////////////////
								ChMatrixNM<double, 6,24> strainD_til;
								strainD_til.Reset();
								tempB = Nx*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,0,0);
								tempB = Ny*(*d)*Sy;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,1,0);
								tempB = Nx*(*d)*Sy + Ny*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,2,0);
								//== Compatible strain (No ANS)==//
								tempB = Nz*(*d)*Sz;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,3,0);
								tempB = Nx*(*d)*Sz + Nz*(*d)*Sx;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,4,0);
								tempB = Ny*(*d)*Sz + Nz*(*d)*Sy;
								strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,5,0);
						//		//// For orthotropic material ///
								for(int ii=0;ii<24;ii++){
									strainD(0,ii)=strainD_til(0,ii)*beta(0)*beta(0)+strainD_til(1,ii)*beta(3)*beta(3)+strainD_til(2,ii)*beta(0)*beta(3)+strainD_til(3,ii)*beta(6)*beta(6)+strainD_til(4,ii)*beta(0)*beta(6)+strainD_til(5,ii)*beta(3)*beta(6);
									strainD(1,ii)=strainD_til(0,ii)*beta(1)*beta(1)+strainD_til(1,ii)*beta(4)*beta(4)+strainD_til(2,ii)*beta(1)*beta(4)+strainD_til(3,ii)*beta(7)*beta(7)+strainD_til(4,ii)*beta(1)*beta(7)+strainD_til(5,ii)*beta(4)*beta(7);
									strainD(2,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(1)+strainD_til(1,ii)*2.0*beta(3)*beta(4)+strainD_til(2,ii)*(beta(1)*beta(3)+beta(0)*beta(4))+strainD_til(3,ii)*2.0*beta(6)*beta(7)+strainD_til(4,ii)*(beta(1)*beta(6)+beta(0)*beta(7))+strainD_til(5,ii)*(beta(4)*beta(6)+beta(3)*beta(7));
									strainD(3,ii)=strainD_til(0,ii)*beta(2)*beta(2)+strainD_til(1,ii)*beta(5)*beta(5)+strainD_til(2,ii)*beta(2)*beta(5)+strainD_til(3,ii)*beta(8)*beta(8)+strainD_til(4,ii)*beta(2)*beta(8)+strainD_til(5)*beta(5)*beta(8);
									strainD(4,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(2)+strainD_til(1,ii)*2.0*beta(3)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(3)+beta(0)*beta(5))+strainD_til(3,ii)*2.0*beta(6)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(6)+beta(0)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(6)+beta(3)*beta(8));
									strainD(5,ii)=strainD_til(0,ii)*2.0*beta(1)*beta(2)+strainD_til(1,ii)*2.0*beta(4)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(4)+beta(1)*beta(5))+strainD_til(3,ii)*2.0*beta(7)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(7)+beta(1)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(7)+beta(4)*beta(8));
								}
						//		/// Gd (8x24) calculation
								for(int ii=0;ii<8;ii++)
								{
									Gd(0,3*(ii)) = j0(0,0)*Nx(0,ii)+j0(1,0)*Ny(0,ii)+j0(2,0)*Nz(0,ii);
									Gd(1,3*(ii)+1) = j0(0,0)*Nx(0,ii)+j0(1,0)*Ny(0,ii)+j0(2,0)*Nz(0,ii);
									Gd(2,3*(ii)+2) = j0(0,0)*Nx(0,ii)+j0(1,0)*Ny(0,ii)+j0(2,0)*Nz(0,ii);

									Gd(3,3*(ii)) = j0(0,1)*Nx(0,ii)+j0(1,1)*Ny(0,ii)+j0(2,1)*Nz(0,ii);
									Gd(4,3*(ii)+1) = j0(0,1)*Nx(0,ii)+j0(1,1)*Ny(0,ii)+j0(2,1)*Nz(0,ii);
									Gd(5,3*(ii)+2) = j0(0,1)*Nx(0,ii)+j0(1,1)*Ny(0,ii)+j0(2,1)*Nz(0,ii);

									Gd(6,3*(ii)) = j0(0,2)*Nx(0,ii)+j0(1,2)*Ny(0,ii)+j0(2,2)*Nz(0,ii);
									Gd(7,3*(ii)+1) = j0(0,2)*Nx(0,ii)+j0(1,2)*Ny(0,ii)+j0(2,2)*Nz(0,ii);
									Gd(8,3*(ii)+2) = j0(0,2)*Nx(0,ii)+j0(1,2)*Ny(0,ii)+j0(2,2)*Nz(0,ii);
								}
						//		///////////////////////////////////
						//		/// Enhanced Assumed Strain 2nd ///
						//		///////////////////////////////////
								strain += strain_EAS;

								ChMatrixNM<double, 9,6>  temp56;
								ChMatrixNM<double, 9,1> HE1;
								ChMatrixNM<double, 9,24> GDEPSP;
								ChMatrixNM<double, 9,9>  KALPHA;
							    for(int ii=0;ii<9;ii++){
									for(int jj=0;jj<6;jj++){
										GT(ii,jj)=G(jj,ii);
									}
								}
								if(flag_Mooney==1){
									ChMatrixNM<double, 3,3> CG;//CG: Right Cauchy-Green tensor  C=trans(F)*F
									ChMatrixNM<double, 3,3> INVCG;
									ChMatrixNM<double, 3,3> IMAT;
									ChMatrixNM<double, 3,3> I1PC;
									ChMatrixNM<double, 3,3> I2PC;
									ChMatrixNM<double, 3,3> JPC;
									ChMatrixNM<double, 3,3> STR;

									ChMatrixNM<double, 3,3> CGN;
									ChMatrixNM<double, 3,3> INVCGN;
									ChMatrixNM<double, 3,3> I1PCN;
									ChMatrixNM<double, 3,3> I2PCN;
									ChMatrixNM<double, 3,3> JPCN;
									ChMatrixNM<double, 3,3> STRN;

									ChMatrixNM<double, 6,1> strain_1;// same as EPS1 in FORTRAN
								    ChMatrixNM<double, 6,1> TEMP5;
									ChMatrixNM<double, 6,1> TEMP5N;
									TEMP5.Reset();
									TEMP5N.Reset();

									CG(0,0)=2.0*strain(0,0)+1.0;
                                    CG(1,1)=2.0*strain(1,0)+1.0;
                                    CG(2,2)=2.0*strain(3,0)+1.0;
                                    CG(1,0)=strain(2,0);
                                    CG(0,1)=CG(1,0);
                                    CG(2,0)=strain(4,0);
                                    CG(0,2)=CG(2,0);
                                    CG(2,1)=strain(5,0);
                                    CG(1,2)=CG(2,1);

									INVCG=CG;
									INVCG.MatrInverse();

									//GetLog() << " INVCG"<<INVCG<<"\n";

									double Deld=0.000001;// same as DD in FORTRAN
									double I1 =CG(0,0)+CG(1,1)+CG(2,2);
									double I2=0.5*(pow(I1,2)-(pow(CG(0,0),2)+pow(CG(1,0),2)+pow(CG(2,0),2)+pow(CG(0,1),2)+pow(CG(1,1),2)+pow(CG(2,1),2)+pow(CG(0,2),2)+pow(CG(1,2),2)+pow(CG(2,2),2)));
									double I3=CG(0,0)*CG(1,1)*CG(2,2)-CG(0,0)*CG(1,2)*CG(2,1)+CG(0,1)*CG(1,2)*CG(2,0)-CG(0,1)*CG(1,0)*CG(2,2)+CG(0,2)*CG(1,0)*CG(2,1)-CG(2,0)*CG(1,1)*CG(0,2);
									double I1BAR=I1/(pow(I3,1.0/3.0));
									double I2BAR=I2/(pow(I3,2.0/3.0));
									double J=sqrt(I3);
									double CCOM1=551584.0; // C10   not 0.551584
									double CCOM2=137896.0; // C01   not 0.137896
									double CCOM3=2.0*(CCOM1+CCOM2)/(1.0-2.0*0.49); //K:bulk modulus
									double StockEPS;// same as StockEPS in FORTRAN

									IMAT.Reset();
                                    IMAT(0,0)=1.0;
                                    IMAT(1,1)=1.0;
                                    IMAT(2,2)=1.0;
									I1PC=(IMAT-INVCG*(1.0/3.0*I1))*pow(I3,-1.0/3.0);
									I2PC=(((IMAT*I1)-CG)-(INVCG*(2.0/3.0)*I2))*pow(I3,-2.0/3.0);
									JPC=INVCG*(J/2.0);
									STR=I1PC*(CCOM1*2.0)+I2PC*(CCOM2*2.0)+JPC*(CCOM3*(J-1.0)*2.0);

									TEMP5(0,0)=STR(0,0);TEMP5(1,0)=STR(1,1);TEMP5(2,0)=STR(0,1);
									TEMP5(3,0)=STR(2,2);TEMP5(4,0)=STR(0,2);TEMP5(5,0)=STR(1,2);

									E_eps.Reset();

									strain_1=strain;
									for(int JJJ=0;JJJ<6;JJJ++){
										StockEPS=strain_1(JJJ,0);
										strain_1(JJJ,0)=StockEPS+Deld;
										CGN(0,0)=2.0*strain_1(0,0)+1.0;
										CGN(1,1)=2.0*strain_1(1,0)+1.0;
                                        CGN(2,2)=2.0*strain_1(3,0)+1.0;
                                        CGN(1,0)=strain_1(2,0);
                                        CGN(0,1)=CGN(1,0);
                                        CGN(2,0)=strain_1(4,0);
										CGN(0,2)=CGN(2,0);
										CGN(2,1)=strain_1(5,0);
										CGN(1,2)=CGN(2,1);
										INVCGN=CGN;
									    INVCGN.MatrInverse();
										//GetLog() << " INVCGN"<<INVCGN<<"\n";

										I1 =CGN(0,0)+CGN(1,1)+CGN(2,2);
										I2=0.5*(pow(I1,2)-(pow(CGN(0,0),2)+pow(CGN(1,0),2)+pow(CGN(2,0),2)+pow(CGN(0,1),2)+pow(CGN(1,1),2)+pow(CGN(2,1),2)+pow(CGN(0,2),2)+pow(CGN(1,2),2)+pow(CGN(2,2),2)));
										I3=CGN(0,0)*CGN(1,1)*CGN(2,2)-CGN(0,0)*CGN(1,2)*CGN(2,1)+CGN(0,1)*CGN(1,2)*CGN(2,0)-CGN(0,1)*CGN(1,0)*CGN(2,2)+CGN(0,2)*CGN(1,0)*CGN(2,1)-CGN(2,0)*CGN(1,1)*CGN(0,2);
										J=sqrt(I3);
										I1BAR=I1/(pow(I3,1.0/3.0));
										I2BAR=I2/(pow(I3,2.0/3.0));
										I1PCN=(IMAT-INVCGN*(1.0/3.0*I1))*pow(I3,-1.0/3.0);
										I2PCN=(((IMAT*I1)-CGN)-(INVCGN*(2.0/3.0)*I2))*pow(I3,-2.0/3.0);
										JPCN=INVCGN*(J/2.0);
										STRN=I1PCN*(CCOM1*2.0)+I2PCN*(CCOM2*2.0)+JPCN*(CCOM3*(J-1.0)*2.0);
								        TEMP5N(0,0)=STRN(0,0);TEMP5N(1,0)=STRN(1,1);TEMP5N(2,0)=STRN(0,1);
									    TEMP5N(3,0)=STRN(2,2);TEMP5N(4,0)=STRN(0,2);TEMP5N(5,0)=STRN(1,2);
										strain_1(JJJ,0)=StockEPS;
										E_eps(JJJ,0)=(TEMP5N(0,0)-TEMP5(0,0))/Deld;E_eps(JJJ,1)=(TEMP5N(1,0)-TEMP5(1,0))/Deld;E_eps(JJJ,2)=(TEMP5N(2,0)-TEMP5(2,0))/Deld;
										E_eps(JJJ,3)=(TEMP5N(3,0)-TEMP5(3,0))/Deld;E_eps(JJJ,4)=(TEMP5N(4,0)-TEMP5(4,0))/Deld;E_eps(JJJ,5)=(TEMP5N(5,0)-TEMP5(5,0))/Deld;
										//for(int i=0;i<6;i++){
										//	GetLog() <<TEMP5N(i)<<"\n";}
										//system("pause");
										//for(int i=0;i<6;i++){
										//	GetLog() <<TEMP5(i)<<"\n";}
										//system("pause");
									   }
									temp56.MatrMultiply(GT,E_eps);
									Fint.MatrTMultiply(strainD,TEMP5);
									Fint *= detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
									HE1.MatrMultiply(GT,TEMP5);
									HE1*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
									Sigm(0,0)=TEMP5(0,0); Sigm(1,1)=TEMP5(0,0); Sigm(2,2)=TEMP5(0,0);
									Sigm(0,3)=TEMP5(2,0); Sigm(1,4)=TEMP5(2,0); Sigm(2,5)=TEMP5(2,0);
									Sigm(0,6)=TEMP5(4,0); Sigm(1,7)=TEMP5(4,0); Sigm(2,8)=TEMP5(4,0);
									Sigm(3,0)=TEMP5(2,0); Sigm(4,1)=TEMP5(2,0); Sigm(5,2)=TEMP5(2,0);
									Sigm(3,3)=TEMP5(1,0); Sigm(4,4)=TEMP5(1,0); Sigm(5,5)=TEMP5(1,0);
									Sigm(3,6)=TEMP5(5,0); Sigm(4,7)=TEMP5(5,0); Sigm(5,8)=TEMP5(5,0);
									Sigm(6,0)=TEMP5(4,0); Sigm(7,1)=TEMP5(4,0); Sigm(8,2)=TEMP5(4,0);
									Sigm(6,3)=TEMP5(5,0); Sigm(7,4)=TEMP5(5,0); Sigm(8,5)=TEMP5(5,0);
									Sigm(6,6)=TEMP5(3,0); Sigm(7,7)=TEMP5(3,0); Sigm(8,8)=TEMP5(3,0);
								}else{
								/// Stress tensor calculation
								stress.MatrMultiply(E_eps,strain);
								Sigm(0,0)=stress(0,0);   Sigm(0,3)=stress(2,0);   Sigm(0,6)=stress(4,0);
								Sigm(1,1)=stress(0,0);   Sigm(1,4)=stress(2,0);   Sigm(1,7)=stress(4,0);
								Sigm(2,2)=stress(0,0);   Sigm(2,5)=stress(2,0);   Sigm(2,8)=stress(4,0);
							   //XX                      //XY                     //XZ

								Sigm(3,0)=stress(2,0);   Sigm(3,3)=stress(1,0);   Sigm(3,6)=stress(5,0);
								Sigm(4,1)=stress(2,0);   Sigm(4,4)=stress(1,0);   Sigm(4,7)=stress(5,0);
								Sigm(5,2)=stress(2,0);   Sigm(5,5)=stress(1,0);   Sigm(5,8)=stress(5,0);
								//XY                     //YY                     //YZ

								Sigm(6,0)=stress(4,0);   Sigm(6,3)=stress(5,0);   Sigm(6,6)=stress(3,0);
								Sigm(7,1)=stress(4,0);   Sigm(7,4)=stress(5,0);   Sigm(7,7)=stress(3,0);
								Sigm(8,2)=stress(4,0);   Sigm(8,5)=stress(5,0);   Sigm(8,8)=stress(3,0);
								//XZ                     //YZ                     //ZZ

								temp56.MatrMultiply(GT,E_eps);
								tempC.MatrTMultiply(strainD,E_eps);
								Fint.MatrMultiply(tempC,strain);
								Fint *= detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								HE1.MatrMultiply(temp56,strain);
								HE1*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								} //end of   if(*flag_Mooney==1)

								/// Jacobian calculation
								temp246.MatrTMultiply(strainD,E_eps);
								temp249.MatrTMultiply(Gd,Sigm);
								JAC11=temp246*strainD+temp249*Gd;
								JAC11*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								
								GDEPSP.MatrMultiply(temp56,strainD);
								GDEPSP*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);
								KALPHA.MatrMultiply(temp56,G);
								KALPHA*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetLengthZ()/2.0);

								ChMatrixNM<double, 216,1> GDEPSPVec;
								ChMatrixNM<double, 81,1>  KALPHAVec;
								ChMatrixNM<double, 576,1>  JACVec;

								result.Reset();
								GDEPSPVec = GDEPSP;
								KALPHAVec = KALPHA;
								JACVec = JAC11;
								result.PasteClippedMatrix(&Fint,0,0,24,1,0,0); 
								result.PasteClippedMatrix(&HE1,0,0,9,1,24,0);
								result.PasteClippedMatrix(&GDEPSPVec,0,0,216,1,33,0); 
								result.PasteClippedMatrix(&KALPHAVec,0,0,81,1,249,0); 
								result.PasteClippedMatrix(&JACVec,0,0,576,1,330,0);
							}
						};//end of class MyForce
						////////////////////////////////////////////////////////////////////////////////////////////////////////////
						ChMatrixNM<double, 906,1> TempIntegratedResult;
						ChMatrixNM<double, 24,1> Finternal;
						// Enhanced Assumed Strain (EAS)
						ChMatrixNM<double, 6,6>   T0;
						ChMatrixNM<double, 9,1>   HE;
						ChMatrixNM<double, 9,24>  GDEPSP;
						ChMatrixNM<double, 9,9>   KALPHA;
						ChMatrixNM<double, 24,24> KTE;
						ChMatrixNM<double, 9,9>   KALPHA1;
						ChMatrixNM<double, 9,1>   ResidHE;
						double detJ0C;
						ChMatrixNM<double, 9,1>   alpha_eas;
						ChMatrixNM<double, 9,1>   renewed_alpha_eas;
						ChMatrixNM<double, 9,1> previous_alpha;
					
						previous_alpha=GetStockAlpha();
						alpha_eas = previous_alpha;
						ResidHE.Reset();
						int flag_M=1;// 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material
						int count=0;
						int fail=1;
						while(fail==1)
						{
							alpha_eas = alpha_eas - ResidHE;
							renewed_alpha_eas = alpha_eas;

							Finternal.Reset();
							HE.Reset();
							GDEPSP.Reset();
							KALPHA.Reset();

							// Enhanced Assumed Strain (EAS)
							T0.Reset();
							detJ0C=0.0;
							T0DetJElementCenterForEAS(d0,T0,detJ0C);
							//== F_internal ==//
							MyForce myformula;
							myformula.d = &d;
							myformula.d0 = &d0;

							if(flag_M==0){
							myformula.E = &E;
							myformula.v = &v;
							}

							myformula.element = this;
						//	//EAS
							myformula.T0 = &T0;
							myformula.detJ0C = &detJ0C;
							myformula.alpha_eas = &alpha_eas;
							ChQuadrature::Integrate3D< ChMatrixNM<double,906,1> >(
											TempIntegratedResult,				// result of integration will go there
											myformula,			// formula to integrate
											-1,					// start of x
											1,					// end of x
											-1,					// start of y
											1,					// end of y
											-1,					// start of z
											1,					// end of z
											2					// order of integration
											);
						//	///===============================================================//
						//	///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
						//	///===TempIntegratedResult(24:28,1) -> HE(5x1)           =========//
						//	///===TempIntegratedResult(29:148,1) -> GDEPSP(5x24)     =========//
						//	///===TempIntegratedResult(149:173,1) -> KALPHA(5x5)     =========//
						//	///===TempIntegratedResult(174:749,1) -> Stiffness Matrix(24x24) =//
						//	///===============================================================//
							ChMatrixNM<double, 216,1> GDEPSPvec;
							ChMatrixNM<double, 81,1> KALPHAvec;
							ChMatrixNM<double, 576,1> JACvec;
							Finternal.PasteClippedMatrix(&TempIntegratedResult,0,0,24,1,0,0); // 
							HE.PasteClippedMatrix(&TempIntegratedResult,24,0,9,1,0,0); // 
							GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult,33,0,216,1,0,0); //
							KALPHAvec.PasteClippedMatrix(&TempIntegratedResult,249,0,81,1,0,0); //
							JACvec.PasteClippedMatrix(&TempIntegratedResult,330,0,576,1,0,0); //
							GDEPSP=GDEPSPvec;
							KALPHA=KALPHAvec;
							KTE = JACvec;
							KALPHA1=KALPHA;
								 //   GetLog() <<HE<<"\n";
									//system("pause");
							  //      GetLog() <<Finternal<<"\n";
									//system("pause");
							if(flag_HE==1) break; // When numerical jacobian loop, no need to calculate HE
							count = count + 1;
							double norm_HE = HE.NormTwo();
							if(norm_HE < 0.00001) 
							{fail=0;
							}else{
							ChMatrixNM<int, 9,1> INDX;
							double DAMMY = 0.0;
							ResidHE=HE;
							LUDCMP55(KALPHA1,9,9,INDX,DAMMY);
							LUBKSB55(KALPHA1,9,9,INDX,ResidHE);
							}
						}//end of while
						Fi = -Finternal;
						////===============================//
						////== Stock_Alpha=================//
						////===============================//
						if(flag_HE==0){
							SetStockAlpha(renewed_alpha_eas(0,0),renewed_alpha_eas(1,0),renewed_alpha_eas(2,0),renewed_alpha_eas(3,0),renewed_alpha_eas(4,0),renewed_alpha_eas(5,0),renewed_alpha_eas(6,0),renewed_alpha_eas(7,0),renewed_alpha_eas(8,0)); //this->
						}
						////===============================//
						////== Jacobian Matrix for alpha ==//
						////===============================//
						if(flag_HE==0){
							ChMatrixNM<double, 9,9> INV_KALPHA;
							ChMatrixNM<double, 9,24> TEMP_GDEPSP;
							ChMatrixNM<double, 9,9> INV_KALPHA_Temp;
							ChMatrixNM<double, 24,24> stock_jac_EAS_elem;

							for(int ii=0;ii<9;ii++){
								double DAMMY = 0.0;
								INV_KALPHA_Temp=KALPHA;
								ChMatrixNM<int, 9,1> INDX;
								ChMatrixNM<double, 9,1> DAMMY_vec;
								DAMMY_vec.Reset();
								DAMMY_vec(ii)=1.0;
								LUDCMP55(INV_KALPHA_Temp,9,9,INDX,DAMMY);
								LUBKSB55(INV_KALPHA_Temp,9,9,INDX,DAMMY_vec);
								INV_KALPHA.PasteClippedMatrix(&DAMMY_vec,0,0,9,1,0,ii); //
							}
							TEMP_GDEPSP.MatrMultiply(INV_KALPHA,GDEPSP);
							stock_jac_EAS_elem.MatrTMultiply(GDEPSP,TEMP_GDEPSP);
							this->SetStockKTE(KTE);
							this->SetStockJac(stock_jac_EAS_elem);
						}
					}//end of else for numerical or analytical
					
					//Add gravity force
					class MyGravity : public ChIntegrable3D< ChMatrixNM<double,24,1> >
					{
					public:
						ChElementBrick* element;
						ChMatrixNM<double, 8,3> *d0;
						ChMatrixNM<double, 3,24> S;
						ChMatrixNM<double, 1,8> N;
						ChMatrixNM<double, 1,8> Nx;
						ChMatrixNM<double, 1,8> Ny;
						ChMatrixNM<double, 1,8> Nz;
						ChMatrixNM<double, 3,1> LocalGravityForce;

						virtual void Evaluate(ChMatrixNM<double,24,1>& result, const double x, const double y, const double z)
						{
							element->ShapeFunctions(N, x, y, z);
						    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
							element->ShapeFunctionsDerivativeY(Ny, x, y, z);
							element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

							// Weights for Gaussian integration
							double wx2 = (element->GetLengthX())/2;
							double wy2 = (element->GetLengthY())/2;
							double wz2 = (element->GetLengthZ())/2;

							//Set gravity acceleration
							LocalGravityForce(0,0) = 0.0;
							LocalGravityForce(1,0) = 0.0;
							LocalGravityForce(2,0) = -9.81;

							// S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]
							ChMatrix33<> Si;
							Si.FillDiag(N(0));
							S.PasteMatrix(&Si, 0,0);
							Si.FillDiag(N(1));
							S.PasteMatrix(&Si, 0,3);
							Si.FillDiag(N(2));
							S.PasteMatrix(&Si, 0,6);
							Si.FillDiag(N(3));
							S.PasteMatrix(&Si, 0,9);
							Si.FillDiag(N(4));
							S.PasteMatrix(&Si, 0,12);
							Si.FillDiag(N(5));
							S.PasteMatrix(&Si, 0,15);
							Si.FillDiag(N(6));
							S.PasteMatrix(&Si, 0,18);
							Si.FillDiag(N(7));
							S.PasteMatrix(&Si, 0,21);

							ChMatrixNM<double, 1,3> Nx_d0;
							Nx_d0.MatrMultiply(Nx,*d0);

							ChMatrixNM<double, 1,3> Ny_d0;
							Ny_d0.MatrMultiply(Ny,*d0);

							ChMatrixNM<double, 1,3> Nz_d0;
							Nz_d0.MatrMultiply(Nz,*d0);

							ChMatrixNM<double, 3,3> rd0;
							rd0(0,0) = Nx_d0(0,0); rd0(1,0) = Nx_d0(0,1); rd0(2,0) = Nx_d0(0,2);
							rd0(0,1) = Ny_d0(0,0); rd0(1,1) = Ny_d0(0,1); rd0(2,1) = Ny_d0(0,2);
							rd0(0,2) = Nz_d0(0,0); rd0(1,2) = Nz_d0(0,1); rd0(2,2) = Nz_d0(0,2);

							double detJ0 = rd0.Det();

							result.MatrTMultiply(S,LocalGravityForce);

							result *= detJ0*wx2*wy2*wz2*(element->Material->Get_density());
						}
					};

					MyGravity myformula1;
					myformula1.d0 = &d0;
					myformula1.element = this;

					ChMatrixNM<double, 24,1> Fgravity;
					ChQuadrature::Integrate3D< ChMatrixNM<double,24,1> >(
									Fgravity,				// result of integration will go there
									myformula1,			// formula to integrate
									-1,					// start of x
									1,					// end of x
									-1,					// start of y
									1,					// end of y
									-1,					// start of z
									1,					// end of z
									2					// order of integration
									);
					Fi += Fgravity;
						    ////check gravity force 
							//for (i=0;i<24;i++){
							//	GetLog()<<Fgravity(i)<<"\n";
							//}
							//system("pause");
				}//end of ComputeInternalForces

	// [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    virtual void T0DetJElementCenterForEAS(ChMatrixNM<double,8,3>& d0,ChMatrixNM<double,6,6>& T0, double& detJ0C)
				{
					double x = 0;
					double y = 0;
					double z = 0;
					//ChMatrixNM<double,6,6> T0;
					//double detJ0C;
					ChMatrixNM<double, 1,8>   Nx;
					ChMatrixNM<double, 1,8>   Ny;
					ChMatrixNM<double, 1,8>   Nz;
					ChMatrixNM<double, 3,3>   rd0;
					ChMatrixNM<double, 3,3>   tempA;
					tempA.Reset();
					ShapeFunctionsDerivativeX(Nx, x, y, z);
					ShapeFunctionsDerivativeY(Ny, x, y, z);
					ShapeFunctionsDerivativeZ(Nz, x, y, z);
					tempA=(Nx*d0);
					tempA.MatrTranspose();
					rd0.PasteClippedMatrix(&tempA,0,0,3,1,0,0);
					tempA=(Ny*d0);
					tempA.MatrTranspose();
					rd0.PasteClippedMatrix(&tempA,0,0,3,1,0,1);
					tempA=(Nz*d0);
					tempA.MatrTranspose();
					rd0.PasteClippedMatrix(&tempA,0,0,3,1,0,2);
					detJ0C=rd0.Det();

					//////////////////////////////////////////////////////////////
					//// Transformation : Orthogonal transformation (A and J) ////
					//////////////////////////////////////////////////////////////
					ChVector<double> G1;
					ChVector<double> G2;
					ChVector<double> G3;
					ChVector<double> G1xG2;
					double G1norm;
					double G1dotG1;
					G1(0)=rd0(0,0); G2(0)=rd0(0,1); G3(0)=rd0(0,2);
					G1(1)=rd0(1,0); G2(1)=rd0(1,1); G3(1)=rd0(1,2);
					G1(2)=rd0(2,0); G2(2)=rd0(2,1); G3(2)=rd0(2,2);
					G1xG2.Cross(G1,G2);
					G1dotG1=Vdot(G1,G1);

					////Tangent Frame
					ChVector<double> A1;
					ChVector<double> A2;
					ChVector<double> A3;;
					A1=G1/sqrt(G1(0)*G1(0)+G1(1)*G1(1)+G1(2)*G1(2));
					A3=G1xG2/sqrt(G1xG2(0)*G1xG2(0)+G1xG2(1)*G1xG2(1)+G1xG2(2)*G1xG2(2));
					A2.Cross(A3,A1);
					double theta = 0.0;
					ChVector<double> AA1;
					ChVector<double> AA2;
					ChVector<double> AA3;
					AA1 = A1*cos(theta)+A2*sin(theta);
					AA2 = -A1*sin(theta)+A2*cos(theta);
					AA3 = A3;

					////Beta
					ChMatrixNM<double, 3,3> j0;
					ChVector<double> j01;
					ChVector<double> j02;
					ChVector<double> j03;
					ChMatrixNM<double, 9,1> beta;
					double temp;
					j0 = rd0;
					j0.MatrInverse();
					j01(0)=j0(0,0); j02(0)=j0(1,0); j03(0)=j0(2,0);
					j01(1)=j0(0,1); j02(1)=j0(1,1); j03(1)=j0(2,1);
					j01(2)=j0(0,2); j02(2)=j0(1,2); j03(2)=j0(2,2);
					temp = Vdot(AA1,j01); beta(0,0) = temp;
					temp = Vdot(AA2,j01); beta(1,0) = temp;
					temp = Vdot(AA3,j01); beta(2,0) = temp;
					temp = Vdot(AA1,j02); beta(3,0) = temp;
					temp = Vdot(AA2,j02); beta(4,0) = temp;
					temp = Vdot(AA3,j02); beta(5,0) = temp;
					temp = Vdot(AA1,j03); beta(6,0) = temp;
					temp = Vdot(AA2,j03); beta(7,0) = temp;
					temp = Vdot(AA3,j03); beta(8,0) = temp;

					////T0
					T0(0,0)=pow(beta(0),2);
					T0(1,0)=pow(beta(1),2);
					T0(2,0)=2.0*beta(0)*beta(1);
					T0(3,0)=pow(beta(2),2);
					T0(4,0)=2.0*beta(0)*beta(2);
					T0(5,0)=2.0*beta(1)*beta(2);
  
					T0(0,1)=pow(beta(3),2);
					T0(1,1)=pow(beta(4),2);
					T0(2,1)=2.0*beta(3)*beta(4);
					T0(3,1)=pow(beta(5),2);
					T0(4,1)=2.0*beta(3)*beta(5);
					T0(5,1)=2.0*beta(4)*beta(5);
  
					T0(0,2)=beta(0)*beta(3);
					T0(1,2)=beta(1)*beta(4);
					T0(2,2)=beta(0)*beta(4)+beta(1)*beta(3);
					T0(3,2)=beta(2)*beta(5);
					T0(4,2)=beta(0)*beta(5)+beta(2)*beta(3);
					T0(5,2)=beta(2)*beta(4)+beta(1)*beta(5);
  
					T0(0,3)=pow(beta(6),2);
					T0(1,3)=pow(beta(7),2);
					T0(2,3)=2.0* beta(6)*beta(7);
					T0(3,3)=pow(beta(8),2);
					T0(4,3)=2.0* beta(6)*beta(8);
					T0(5,3)=2.0* beta(7)*beta(8);
  
					T0(0,4)=beta(0)*beta(6);
					T0(1,4)=beta(1)*beta(7);
					T0(2,4)=beta(0)*beta(7)+beta(6)*beta(1);
					T0(3,4)=beta(2)*beta(8);
					T0(4,4)=beta(0)*beta(8)+beta(2)*beta(6);
					T0(5,4)=beta(1)*beta(8)+beta(2)*beta(7);
  
					T0(0,5)=beta(3)*beta(6);
					T0(1,5)=beta(4)*beta(7);
					T0(2,5)=beta(3)*beta(7)+beta(4)*beta(6);
					T0(3,5)=beta(5)*beta(8);
					T0(4,5)=beta(3)*beta(8)+beta(6)*beta(5);
					T0(5,5)=beta(4)*beta(8)+beta(5)*beta(7);

				}

	//// Temporary function of LU decomposition 1 (new version)
		virtual void LUBKSB55(ChMatrixNM<double,9,9>&A,double N,double NP,ChMatrixNM<int,9,1>& INDX,ChMatrixNM<double,9,1>& B){
			int II=0;
			int LL;
			double SUM;
			for (int I=0;I<N;I++){
			  LL=INDX(I);
			  SUM=B(LL);
			  B(LL)=B(I);
			  if(II!=0){
			  for (int J=II-1;J<I;J++){
			    SUM-=A(I,J)*B(J);
			  }
			  }else if(SUM!=0.0){
			    II=I+1;                
			  }
			  B(I)=SUM;
			}
			
		    for(int I=N-1;I>=0;I--){
			  SUM=B(I);
			  for( int J=I+1;J<N;J++){
			       SUM-=A(I,J)*B(J);
			  }
			  B(I)=SUM/A(I,I);
			}
	}

	// Temporary function of LU decomposition 2 (new version)
		virtual void LUDCMP55(ChMatrixNM<double,9,9>&A,double N,double NP,ChMatrixNM<int,9,1>& INDX,double D){
      int NMAX=3000;
	  double AAMAX;
	  double SUM;
	  double DUM;
	  int IMAX;
	  double TINY=1.0e-20;
	  ChMatrixNM<double,3000,1> VV; 
      D=1.0;
      for(int I=0;I<N;I++){
        AAMAX=0.0;
        for(int J=0;J<NP;J++){
			if((abs(A(I,J)))>AAMAX){AAMAX=(abs(A(I,J)));};
		}
		if(AAMAX==0.0){ GetLog() <<"SINGULAR MATRIX."; system("pause"); }
        VV(I)=1.0/AAMAX;     
	  }

      for(int J=0;J<NP;J++){                
			 for(int I=0;I<J;I++){        
				SUM=A(I,J);                
				for(int K=0;K<I;K++){        
					  SUM-=A(I,K)*A(K,J); 
				}
			 A(I,J)=SUM;          			               
			 }

      AAMAX=0.0;           
	  for(int I=J;I<N;I++){          
		  SUM=A(I,J);                
		  for (int K=0;K<J;K++){        
			    SUM-=A(I,K)*A(K,J); 
		  }
		  A(I,J)=SUM;   
		  if(DUM=VV(I)*(abs(SUM))>=AAMAX){ 
			  IMAX=I;                
			  AAMAX=DUM;             
		  }
	  }

      if(J!=IMAX){    
		  for(int K=0;K<NP;K++){          
			  DUM=A(IMAX,K);         
			  A(IMAX,K)=A(J,K);      
			  A(J,K)=DUM;     
		  }
		  D=-D;       
		  VV(IMAX)=VV(J);        
	  }

      INDX(J)=IMAX;
	  if(A(J,J)==0.0){A(J,J)=TINY;}
      if(J!=N-1){       
		  DUM=1.0/A(J,J);                
		  for(int I=J+1;I<N;I++){                 
		    A(I,J)*=DUM;              
		  }
	  }
	  }
	}
	// [EAS] Basis function of M for Enhanced Assumed Strain
    virtual void Basis_M(ChMatrixNM<double,6,9>& M, double x, double y, double z)
				{
					M.Reset();
					M(0,0) = x;
					M(1,1) = y;
					M(2,2) = x;
					M(2,3) = y;
					M(3,4) = z;
					M(4,5) = x;
					M(4,6) = z;
					M(5,7) = y;
					M(5,8) = z;
				}
};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif







