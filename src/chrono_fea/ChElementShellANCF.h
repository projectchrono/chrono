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

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H



#include "ChElementShell.h"
#include "physics/ChContinuumMaterial.h"
#include "ChNodeFEAxyzD.h"
#include "core/ChQuadrature.h"


namespace chrono
{

namespace fea
{



/// ANCF laminated shell element with four nodes.

class  ChElementShellANCF : public ChElementShell
{
protected:
	std::vector< ChSharedPtr<ChNodeFEAxyzD> > nodes;
	
	double thickness;
	int elementnumber;
	double Alpha;
	ChSharedPtr<ChContinuumElastic> Material;

	ChMatrixNM<double,24,24> StiffnessMatrix; // stiffness matrix
	ChMatrixNM<double,24,24> MassMatrix;	   // mass matrix

	ChMatrixNM<double,24,24> stock_jac_EAS; // EAS per elmeent 24

	ChMatrixNM<double, 24,24> stock_KTE; // Analytical Jacobian

	ChMatrixNM<double, 24,1> initialposD; // Initial Coordinate per element
	ChMatrixNM<double, 24,1> GravForce;  // Gravity Force 

	// Material Properties for orthotropic per element (14x7) Max #layer is 7
	ChMatrixNM<double,98,1> InertFlexVec; //2015/5/28  for Laminate shell
	ChMatrixNM<double,35,1> StockAlpha_EAS; // StockAlpha(5*7,1): Max #Layer is 7
	ChMatrixNM<double,7,2> GaussZRange; // StockAlpha(7,2): Max #Layer is 7 (-1 < GaussZ < 1)
	
	int NumLayer;     

	int flag_HE=0;

	double dt;

	int FlagGravity;

	int FlagAirPressure;

public:

	ChElementShellANCF()
				{
					nodes.resize(4);

					this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());
					this->MassMatrix.Resize(this->GetNdofs(), this->GetNdofs());

				}

	virtual ~ChElementShellANCF() {}

	virtual int GetNnodes()  {return 4;}
	virtual int GetNcoords() {return 4*6;}
	virtual int GetNdofs()   {return 4*6;}

	virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes( ChSharedPtr<ChNodeFEAxyzD> nodeA, ChSharedPtr<ChNodeFEAxyzD> nodeB, ChSharedPtr<ChNodeFEAxyzD> nodeC, ChSharedPtr<ChNodeFEAxyzD> nodeD) 
				{
					assert(!nodeA.IsNull());
					assert(!nodeB.IsNull());
					assert(!nodeC.IsNull());
					assert(!nodeD.IsNull());

					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					nodes[2]=nodeC;
					nodes[3]=nodeD;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[0]->Variables_D());
					mvars.push_back(&nodes[1]->Variables());
					mvars.push_back(&nodes[1]->Variables_D());
					mvars.push_back(&nodes[2]->Variables());
					mvars.push_back(&nodes[2]->Variables_D());
					mvars.push_back(&nodes[3]->Variables());
					mvars.push_back(&nodes[3]->Variables_D());
					Kmatr.SetVariables(mvars);

					// Initial position and slope per element //// 2015/5/23  for Initial position
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> dA = this->nodes[0]->GetD();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> dB = this->nodes[1]->GetD();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> dC = this->nodes[2]->GetD();
					ChVector<> pD = this->nodes[3]->GetPos();
					ChVector<> dD = this->nodes[3]->GetD();
					initialposD(0,0)=pA(0); initialposD(1,0)=pA(1); initialposD(2,0)=pA(2);
					initialposD(3,0)=dA(0); initialposD(4,0)=dA(1); initialposD(5,0)=dA(2);
					initialposD(6,0)=pB(0); initialposD(7,0)=pB(1); initialposD(8,0)=pB(2);
					initialposD(9,0)=dB(0); initialposD(10,0)=dB(1); initialposD(11,0)=dB(2);
					initialposD(12,0)=pC(0); initialposD(13,0)=pC(1); initialposD(14,0)=pC(2);
					initialposD(15,0)=dC(0); initialposD(16,0)=dC(1); initialposD(17,0)=dC(2);
					initialposD(18,0)=pD(0); initialposD(19,0)=pD(1); initialposD(20,0)=pD(2);
					initialposD(21,0)=dD(0); initialposD(22,0)=dD(1); initialposD(23,0)=dD(2);

				}

			//
			// FEM functions
			//

				/// Set the section & material of shell element .
				/// It is a shared property, so it can be shared between other beams.
	void   SetThickness( double th) { thickness = th;} // Total shell thickness

	void   SetElemNum(int kb){ elementnumber = kb;}         //// 2015/5/23 for EAS

	void   SetStockAlpha(ChMatrixNM<double,35,1> a){ StockAlpha_EAS = a;}

	void   SetStockJac(ChMatrixNM<double,24,24> a){stock_jac_EAS=a;	} //// 2015/5/23  for EAS

	void   SetStockKTE(ChMatrixNM<double,24,24> a){stock_KTE=a;	} //// 2015/5/23  for EAS

	void   SetGravForce(ChMatrixNM<double,24,1> a){GravForce=a; }

	void   SetInertFlexVec(ChMatrixNM<double,98,1> a){InertFlexVec = a;	} //// 2015/5/28  for Laminate shell

	void   SetNumLayer(int a){NumLayer = a;} //// 2015/5/28  for Laminate shell

	void   SetGaussZRange(ChMatrixNM<double,7,2> a){GaussZRange = a;} //// 2015/6/1  for Laminate shell

	void   Setdt(double a) { dt = a;} // To calculate structural damping coefficient

	void   SetGravityZ(int a){FlagGravity = a;} // Gravity Flag

	void   SetAirPressure(int a){FlagAirPressure = a;} // AirPressure Flag

				/// Get the section & material of the element
	double GetThickness() {return thickness;} /// Total shell thickness

	int GetElemNum() {return elementnumber;}                //// 2015/5/23  for EAS

	ChMatrixNM<double,35,1> GetStockAlpha() {return StockAlpha_EAS;} //// 2015/5/23  for EAS

	ChMatrixNM<double,24,24> GetStockJac() { return stock_jac_EAS;} //// 2015/5/23  for EAS

	ChMatrixNM<double,24,24> GetStockKTE() { return stock_KTE;} //// Retrieve ananyltical jacobian

	ChMatrixNM<double,24,1> GetGravForce() { return GravForce;}

	ChMatrixNM<double,24,1> GetInitialPosD() {return initialposD;} //// 2015/5/23  for Initial position

	ChMatrixNM<double,98,1> GetInertFlexVec() {return InertFlexVec;} //// 2015/5/28  for Laminate shell

	ChMatrixNM<double,7,2> GetGaussZRange() {return GaussZRange;} //// 2015/6/1  for Laminate shell

	int GetNumLayer(){return NumLayer;} //// 2015/5/28  for Laminate shell

	double Getdt(){return dt;} //// To calculate structural damping coefficient

	int GetFlagGravity(){return FlagGravity;} // Gravity Flag

	int GetAirPressure(){return FlagAirPressure;} // AirPressure Flag

	/// 2015/6/23 Structural Damping 
	void SetAlphaDamp(double a) {Alpha = a;}

	double GetAlphaDamp() {return Alpha;}
	

				/// Get each node
	ChSharedPtr<ChNodeFEAxyzD> GetNodeA() {return nodes[0];}

	ChSharedPtr<ChNodeFEAxyzD> GetNodeB() {return nodes[1];}

	ChSharedPtr<ChNodeFEAxyzD> GetNodeC() {return nodes[2];}

	ChSharedPtr<ChNodeFEAxyzD> GetNodeD() {return nodes[3];}

	//double GetLengthX() {return nodes[1]->GetX0().x - nodes[0]->GetX0().x;}
	double GetLengthX() {return InertFlexVec(1);} // For laminate shell. each layer has the same elmenet length

	//double GetLengthY() {return nodes[2]->GetX0().y - nodes[0]->GetX0().y;}
    double GetLengthY() {return InertFlexVec(2);} // For laminate shell. each layer has the same elmenet length

	void SetMaterial(ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
    ChSharedPtr<ChContinuumElastic> GetMaterial() { return Material; }

	 
	

				/// Fills the N shape function matrix.
				/// NOTE! actually N should be a 3row, 24 column sparse matrix,
				/// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
				/// but to avoid wasting zero and repeated elements, here
				/// it stores only the s1 through s8 values in a 1 row, 8 columns matrix!
	virtual void ShapeFunctions(ChMatrix<>& N, double x, double y, double z)
				{
					double a = this->GetLengthX();
					double b = this->GetLengthY();
					double c = this->GetThickness();

					N(0) = 0.25*(1.0-x)*(1.0-y);
					N(1) = z*c/2.0*0.25*(1.0-x)*(1.0-y);
					N(2) = 0.25*(1.0+x)*(1.0-y);
					N(3) = z*c/2.0*0.25*(1.0+x)*(1.0-y);
					N(4) = 0.25*(1.0-x)*(1.0+y);
					N(5) = z*c/2.0*0.25*(1.0-x)*(1.0+y);
					N(6) = 0.25*(1.0+x)*(1.0+y);
					N(7) = z*c/2.0*0.25*(1.0+x)*(1.0+y);

				};

				/// Fills the N shape function derivative matrix with respect to
				/// the x, y, and z coordinate.
				/// NOTE! to avoid wasting zero and repeated elements, here
				/// it stores only the four values in a 1 row, 8 columns matrix!
	virtual void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z)
				{
					double a = this->GetLengthX();
					double b = this->GetLengthY();
					double c = this->GetThickness();

					Nx(0) = 0.25 * (-2.0/a) * (1.0-y);
					Nx(1) = z * c / 2.0 * 0.25 * (-2.0/a) * (1.0-y);
					Nx(2) = 0.25 * (2.0/a) * (1.0-y);
					Nx(3) = z * c / 2.0 * 0.25 * (2.0 /a) * (1.0-y);
					Nx(4) = 0.25 * (-2.0/a) * (1.0+y);
					Nx(5) = z * c / 2.0 * 0.25 * (-2.0/a) * (1.0+y);
					Nx(6) = 0.25 * (2.0/a) * (1.0+y);
					Nx(7) = z * c / 2.0 * 0.25 * (2.0/a) * (1.0+y);
				};

	virtual void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z)
				{
					double a = this->GetLengthX();
					double b = this->GetLengthY();
					double c = this->GetThickness();

					Ny(0) = 0.25 * (1.0-x)*(-2.0/b);
					Ny(1) = z * c / 2.0 * 0.25 * (1.0-x) * (-2.0/b);
					Ny(2) = 0.25 * (1.0+x)*(-2.0/b);
					Ny(3) = z * c / 2.0*0.25*(1.0+x)*(-2.0/b);
					Ny(4) = 0.25 * (1.0-x)*( 2.0/b);
					Ny(5) = z * c / 2.0 *0.25*(1.0-x)*( 2.0/b);
					Ny(6) = 0.25 * (1.0+x)*( 2.0/b);
					Ny(7) = z * c / 2.0 * 0.25 * (1.0+x) * (2.0/b);
				};

	virtual void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z)
				{
					double a = this->GetLengthX();
					double b = this->GetLengthY();
					double c = this->GetThickness();

					Nz(0) = 0.0;
					Nz(1) = 0.250*(1.0-x)*(1.0-y);
					Nz(2) = 0.0;
					Nz(3) = 0.250*(1.0+x)*(1.0-y);
					Nz(4) = 0.0;
					Nz(5) = 0.250*(1.0-x)*(1.0+y);
					Nz(6) = 0.0;
					Nz(7) = 0.250*(1.0+x)*(1.0+y);
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
					mD.PasteVector(this->nodes[0]->GetD(),3,0);
					mD.PasteVector(this->nodes[1]->GetPos(),6,0);
					mD.PasteVector(this->nodes[1]->GetD(),9,0);
					mD.PasteVector(this->nodes[2]->GetPos(),12,0);
					mD.PasteVector(this->nodes[2]->GetD(),15,0);
					mD.PasteVector(this->nodes[3]->GetPos(),18,0);
					mD.PasteVector(this->nodes[3]->GetD(),21,0);
				}

				
				/// Computes the STIFFNESS MATRIX of the element:    
				/// K = integral( .... ),
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed 
	virtual void ComputeStiffnessMatrix()
				{	
					
					bool use_numerical_differentiation = false;
					//bool use_numerical_differentiation = true;

					if (use_numerical_differentiation)
					{
						
					}else{
						///  Recover stored Jacobian
						ChMatrixNM<double, 24,24>  stock_KTE_elem;
						stock_KTE_elem=this->GetStockKTE();
						StiffnessMatrix = stock_KTE_elem;

						//GetLog() << "stock_KTE_elem" << stock_KTE_elem;
						//system("pause");

						/// Recover stored EAS Jacobian
						ChMatrixNM<double, 24,24>  stock_jac_EAS_elem;
						stock_jac_EAS_elem=this->GetStockJac();
						StiffnessMatrix -= stock_jac_EAS_elem;
						
						//GetLog() << "stock_jac_EAS_elem" << stock_jac_EAS_elem;
						//system("pause");

					}

				}

				/// Computes the MASS MATRIX of the element
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed 
	virtual void ComputeMassMatrix()
				{	

					ChMatrixNM<double, 24,1> InitialCoord;
					ChMatrixNM<double, 8,3>   d0;
					InitialCoord=this->GetInitialPosD();
					d0(0,0) = InitialCoord(0,0);	d0(0,1) = InitialCoord(1,0);	d0(0,2) = InitialCoord(2,0);
					d0(1,0) = InitialCoord(3,0);	d0(1,1) = InitialCoord(4,0);	d0(1,2) = InitialCoord(5,0);
					d0(2,0) = InitialCoord(6,0);	d0(2,1) = InitialCoord(7,0);	d0(2,2) = InitialCoord(8,0);
					d0(3,0) = InitialCoord(9,0);	d0(3,1) = InitialCoord(10,0);	d0(3,2) = InitialCoord(11,0);
					d0(4,0) = InitialCoord(12,0);	d0(4,1) = InitialCoord(13,0);	d0(4,2) = InitialCoord(14,0);
					d0(5,0) = InitialCoord(15,0);	d0(5,1) = InitialCoord(16,0);	d0(5,2) = InitialCoord(17,0);
					d0(6,0) = InitialCoord(18,0);	d0(6,1) = InitialCoord(19,0);	d0(6,2) = InitialCoord(20,0);
					d0(7,0) = InitialCoord(21,0);	d0(7,1) = InitialCoord(22,0);	d0(7,2) = InitialCoord(23,0);

					ChMatrixNM<double, 98,1> InertFlexVec1;
					InertFlexVec1=GetInertFlexVec();
					ChMatrixNM<double, 7,2> GaussZRange1;
					GaussZRange1=GetGaussZRange();
					
					ChMatrixNM<double,24,24> TempMassMatrix;
					this->MassMatrix.Reset();
					int NumLayerPerElem = GetNumLayer();
					for (int kl=0;kl<NumLayerPerElem;kl++)
					{
						int ij=14*kl;
						double rho		= InertFlexVec1(ij); 

						/// Integrate  rho*(S'*S)
						/// where S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
						class MyMass : public ChIntegrable3D< ChMatrixNM<double,24,24> >
						{
						public:
							ChElementShellANCF* element;
							ChMatrixNM<double, 8,3> *d0; //// pointer to initial coordinates
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
								// S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
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

								////Matrix Multiplication
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

								// perform  r = S'*S
								result.MatrTMultiply(S,S); 

								// multiply integration weighs
								result *= detJ0*(element->GetLengthX()/2)*(element->GetLengthY()/2)*(element->GetThickness()/2);
							}
						};

						MyMass myformula;
						myformula.d0 = &d0;
						myformula.element = this;
						
						TempMassMatrix.Reset();

						ChQuadrature::Integrate3D< ChMatrixNM<double,24,24> >(
										TempMassMatrix,	// result of integration will go there
										myformula,			// formula to integrate
										-1,					// start of x
										1,					// end of x
										-1,					// start of y
										1,					// end of y
										GaussZRange1(kl,0),					// start of z
										GaussZRange1(kl,1),					// end of z
										2					// order of integration
										);					
						TempMassMatrix *= rho;
						this->MassMatrix += TempMassMatrix;

						//GetLog() << "MassMatrix" << "\n\n";
						//for(int iii=0;iii<24;iii++){
						//	for(int jjj=0;jjj<24;jjj++){
						//		GetLog() << this->MassMatrix(iii,jjj) << "\n";
						//	}
						//	system("pause");
						//}
						

					} // Layer Loop

					//GetLog()<<"this->MassMatrix"<<this->MassMatrix;
					//system("pause");
				}


				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, ex. the mass matrix in ANCF is constant
	virtual void ComputeGravityForce()
				{

					/// Initial nodal coordinates
					ChMatrixNM<double, 24,1> InitialCoord;
					ChMatrixNM<double, 8,3>   d0;
					InitialCoord=this->GetInitialPosD();
					d0(0,0) = InitialCoord(0,0);	d0(0,1) = InitialCoord(1,0);	d0(0,2) = InitialCoord(2,0);
					d0(1,0) = InitialCoord(3,0);	d0(1,1) = InitialCoord(4,0);	d0(1,2) = InitialCoord(5,0);
					d0(2,0) = InitialCoord(6,0);	d0(2,1) = InitialCoord(7,0);	d0(2,2) = InitialCoord(8,0);
					d0(3,0) = InitialCoord(9,0);	d0(3,1) = InitialCoord(10,0);	d0(3,2) = InitialCoord(11,0);
					d0(4,0) = InitialCoord(12,0);	d0(4,1) = InitialCoord(13,0);	d0(4,2) = InitialCoord(14,0);
					d0(5,0) = InitialCoord(15,0);	d0(5,1) = InitialCoord(16,0);	d0(5,2) = InitialCoord(17,0);
					d0(6,0) = InitialCoord(18,0);	d0(6,1) = InitialCoord(19,0);	d0(6,2) = InitialCoord(20,0);
					d0(7,0) = InitialCoord(21,0);	d0(7,1) = InitialCoord(22,0);	d0(7,2) = InitialCoord(23,0);
					
					ChMatrixNM<double, 98,1> InertFlexVec1;
					InertFlexVec1=GetInertFlexVec();
					ChMatrixNM<double, 7,2> GaussZRange1;
					ChMatrixNM<double,24,1> TempGravityForce;
					GaussZRange1=GetGaussZRange();
					int NumLayerPerElem = GetNumLayer();
					for (int kl=0;kl<NumLayerPerElem;kl++)
					{
						
						int ij=14*kl;

						//// Material properties
						double rho	 = InertFlexVec1(ij);
						//Add gravity force
							class MyGravity : public ChIntegrable3D< ChMatrixNM<double,24,1> >
							{
							public:
								ChElementShellANCF* element;
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
									double wx2 = (element->GetLengthX())/2.0;
									double wy2 = (element->GetLengthY())/2.0;
									double wz2 = (element->GetThickness())/2.0;

									//Set gravity acceleration
									if(element->GetFlagGravity()==0){
									LocalGravityForce(0,0) = 0.0;
									LocalGravityForce(1,0) = 0.0;
									LocalGravityForce(2,0) = 0.0;
									}else if(element->GetFlagGravity()==1){
									LocalGravityForce(0,0) = 0.0;
									LocalGravityForce(1,0) = 0.0;
									LocalGravityForce(2,0) = -9.81;
									}

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

									result *= detJ0*wx2*wy2*wz2; // 5/28/2015
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
											GaussZRange1(kl,0),					// start of z
											GaussZRange1(kl,1),					// end of z
											2					// order of integration
											);
						
							Fgravity *= rho;   // 5/28/2015

							//GetLog() << "Fgravity" << "\n\n";
							//for(int iii=0;iii<24;iii++){
							//   GetLog() << Fgravity(iii) << "\n";
							//}
							//system("pause");
							TempGravityForce += Fgravity;
						}
						ChMatrixNM<double,24,1>Fg = TempGravityForce;
						SetGravForce(Fg);
				}

	virtual void SetupInitial() 
				{
					ComputeGravityForce();
					// Compute initial Jacobian
					ChMatrixDynamic<double>Temp;
					ComputeInternalForces(Temp);
					// Compute mass matrix
					ComputeMassMatrix();
					
					// initial EAS parameters
					//stock_jac_EAS.Reset();

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

					int i = this->GetElemNum();

					/// Current nodal coordiantes
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> dA = this->nodes[0]->GetD();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> dB = this->nodes[1]->GetD();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> dC = this->nodes[2]->GetD();
					ChVector<> pD = this->nodes[3]->GetPos();
					ChVector<> dD = this->nodes[3]->GetD();

					/// Current nodal velocity for structural damping
					ChVector<> pA_dt = this->nodes[0]->GetPos_dt();
					ChVector<> dA_dt = this->nodes[0]->GetD_dt();
					ChVector<> pB_dt = this->nodes[1]->GetPos_dt();
					ChVector<> dB_dt = this->nodes[1]->GetD_dt();
					ChVector<> pC_dt = this->nodes[2]->GetPos_dt();
					ChVector<> dC_dt = this->nodes[2]->GetD_dt();
					ChVector<> pD_dt = this->nodes[3]->GetPos_dt();
					ChVector<> dD_dt = this->nodes[3]->GetD_dt();

				    ChMatrixNM<double, 24,1>   d_dt; // for structural damping
					d_dt(0,0) = pA_dt.x;	d_dt(1,0) = pA_dt.y;	d_dt(2,0) = pA_dt.z;
					d_dt(3,0) = dA_dt.x;	d_dt(4,0) = dA_dt.y;	d_dt(5,0) = dA_dt.z;
					d_dt(6,0) = pB_dt.x;	d_dt(7,0) = pB_dt.y;	d_dt(8,0) = pB_dt.z;
					d_dt(9,0) = dB_dt.x;	d_dt(10,0) = dB_dt.y;	d_dt(11,0) = dB_dt.z;
					d_dt(12,0) = pC_dt.x;	d_dt(13,0) = pC_dt.y;	d_dt(14,0) = pC_dt.z;
					d_dt(15,0) = dC_dt.x;	d_dt(16,0) = dC_dt.y;	d_dt(17,0) = dC_dt.z;
					d_dt(18,0) = pD_dt.x;	d_dt(19,0) = pD_dt.y;	d_dt(20,0) = pD_dt.z;
					d_dt(21,0) = dD_dt.x;	d_dt(22,0) = dD_dt.y;	d_dt(23,0) = dD_dt.z;

					ChMatrixNM<double, 8,3>   d;
					d(0,0) = pA.x;	d(0,1) = pA.y;	d(0,2) = pA.z;
					d(1,0) = dA.x;	d(1,1) = dA.y;	d(1,2) = dA.z;
					d(2,0) = pB.x;	d(2,1) = pB.y;	d(2,2) = pB.z;
					d(3,0) = dB.x;	d(3,1) = dB.y;	d(3,2) = dB.z;
					d(4,0) = pC.x;	d(4,1) = pC.y;	d(4,2) = pC.z;
					d(5,0) = dC.x;	d(5,1) = dC.y;	d(5,2) = dC.z;
					d(6,0) = pD.x;	d(6,1) = pD.y;	d(6,2) = pD.z;
					d(7,0) = dD.x;	d(7,1) = dD.y;	d(7,2) = dD.z;

					/// Initial nodal coordinates
					ChMatrixNM<double, 24,1> InitialCoord;
					ChMatrixNM<double, 8,3>   d0;
					InitialCoord=this->GetInitialPosD();
					d0(0,0) = InitialCoord(0,0);	d0(0,1) = InitialCoord(1,0);	d0(0,2) = InitialCoord(2,0);
					d0(1,0) = InitialCoord(3,0);	d0(1,1) = InitialCoord(4,0);	d0(1,2) = InitialCoord(5,0);
					d0(2,0) = InitialCoord(6,0);	d0(2,1) = InitialCoord(7,0);	d0(2,2) = InitialCoord(8,0);
					d0(3,0) = InitialCoord(9,0);	d0(3,1) = InitialCoord(10,0);	d0(3,2) = InitialCoord(11,0);
					d0(4,0) = InitialCoord(12,0);	d0(4,1) = InitialCoord(13,0);	d0(4,2) = InitialCoord(14,0);
					d0(5,0) = InitialCoord(15,0);	d0(5,1) = InitialCoord(16,0);	d0(5,2) = InitialCoord(17,0);
					d0(6,0) = InitialCoord(18,0);	d0(6,1) = InitialCoord(19,0);	d0(6,2) = InitialCoord(20,0);
					d0(7,0) = InitialCoord(21,0);	d0(7,1) = InitialCoord(22,0);	d0(7,2) = InitialCoord(23,0);

					/// Material properties
					ChMatrixNM<double, 98,1> InertFlexVec1;
					InertFlexVec1=GetInertFlexVec();
					ChMatrixNM<double, 35,1> StockAlpha1;
					StockAlpha1=GetStockAlpha();
					//GetLog() << "StockAlpha1_Bef" << "\n";
					//for(int ii=0;ii<35;ii++){
					//	GetLog() << StockAlpha1(ii) << "\n";
					//}
					//system("pause");
					//GetLog() << "StockAlpha1" << StockAlpha1 << "\n\n";
					ChMatrixNM<double, 7,2> GaussZRange1;
					GaussZRange1=GetGaussZRange();

				    ChMatrixNM<double,24,1> TempInternalForce;
					ChMatrixNM<double,24,24> TempJacobian;
					ChMatrixNM<double,24,24> TempJacobian_EAS;
					//TempJacobian_EAS.Reset();
					ChMatrixNM<double, 24,24> stock_jac_EAS_elem1; // laminate structure
					ChMatrixNM<double, 24,24> KTE1; // Laminate structure

					//this->.Reset();
					Fi.Reset();
					stock_jac_EAS_elem1.Reset();
					KTE1.Reset();
					
					int NumLayerPerElem = GetNumLayer();
					TempInternalForce.Reset();
					for (int kl=0;kl<NumLayerPerElem;kl++)
					{
						
						TempJacobian_EAS.Reset();
						TempJacobian.Reset();

						//int j=0;
						int ij=14*kl;

						//// MAterial properties
						double rho	 = InertFlexVec1(ij); //Material->Get_density();
						double theta = InertFlexVec1(ij+4)*3.14159265358979323846264338327950288/180.0; // Fiber angle (rad)
						double Ex = InertFlexVec1(ij+5); //Material->Get_Ex();
						double Ey = InertFlexVec1(ij+6); //Material->Get_Ey();
						double Ez = InertFlexVec1(ij+7); //Material->Get_Ez();
						double vx = InertFlexVec1(ij+8); //Material->Get_vx();
						double vy = InertFlexVec1(ij+9); //Material->Get_vy();
						double vz = InertFlexVec1(ij+10); //Material->Get_vz();
						double Gx = InertFlexVec1(ij+11); //Material->Get_Gx();
						double Gy = InertFlexVec1(ij+12); //Material->Get_Gy();
						double Gz = InertFlexVec1(ij+13); //Material->Get_Gz();

						//GetLog() << kl <<"\n\n";
						//for(int ijkll=0;ijkll<98;ijkll++){
						//	GetLog() <<InertFlexVec1(ijkll) <<"\n";
						//}
						//system("pause");

						//// Cauchy-Green Tensor Calculation						
						ChMatrixNM<double, 6,6> E_eps;
						double CCOM[12];

						CCOM[0]=Ex; //Ex
						CCOM[1]=Ey; //Ey
						CCOM[2]=Ez; //Ez
						CCOM[3]=vx; //Nuxy
						CCOM[4]=vy; //Nuxz
						CCOM[5]=vz; //Nuyz
						CCOM[6]=CCOM[3]*CCOM[1]/CCOM[0];   //!Nuyx
						CCOM[7]=CCOM[4]*CCOM[2]/CCOM[0];  //!Nuzx
						CCOM[8]=CCOM[5]*CCOM[2]/CCOM[1];  //!Nuzy
						CCOM[9]=Gx; //Gxy
						CCOM[10]=Gy; //Gxz
						CCOM[11]=Gz; //Gyz
						double DELTA = 1.0-(CCOM[3]*CCOM[3])*CCOM[1]/CCOM[0]-(CCOM[4]*CCOM[4])*CCOM[2]/CCOM[0]-(CCOM[5]*CCOM[5])*CCOM[2]/CCOM[1]-2.0*CCOM[3]*CCOM[4]*CCOM[5]*CCOM[2]/CCOM[0];
						E_eps(0,0)=CCOM[0]*(1.0-(CCOM[5]*CCOM[5])*CCOM[2]/CCOM[1])/DELTA;
						E_eps(1,1)=CCOM[1]*(1.0-(CCOM[4]*CCOM[4])*CCOM[2]/CCOM[0])/DELTA;
						E_eps(3,3)=CCOM[2]*(1.0-(CCOM[3]*CCOM[3])*CCOM[1]/CCOM[0])/DELTA;
						E_eps(0,1)=CCOM[1]*(CCOM[3]+CCOM[4]*CCOM[5]*CCOM[2]/CCOM[1])/DELTA;
						E_eps(0,3)=CCOM[2]*(CCOM[4]+CCOM[5]*CCOM[3]                )/DELTA;
						E_eps(1,0)=CCOM[1]*(CCOM[3]+CCOM[4]*CCOM[5]*CCOM[2]/CCOM[1])/DELTA;
						E_eps(1,3)=CCOM[2]*(CCOM[5]+CCOM[4]*CCOM[3]*CCOM[1]/CCOM[0])/DELTA;
						E_eps(3,0)=CCOM[2]*(CCOM[4]+CCOM[5]*CCOM[3]                )/DELTA;
						E_eps(3,1)=CCOM[2]*(CCOM[5]+CCOM[4]*CCOM[3]*CCOM[1]/CCOM[0])/DELTA;
						E_eps(2,2)=CCOM[9];
						E_eps(4,4)=CCOM[10];
						E_eps(5,5)=CCOM[11];


						/// If numerical differentiation is used, only the internal force and EAS stiffness
						/// will be calculated. If the numerical differentiation is not used, the jacobian
						/// will also be calculated.
						bool use_numerical_differentiation = false;
					
						/// Internal force and EAS parameters are caulculated for numerical differentiation. 
						if(use_numerical_differentiation){
							
						}else{

						///==========================================================================================================
						///============ Internal force, EAS stiffness, and analytical jacobian are calculated =======================
						///==========================================================================================================

							class MyForce : public ChIntegrable3D< ChMatrixNM<double,750,1> >
							{
							public:
								ChElementShellANCF* element;
								//// External values
								ChMatrixNM<double, 8,3>  *d;
								//ChMatrixNM<double, 8,3> *v;
								ChMatrixNM<double, 8,1>  *strain_ans; 
								ChMatrixNM<double, 8,24>  *strainD_ans; 
								ChMatrixNM<double, 8,3>  *d0;
								ChMatrixNM<double, 24,1>  *d_dt; // for structural damping
								ChMatrixNM<double, 6,6>  *T0;
								ChMatrixNM<double, 5,1>  *alpha_eas;
								ChMatrixNM<double, 6,6>  *E_eps;
								double *detJ0C;
								double *theta;
						
								ChMatrixNM<double, 24,1>  Fint;
								ChMatrixNM<double, 24,24>  JAC11;
								ChMatrixNM<double, 9,24>  Gd;
								ChMatrixNM<double, 6,1>  stress;
								ChMatrixNM<double, 9,9>  Sigm;
								ChMatrixNM<double, 24,6>  temp246;
								ChMatrixNM<double, 24,9>  temp249;
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
								ChMatrixNM<double, 8,8>   d0_d0;
								ChMatrixNM<double, 8,1>   d0d0Nx;
								ChMatrixNM<double, 8,1>   d0d0Ny;
								ChMatrixNM<double, 8,1>   d0d0Nz;
								ChMatrixNM<double, 1,3>  Nxd;
								ChMatrixNM<double, 1,3>  Nyd;
								ChMatrixNM<double, 1,3>  Nzd;
								ChMatrixNM<double, 1,1>   tempA;
								ChMatrixNM<double, 1,1>   tempA1;
								ChMatrixNM<double, 1,24>  tempB;
								ChMatrixNM<double, 24,6>  tempC;
								double detJ0;
								//double dt;
								double alphaHHT;
								double betaHHT;
								double gammaHHT;
								// ANS
								ChMatrixNM<double, 1,8>   N;
								ChMatrixNM<double, 1,4>   S_ANS;
								ChMatrixNM<double, 1,24>  tempBB;
								//EAS
								ChMatrixNM<double, 6,5>   M;
								ChMatrixNM<double, 6,5>   G;
								ChMatrixNM<double, 5,6>   GT;
								ChMatrixNM<double, 6,1> strain_EAS;



								/// Evaluate (strainD'*strain)  at point x 
								virtual void Evaluate(ChMatrixNM<double,750,1>& result, const double x, const double y, const double z)
								{
									//GetLog() << "X,Y,Z, " << x << y << z << "\n\n";
									element->ShapeFunctions(N, x, y, z); // ANS used for ZZ strain and strainD
									element->ShapeFunctionsDerivativeX(Nx, x, y, z);
									element->ShapeFunctionsDerivativeY(Ny, x, y, z);
									element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
							
									element->shapefunction_ANS_BilinearShell(S_ANS, x, y);
									element->Basis_M(M,x,y,z); // EAS

									alphaHHT=-0.2;
									betaHHT=0.25*(1.0-alphaHHT)*(1.0-alphaHHT);
									gammaHHT=0.5-alphaHHT;

									//GetLog() << alphaHHT <<"\n"<<betaHHT<<"\n"<<gammaHHT<<"\n";
									//system("pause");

									// Expand shape function derivatives in 3x24 matrices
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
									ChMatrixNM<double, 1,3>   temp33;
									temp33.Reset();
									temp33=(Nx*(*d0));
									temp33.MatrTranspose();
									rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,0);
									temp33.MatrTranspose();
									temp33=(Ny*(*d0));
									temp33.MatrTranspose();
									rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,1);
									temp33.MatrTranspose();
									temp33=(Nz*(*d0));
									temp33.MatrTranspose();
									rd0.PasteClippedMatrix(&temp33,0,0,3,1,0,2);
									detJ0=rd0.Det();
									//GetLog() << "rd0 " << rd0 << "\n\n";
									//////////////////////////////////////////////////////////////
									//// Transformation : Orthogonal transformation (A and J) ////
									//////////////////////////////////////////////////////////////
									ChVector<double> G1;
									ChVector<double> G2;
									ChVector<double> G3;
									ChVector<double> G1xG2;
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
									ChVector<double> AA1;
									ChVector<double> AA2;
									ChVector<double> AA3;
									//GetLog() << "theta" << *theta;
									
									AA1 = A1*cos(*theta)+A2*sin(*theta);
									AA2 = -A1*sin(*theta)+A2*cos(*theta);
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
									//tempA=Nz*ddNz;
									//strain_til(3,0) = 0.5*(tempA(0,0)-1.0);
									//tempA=Nx*ddNz;
									//strain_til(4,0) = tempA(0,0);
									//tempA=Ny*ddNz;
									//strain_til(5,0) = tempA(0,0);
									//== Incompatible strain (ANS) ==//
									strain_til(3,0) = N(0,0)*(*strain_ans)(0,0)+N(0,2)*(*strain_ans)(1,0)+N(0,4)*(*strain_ans)(2,0)+N(0,6)*(*strain_ans)(3,0);
									strain_til(4,0) = S_ANS(0,2)*(*strain_ans)(6,0)+S_ANS(0,3)*(*strain_ans)(7,0);
									strain_til(5,0) = S_ANS(0,0)*(*strain_ans)(4,0)+S_ANS(0,1)*(*strain_ans)(5,0);
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
									//tempB = Nz*(*d)*Sz;
									//strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,3,0);
									//tempB = Nx*(*d)*Sz + Nz*(*d)*Sx;
									//strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,4,0);
									//tempB = Ny*(*d)*Sz + Nz*(*d)*Sy;
									//strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,5,0);
									//== Incompatible strain (ANS)==//
									tempBB.Reset();
									for(int i=0;i<4;i++){
										int ij=i*2;
										tempB.PasteClippedMatrix(strainD_ans,i,0,1,24,0,0);
										tempB *= N(0,ij);
										tempBB += tempB;
									}
									strainD_til.PasteClippedMatrix(&tempBB,0,0,1,24,3,0); // strainD for zz
									//
									tempBB.Reset();
									for(int i=0;i<2;i++){
										int ij=i+6;
										int ij1=i+2;
										tempB.PasteClippedMatrix(strainD_ans,ij,0,1,24,0,0);
										tempB *= S_ANS(0,ij1);
										tempBB += tempB;							
									}
									strainD_til.PasteClippedMatrix(&tempBB,0,0,1,24,4,0); // strainD for xz
									//
									tempBB.Reset();
									for(int i=0;i<2;i++){
										int ij=i+4;
										int ij1=i;
										tempB.PasteClippedMatrix(strainD_ans,ij,0,1,24,0,0);
										tempB *= S_ANS(0,ij1);
										tempBB += tempB;							
									}
									strainD_til.PasteClippedMatrix(&tempBB,0,0,1,24,5,0); // strainD for yz
									//// For orthotropic material ///
									for(int ii=0;ii<24;ii++){
										strainD(0,ii)=strainD_til(0,ii)*beta(0)*beta(0)+strainD_til(1,ii)*beta(3)*beta(3)+strainD_til(2,ii)*beta(0)*beta(3)+strainD_til(3,ii)*beta(6)*beta(6)+strainD_til(4,ii)*beta(0)*beta(6)+strainD_til(5,ii)*beta(3)*beta(6);
										strainD(1,ii)=strainD_til(0,ii)*beta(1)*beta(1)+strainD_til(1,ii)*beta(4)*beta(4)+strainD_til(2,ii)*beta(1)*beta(4)+strainD_til(3,ii)*beta(7)*beta(7)+strainD_til(4,ii)*beta(1)*beta(7)+strainD_til(5,ii)*beta(4)*beta(7);
										strainD(2,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(1)+strainD_til(1,ii)*2.0*beta(3)*beta(4)+strainD_til(2,ii)*(beta(1)*beta(3)+beta(0)*beta(4))+strainD_til(3,ii)*2.0*beta(6)*beta(7)+strainD_til(4,ii)*(beta(1)*beta(6)+beta(0)*beta(7))+strainD_til(5,ii)*(beta(4)*beta(6)+beta(3)*beta(7));
										strainD(3,ii)=strainD_til(0,ii)*beta(2)*beta(2)+strainD_til(1,ii)*beta(5)*beta(5)+strainD_til(2,ii)*beta(2)*beta(5)+strainD_til(3,ii)*beta(8)*beta(8)+strainD_til(4,ii)*beta(2)*beta(8)+strainD_til(5)*beta(5)*beta(8);
										strainD(4,ii)=strainD_til(0,ii)*2.0*beta(0)*beta(2)+strainD_til(1,ii)*2.0*beta(3)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(3)+beta(0)*beta(5))+strainD_til(3,ii)*2.0*beta(6)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(6)+beta(0)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(6)+beta(3)*beta(8));
										strainD(5,ii)=strainD_til(0,ii)*2.0*beta(1)*beta(2)+strainD_til(1,ii)*2.0*beta(4)*beta(5)+strainD_til(2,ii)*(beta(2)*beta(4)+beta(1)*beta(5))+strainD_til(3,ii)*2.0*beta(7)*beta(8)+strainD_til(4,ii)*(beta(2)*beta(7)+beta(1)*beta(8))+strainD_til(5,ii)*(beta(5)*beta(7)+beta(4)*beta(8));
									}
									/// Gd (9x24) calculation
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

									///////////////////////////////////
									/// Enhanced Assumed Strain 2nd ///
									///////////////////////////////////
									strain += strain_EAS;

									//////////////////////////////////////
									/// Structural damping (6/23/2015) ///
									//////////////////////////////////////
						/*			ChMatrixNM<double, 6,1> dstrain;
									dstrain.Reset();
									int kk=0;
									for(int ii=0;ii<8;ii++)
									{
										dstrain(0,0)+=(strainD(0,kk)*(*v)(ii,0))+(strainD(0,kk+1)*(*v)(ii,1))+(strainD(0,kk+2)*(*v)(ii,2));
										dstrain(1,0)+=(strainD(1,kk)*(*v)(ii,0))+(strainD(1,kk+1)*(*v)(ii,1))+(strainD(1,kk+2)*(*v)(ii,2));
										dstrain(2,0)+=(strainD(2,kk)*(*v)(ii,0))+(strainD(2,kk+1)*(*v)(ii,1))+(strainD(2,kk+2)*(*v)(ii,2));
										dstrain(3,0)+=(strainD(3,kk)*(*v)(ii,0))+(strainD(3,kk+1)*(*v)(ii,1))+(strainD(3,kk+2)*(*v)(ii,2));
										dstrain(4,0)+=(strainD(4,kk)*(*v)(ii,0))+(strainD(4,kk+1)*(*v)(ii,1))+(strainD(4,kk+2)*(*v)(ii,2));
										dstrain(5,0)+=(strainD(5,kk)*(*v)(ii,0))+(strainD(5,kk+1)*(*v)(ii,1))+(strainD(5,kk+2)*(*v)(ii,2));
										kk=kk+3;
									}
									kk=0;*/
						            /// Strain time derivative for structural damping
									ChMatrixNM<double, 6,1>   DEPS;
									DEPS.Reset();
									for(int ii=0;ii<24;ii++)
									{
										DEPS(0,0)=DEPS(0,0)+strainD(0,ii)*((*d_dt)(ii,0));
										DEPS(1,0)=DEPS(1,0)+strainD(1,ii)*((*d_dt)(ii,0));
										DEPS(2,0)=DEPS(2,0)+strainD(2,ii)*((*d_dt)(ii,0));
										DEPS(3,0)=DEPS(3,0)+strainD(3,ii)*((*d_dt)(ii,0));
										DEPS(4,0)=DEPS(4,0)+strainD(4,ii)*((*d_dt)(ii,0));
										DEPS(5,0)=DEPS(5,0)+strainD(5,ii)*((*d_dt)(ii,0));
									}
									//dstrain=dstrain*(element->GetAlphaDamp());
									double DampCoefficient = gammaHHT/(betaHHT*element->Getdt());//dt*gammaHHT;
									//GetLog() << DampCoefficient << "\n";
									//if((*v)(0,0)!=0.0){
									//GetLog()<<dstrain<<"\n"<<strain<<"\n"<<*v<<"\n";
									//system("pause");
									//strain+=dstrain*(element->GetAlphaDamp());
									///////////////////////////////////
									/// Add structural damping      ///
									///////////////////////////////////
									double stdamp=element->GetAlphaDamp();
									DEPS*=stdamp;
									strain +=  DEPS;
									
									/// Stress tensor calculation
									stress.MatrMultiply(*E_eps,strain);
									Sigm(0,0)=stress(0,0); //XX
									Sigm(1,1)=stress(0,0);
									Sigm(2,2)=stress(0,0);

									Sigm(0,3)=stress(2,0); //XY
									Sigm(1,4)=stress(2,0);
									Sigm(2,5)=stress(2,0);

									Sigm(0,6)=stress(4,0); //XZ
									Sigm(1,7)=stress(4,0);
									Sigm(2,8)=stress(4,0);

									Sigm(3,0)=stress(2,0); //XY
									Sigm(4,1)=stress(2,0);
									Sigm(5,2)=stress(2,0);

									Sigm(3,3)=stress(1,0); //YY
									Sigm(4,4)=stress(1,0);
									Sigm(5,5)=stress(1,0);

									Sigm(3,6)=stress(5,0); //YZ
									Sigm(4,7)=stress(5,0);
									Sigm(5,8)=stress(5,0);

									Sigm(6,0)=stress(4,0); //XZ
									Sigm(7,1)=stress(4,0);
									Sigm(8,2)=stress(4,0);

									Sigm(6,3)=stress(5,0); //YZ
									Sigm(7,4)=stress(5,0);
									Sigm(8,5)=stress(5,0);

									Sigm(6,6)=stress(3,0); //ZZ
									Sigm(7,7)=stress(3,0);
									Sigm(8,8)=stress(3,0);

									/// Jacobian calculation ///
									temp246.MatrTMultiply(strainD,*E_eps);
									temp249.MatrTMultiply(Gd,Sigm);
									JAC11=(temp246*strainD*(1.0+DampCoefficient*(element->GetAlphaDamp())))+temp249*Gd;
									JAC11*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetThickness()/2.0);
									/// Internal force calculation ///
									tempC.MatrTMultiply(strainD,*E_eps);
									Fint.MatrMultiply(tempC,strain);
									Fint *= detJ0*(element->GetLengthX()/2)*(element->GetLengthY()/2)*(element->GetThickness()/2);

//									GetLog() << "G(jj,ii)" << "\n\n";
									for(int ii=0;ii<5;ii++){
										for(int jj=0;jj<6;jj++){
											GT(ii,jj)=G(jj,ii);
											//GetLog()  << G(jj,ii) << "\n";
										}
									}
									// for EAS
									ChMatrixNM<double, 5,6>  temp56;
									temp56.MatrMultiply(GT,*E_eps);
									ChMatrixNM<double, 5,1> HE1;
									ChMatrixNM<double, 5,24> GDEPSP;
									ChMatrixNM<double, 5,5>  KALPHA;
									ChMatrixNM<double, 120,1> GDEPSPVec;
									ChMatrixNM<double, 25,1>  KALPHAVec;
									ChMatrixNM<double, 576,1>  JACVec;
									HE1.MatrMultiply(temp56,strain);
									HE1*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetThickness()/2.0);
									GDEPSP.MatrMultiply(temp56,strainD);
									GDEPSP*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetThickness()/2.0);
									KALPHA.MatrMultiply(temp56,G);
									KALPHA*=detJ0*(element->GetLengthX()/2.0)*(element->GetLengthY()/2.0)*(element->GetThickness()/2.0);
									result.Reset();

									for (int i = 0; i < 5;i++){
										for (int j = 0; j < 24; j++){
										GDEPSPVec(i*24+j) = GDEPSP(i,j);
										}
									}
									for (int i = 0; i < 5; i++){
										for (int j = 0; j < 5; j++){
											KALPHAVec(i * 5 + j) = KALPHA(i, j);
										}
									}
									for (int i = 0; i < 24; i++){
										for (int j = 0; j < 24; j++){
											JACVec(i * 24 + j) = JAC11(i, j);
										}
									}

									
									//GetLog() << "GDEPSP" << GDEPSP << "\n\n";
									//GetLog() << "GDEPSPVec" << GDEPSPVec << "\n\n";
									//GetLog() << "KALPHA" << KALPHA << "\n\n";
									//GetLog() << "KALPHAVec" << KALPHAVec << "\n\n";
									//GetLog() << "Fint" << Fint << "\n\n";
									//GetLog() << "HE1" << HE1 << "\n\n";
									//GetLog() << "JAC11" << JAC11 << "\n\n";
									//system("pause");

									/// Total result vector ///
									result.PasteClippedMatrix(&Fint,0,0,24,1,0,0); 
									result.PasteClippedMatrix(&HE1,0,0,5,1,24,0);
									result.PasteClippedMatrix(&GDEPSPVec,0,0,120,1,29,0); 
									result.PasteClippedMatrix(&KALPHAVec,0,0,25,1,149,0); 
									result.PasteClippedMatrix(&JACVec,0,0,576,1,174,0);
								}
							};
							//////////////////////////////////////////////////////////////////////////////////////////////////////////
							ChMatrixNM<double, 750,1> TempIntegratedResult;
							ChMatrixNM<double, 24,1> Finternal;
							// Assumed Natural Strain (ANS)
							ChMatrixNM<double, 8,1>   strain_ans;
							ChMatrixNM<double, 8,24>   strainD_ans;
							// Enhanced Assumed Strain (EAS)
							ChMatrixNM<double, 6,6>   T0;
							ChMatrixNM<double, 5,1>   HE;
							ChMatrixNM<double, 5,24>  GDEPSP;
							ChMatrixNM<double, 5,5>   KALPHA;
							ChMatrixNM<double, 24,24> KTE;
							ChMatrixNM<double, 5,5>   KALPHA1;
							ChMatrixNM<double, 5,1>   ResidHE;
							double detJ0C;
							ChMatrixNM<double, 5,1>   alpha_eas;
							ChMatrixNM<double, 5,1>   renewed_alpha_eas;
							ChMatrixNM<double, 5,1>   previous_alpha;
					
							//previous_alpha=GetStockAlpha();
							int ijkl=kl*5;
							previous_alpha(0,0)=StockAlpha1(ijkl);
							previous_alpha(1,0)=StockAlpha1(ijkl+1);
							previous_alpha(2,0)=StockAlpha1(ijkl+2);
							previous_alpha(3,0)=StockAlpha1(ijkl+3);
							previous_alpha(4,0)=StockAlpha1(ijkl+4);
							alpha_eas = previous_alpha;
							ResidHE.Reset();
							int count=0;
							int fail=1;
							/// Begin EAS loop ///
							while (fail == 1)
							{
								alpha_eas = alpha_eas - ResidHE;
								renewed_alpha_eas = alpha_eas;

								Finternal.Reset();
								HE.Reset();
								GDEPSP.Reset();
								KALPHA.Reset();
								strain_ans.Reset();
								strainD_ans.Reset();

								// Assumed Natural Strain (ANS)
								AssumedNaturalStrain_BilinearShell(d, d0, strain_ans, strainD_ans);

								// Enhanced Assumed Strain (EAS)
								T0.Reset();
								detJ0C = 0.0;
								T0DetJElementCenterForEAS(d0, T0, detJ0C, theta);
								//GetLog() << "T0" << T0 << "\n\n";
								MyForce myformula;
								myformula.d = &d;
								//myformula.v = &v;
								myformula.d_dt = &d_dt; // For Structural Damping
								myformula.strain_ans = &strain_ans;
								myformula.strainD_ans = &strainD_ans;
								myformula.d0 = &d0;
								myformula.E_eps = &E_eps;
								myformula.element = this;
								//EAS
								myformula.T0 = &T0;
								myformula.detJ0C = &detJ0C;
								myformula.theta = &theta;
								myformula.alpha_eas = &alpha_eas;
								ChQuadrature::Integrate3D< ChMatrixNM<double, 750, 1> >(
									TempIntegratedResult,				// result of integration will go there
									myformula,			// formula to integrate
									-1,					// start of x
									1,					// end of x
									-1,					// start of y
									1,					// end of y
									GaussZRange1(kl, 0),					// start of z
									GaussZRange1(kl, 1),					// end of z
									2					// order of integration
									);

								///===============================================================//
								///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
								///===TempIntegratedResult(24:28,1) -> HE(5x1)           =========//
								///===TempIntegratedResult(29:148,1) -> GDEPSP(5x24)     =========//
								///===TempIntegratedResult(149:173,1) -> KALPHA(5x5)     =========//
								///===TempIntegratedResult(174:749,1) -> Stiffness Matrix(24x24) =//
								///===============================================================//
								ChMatrixNM<double, 120, 1> GDEPSPvec;
								ChMatrixNM<double, 25, 1> KALPHAvec;
								ChMatrixNM<double, 576, 1> JACvec;
								/// Storing result vector ///
								Finternal.PasteClippedMatrix(&TempIntegratedResult, 0, 0, 24, 1, 0, 0); // 
								HE.PasteClippedMatrix(&TempIntegratedResult, 24, 0, 5, 1, 0, 0); // 
								GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult, 29, 0, 120, 1, 0, 0); //
								KALPHAvec.PasteClippedMatrix(&TempIntegratedResult, 149, 0, 25, 1, 0, 0); //
								JACvec.PasteClippedMatrix(&TempIntegratedResult, 174, 0, 576, 1, 0, 0); //
								{
								for (int i = 0; i < 5; i++){
									for (int j = 0; j < 24; j++){
										GDEPSP(i, j) = GDEPSPvec(i * 24 + j);
									}
								}
								for (int i = 0; i < 5; i++){
									for (int j = 0; j < 5; j++){
										KALPHA(i, j) = KALPHAvec(i * 5 + j);
									}
								}
								for (int i = 0; i < 24; i++){
									for (int j = 0; j < 24; j++){
										TempJacobian(i, j) = JACvec(i * 24 + j);
									}
								}
							}

								// GDEPSP=GDEPSPvec;
								// KALPHA=KALPHAvec;
								// TempJacobian = JACvec; // For Laminate shell

								//GetLog() << "GDEPSP" << GDEPSP << "\n\n";
								//GetLog() << "KALPHA" << KALPHA << "\n\n";
								//GetLog() << "TempJacobian" << TempJacobian << "\n\n";
								//GetLog() << "HE" << HE << "\n\n";
								//GetLog() << "Finternal" << Finternal << "\n\n";
								//system("pause");

								KALPHA1=KALPHA;
								if(flag_HE==1) break; // When numerical jacobian loop, no need to calculate HE
								count = count + 1;
								double norm_HE = HE.NormTwo();
								//GetLog() << i << "  count " << count << "  NormHE "  << norm_HE << "\n";
								if(norm_HE < 0.00001) 
								{fail=0;
								}else{
								ChMatrixNM<int, 5,1> INDX;
								double DAMMY = 0.0;
								ResidHE=HE;
								LUDCMP55(KALPHA1,5,5,INDX,DAMMY);
								LUBKSB55(KALPHA1,5,5,INDX,ResidHE);
								}
								if(flag_HE==0&&count>2){
								GetLog() << i << "  count " << count << "  NormHE "  << norm_HE << "\n";
								}
							}
							//Fi = -Finternal;
							TempInternalForce += -Finternal;
							//===============================//
							//== Stock_Alpha=================//
							//===============================//
							if(flag_HE==0){
								int ijkl=kl*5;
								StockAlpha1(ijkl)=renewed_alpha_eas(0,0);
								StockAlpha1(ijkl+1)=renewed_alpha_eas(1,0);
								StockAlpha1(ijkl+2)=renewed_alpha_eas(2,0);
								StockAlpha1(ijkl+3)=renewed_alpha_eas(3,0);
								StockAlpha1(ijkl+4)=renewed_alpha_eas(4,0);
								if(kl==NumLayerPerElem-1){
								  SetStockAlpha(StockAlpha1); //this->
								  //for(int ii=0;ii<35;ii++){
								  //  GetLog() << "StockAlpha1" << StockAlpha1(ii) << "\n";
								  //}
								}
							}
							//===============================//
							//== Jacobian Matrix for alpha ==//
							//===============================//
							if(flag_HE==0){
								ChMatrixNM<double, 5,5> INV_KALPHA;
								ChMatrixNM<double, 5,24> TEMP_GDEPSP;
								ChMatrixNM<double, 5,5> INV_KALPHA_Temp;
								Inverse55(INV_KALPHA,KALPHA);

								TEMP_GDEPSP.MatrMultiply(INV_KALPHA,GDEPSP);
								TempJacobian_EAS.MatrTMultiply(GDEPSP,TEMP_GDEPSP);
								
								stock_jac_EAS_elem1+=TempJacobian_EAS; 
								KTE1+=TempJacobian; 

								if(kl==NumLayerPerElem-1){
								  this->SetStockJac(stock_jac_EAS_elem1);
								  this->SetStockKTE(KTE1);
								}

							}
						}

						

						if(kl==0)
						{
							
							//Add Tire Air Pressure force
							class MyAirPressure : public ChIntegrable2D< ChMatrixNM<double,24,1> >
							{
							public:
								ChElementShellANCF* element;
								ChMatrixNM<double, 8,3> *d0;
								ChMatrixNM<double, 8,3> *d;
								ChMatrixNM<double, 3,24> S;
								ChMatrixNM<double, 1,4>   S_ANS;
								ChMatrixNM<double, 1,8> N;
								ChMatrixNM<double, 1,8> Nx;
								ChMatrixNM<double, 1,8> Ny;
								ChMatrixNM<double, 1,8> Nz;
								ChMatrixNM<double, 3,1> LocalAirPressure;

								virtual void Evaluate(ChMatrixNM<double,24,1>& result, const double x, const double y)
								{
									double z=0.0;
									element->ShapeFunctions(N, x, y, z);
									element->ShapeFunctionsDerivativeX(Nx, x, y, z);
									element->ShapeFunctionsDerivativeY(Ny, x, y, z);
									element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
							
									// Weights for Gaussian integration
									double wx2 = (element->GetLengthX())/2.0;
									double wy2 = (element->GetLengthY())/2.0;

									//Set Air Pressure
									double Pressure0;
									if(element->GetAirPressure()==0){
									    Pressure0=0000.0; // 220 KPa
									}else if(element->GetAirPressure()==1){
										Pressure0=220.0*1000.0; // 220 KPa
									}
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

									ChMatrixNM<double, 1,3> Nx_d;
									Nx_d.MatrMultiply(Nx,*d);

									ChMatrixNM<double, 1,3> Ny_d;
									Ny_d.MatrMultiply(Ny,*d);

									ChMatrixNM<double, 1,3> Nz_d;
									Nz_d.MatrMultiply(Nz,*d);

									ChMatrixNM<double, 3,3> rd;
									rd(0,0) = Nx_d(0,0); rd(1,0) = Nx_d(0,1); rd(2,0) = Nx_d(0,2);
									rd(0,1) = Ny_d(0,0); rd(1,1) = Ny_d(0,1); rd(2,1) = Ny_d(0,2);
									rd(0,2) = Nz_d(0,0); rd(1,2) = Nz_d(0,1); rd(2,2) = Nz_d(0,2);

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

									ChMatrixNM<double, 3,1> G1xG2;
									G1xG2(0)=rd(1,0)*rd(2,1)-rd(2,0)*rd(1,1);
									G1xG2(1)=rd(2,0)*rd(0,1)-rd(0,0)*rd(2,1);
									G1xG2(2)=rd(0,0)*rd(1,1)-rd(1,0)*rd(0,1);
									double G1xG2norm=sqrt(G1xG2(0)*G1xG2(0)+G1xG2(1)*G1xG2(1)+G1xG2(2)*G1xG2(2));

									LocalAirPressure = -G1xG2*(Pressure0/G1xG2norm);
									result.MatrTMultiply(S,LocalAirPressure);

									result *= detJ0*wx2*wy2; // 6/12/2015
								}
							};

							MyAirPressure myformula2;
							myformula2.d0 = &d0;
							myformula2.d = &d;
							myformula2.element = this;

							ChMatrixNM<double, 24,1> Fpressure;
							ChQuadrature::Integrate2D< ChMatrixNM<double,24,1> >(
											Fpressure,				// result of integration will go there
											myformula2,			// formula to integrate
											-1,					// start of x
											1,					// end of x
											-1,					// start of y
											1,					// end of y
											2					// order of integration
											);
							
							//GetLog() << "Fpressure" << "\n\n";
							//for(int iii=0;iii<24;iii++){
							//   GetLog() << Fpressure(iii) << "\n";
							//}
							//system("pause");
							TempInternalForce += Fpressure;
						}
					} // Layer Loop
					TempInternalForce += this->GetGravForce();
					Fi = TempInternalForce;

				}

	// [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    virtual void shapefunction_ANS_BilinearShell(ChMatrixNM<double,1,4>& S_ANS, double x, double y)
				{
					S_ANS(0,0) = -0.5 * x + 0.5;
					S_ANS(0,1) = 0.5 * x + 0.5;
					S_ANS(0,2) = -0.5 * y + 0.5;
					S_ANS(0,3) = 0.5 * y + 0.5;
				}

	// [ANS] Calculation of ANS strain and strainD
    virtual void AssumedNaturalStrain_BilinearShell(ChMatrixNM<double,8,3>& d,ChMatrixNM<double,8,3>& d0, ChMatrixNM<double,8,1>& strain_ans,ChMatrixNM<double,8,24>& strainD_ans)
				{
					ChMatrixNM<double,8,3> temp_knot;
					temp_knot.Reset();
					temp_knot(0,0)=-1.0;
					temp_knot(0,1)=-1.0;
					temp_knot(1,0)=1.0;
					temp_knot(1,1)=-1.0;
					temp_knot(2,0)=-1.0;
					temp_knot(2,1)=1.0;
					temp_knot(3,0)=1.0;
					temp_knot(3,1)=1.0;
    
					temp_knot(4,0)=-1.0; //A
					temp_knot(4,1)=0.0;  //A
					temp_knot(5,0)=1.0;  //B
					temp_knot(5,1)=0.0;  //B
    
					temp_knot(6,0)=0.0;   //C
					temp_knot(6,1)=-1.0; //C
					temp_knot(7,0)=0.0;  //D
					temp_knot(7,1)=1.0;   //D

					ChElementShellANCF* element;
					ChMatrixNM<double, 3,24>  Sx;
					ChMatrixNM<double, 3,24>  Sy;
					ChMatrixNM<double, 3,24>  Sz;
					ChMatrixNM<double, 1,8>   Nx;
					ChMatrixNM<double, 1,8>   Ny;
					ChMatrixNM<double, 1,8>   Nz;
					ChMatrixNM<double, 8,8>   d_d;
					ChMatrixNM<double, 8,1>   ddNx;
					ChMatrixNM<double, 8,1>   ddNy;
					ChMatrixNM<double, 8,1>   ddNz;
					ChMatrixNM<double, 8,8>   d0_d0;
					ChMatrixNM<double, 8,1>   d0d0Nx;
					ChMatrixNM<double, 8,1>   d0d0Ny;
					ChMatrixNM<double, 8,1>   d0d0Nz;
					ChMatrixNM<double, 1,3>  Nxd;
					ChMatrixNM<double, 1,3>  Nyd;
					ChMatrixNM<double, 1,3>  Nzd;
					ChMatrixNM<double, 1,1>   tempA;
					ChMatrixNM<double, 1,1>   tempA1;
					ChMatrixNM<double, 1,24>  tempB;

					for(int kk=0;kk<8;kk++){

						ShapeFunctionsDerivativeX(Nx, temp_knot(kk,0), temp_knot(kk,1), temp_knot(kk,2));
						ShapeFunctionsDerivativeY(Ny, temp_knot(kk,0), temp_knot(kk,1), temp_knot(kk,2));
						ShapeFunctionsDerivativeZ(Nz, temp_knot(kk,0), temp_knot(kk,1), temp_knot(kk,2));

						// Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
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

						d_d.MatrMultiplyT(d,d);
						ddNx.MatrMultiplyT(d_d,Nx);
						ddNy.MatrMultiplyT(d_d,Ny);
						ddNz.MatrMultiplyT(d_d,Nz);

						d0_d0.MatrMultiplyT(d0,d0);
						d0d0Nx.MatrMultiplyT(d0_d0,Nx);
						d0d0Ny.MatrMultiplyT(d0_d0,Ny);
						d0d0Nz.MatrMultiplyT(d0_d0,Nz);

						if(kk==0||kk==1||kk==2||kk==3){
						    tempA= Nz*ddNz;
							tempA1= Nz*d0d0Nz;
							strain_ans(kk,0) = 0.5*(tempA(0,0)-tempA1(0,0));
							tempB = Nz*(d)*Sz;
     						strainD_ans.PasteClippedMatrix(&tempB,0,0,1,24,kk,0);
						}
						if(kk==4||kk==5){   //kk=4,5 =>yz
							tempA=Ny*ddNz;
							tempA1=Ny*d0d0Nz;
						    strain_ans(kk,0) = tempA(0,0)-tempA1(0,0);
							tempB = Ny*(d)*Sz + Nz*(d)*Sy;
     						strainD_ans.PasteClippedMatrix(&tempB,0,0,1,24,kk,0);
						}
						if(kk==6||kk==7){   //kk=6,7 =>xz
							tempA=Nx*ddNz;
							tempA1=Nx*d0d0Nz;
						    strain_ans(kk,0) = tempA(0,0)-tempA1(0,0);
							tempB = Nx*(d)*Sz + Nz*(d)*Sx;
     						strainD_ans.PasteClippedMatrix(&tempB,0,0,1,24,kk,0);
						}
	
					}

				}

	// [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    virtual void T0DetJElementCenterForEAS(ChMatrixNM<double,8,3>& d0,ChMatrixNM<double,6,6>& T0, double& detJ0C, double &theta)
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
					ChMatrixNM<double, 1,3>   tempA;
					tempA.Reset();
					ShapeFunctionsDerivativeX(Nx, x, y, z);
					ShapeFunctionsDerivativeY(Ny, x, y, z);
					ShapeFunctionsDerivativeZ(Nz, x, y, z);
					tempA=(Nx*d0);
					tempA.MatrTranspose();
					rd0.PasteClippedMatrix(&tempA,0,0,3,1,0,0);
					tempA.MatrTranspose();					
					tempA=(Ny*d0);
					tempA.MatrTranspose();
					rd0.PasteClippedMatrix(&tempA,0,0,3,1,0,1);
					tempA.MatrTranspose();					
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
					double G1dotG1;
					//G1.PasteClippedMatrix(&rd0,0,0,3,1,0,0);
					//G2.PasteClippedMatrix(&rd0,0,1,3,1,0,0);
					G1(0)=rd0(0,0); G2(0)=rd0(0,1); G3(0)=rd0(0,2);
					G1(1)=rd0(1,0); G2(1)=rd0(1,1); G3(1)=rd0(1,2);
					G1(2)=rd0(2,0); G2(2)=rd0(2,1); G3(2)=rd0(2,2);
					G1xG2.Cross(G1,G2);
					G1dotG1=Vdot(G1,G1);

					////Tangent Frame
					ChVector<double> A1;
					ChVector<double> A2;
					ChVector<double> A3;
					//A1=G1.Normalize();
					A1=G1/sqrt(G1(0)*G1(0)+G1(1)*G1(1)+G1(2)*G1(2));
					A3=G1xG2/sqrt(G1xG2(0)*G1xG2(0)+G1xG2(1)*G1xG2(1)+G1xG2(2)*G1xG2(2));
					A2.Cross(A3,A1);
					//double theta = 0.0;
					//if(NonlinearMaterialFlag==0){theta = 0.0;}
					//if(NonlinearMaterialFlag==2){theta = INRTAFlex[i][kl][4] * PI/180.0;}
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
					//j01(0)=j0(0,0); j02(0)=j0(0,1); j03(0)=j0(0,2);
					//j01(1)=j0(1,0); j02(1)=j0(1,1); j03(1)=j0(1,2);
					//j01(2)=j0(2,0); j02(2)=j0(2,1); j03(2)=j0(2,2);
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

////Temporary function of inverse
	virtual void  INV (ChMatrixNM<double,5,5> &a,int n){       // Invert matrix by Gauss method

			// - - - Local Variables - - -
		    ChMatrixNM<double,5,5> b;
			double c;
			double d;
			double preValue = 0.0;
			ChMatrixNM<double,5,1> temp;
			int m;
			int count;
			ChMatrixNM<int,1,1> imax;
			ChMatrixNM<int,5,1> ipvt;
			// - - - - - - - - - - - - - -

			b = a;
			for (int i=0;i<n;i++){
				ipvt(i) = i;
			}

			for(int k=0;k<n;k++){
			   //imax = MAXLOC(ABS(b(k:n,k)))
			   count=0;
			   preValue=0.0;
			   for(int ii=k;ii<n;ii++){
				   if(preValue<abs(b(ii,k))){
					   preValue=abs(b(ii,k));
				       count=count+1;
				   }
				   imax(1)=count+k;
			   }
				m = k-1+imax(1);

			   if(m != k){
				  int temp_ipvt = ipvt( m );
                  ipvt( m )=ipvt( k ); // ipvt( (/m,k/) ) = ipvt( (/k,m/) )
				  ipvt( k )=temp_ipvt;
				  for (int ii=0;ii<n;ii++){ 
					double temp_b = b((m),ii);  //b((/m,k/),:) = b((/k,m/),:)
					b(m,ii)=b(k,ii);
					b(k,ii)=temp_b;
				  }
			   }
			   d = 1/b(k,k);
			   for(int ii=0;ii<n;ii++){
			     temp(ii) = b(ii,k);
			   }
			   for(int j = 0;j< n;j++){
				  c = b(k,j)*d;
				  for(int ii=0;ii<n;ii++){
				    b(ii,j) = b(ii,j)-temp(ii)*c;
				  }
				  b(k,j) = c;
			   }
			   for(int ii=0;ii<n;ii++){
			     b(ii,k) = temp(ii)*(-d);
			   }
			   b(k,k) = d;
			}
			for(int ii=0;ii<n;ii++){
			  for(int jj=0;jj<n;jj++){
			    a(ii,ipvt(jj)) = b(ii,jj);
			  }
			}
		}
	//// Temporary function of LU decomposition 1
	//virtual void LUBKSB55(ChMatrixNM<double,5,5>&A,double N,double NP,ChMatrixNM<int,5,1>& INDX,ChMatrixNM<double,5,1>& B){
	//		int II=0;
	//		for (int I=0;I<N;I++){
	//		  int LL=INDX(I);
	//		  double SUM=B(LL);
	//		  B(LL)=B(I);
	//		  if(II!=0){
	//		  for (int J=II;J<I-1;J++){
	//		    SUM=SUM-A(I,J)*B(J);
	//		  }
	//		  }else if(SUM!=0.0){
	//		    II=I;                
	//		  }
	//		  B(I)=SUM;
	//		}
	//		
	//	    for(int I=N-1;I>-1;I--){
	//		  double SUM=B(I);
	//		 // if(I<N-1){
	//		 //   for( int J=I+1;J<N-1;J++){
	//		 //      SUM=SUM-A(I,J)*B(J);
	//			//}
	//		 // }
	//		  if(I<N){
	//		    for( int J=I+1;J<N;J++){
	//		       SUM=SUM-A(I,J)*B(J);
	//			}
	//		  }
	//		B(I)=SUM/A(I,I);
	//		}
	//}
	virtual void LUBKSB55(ChMatrixNM<double,5,5>&A,int N,int NP,ChMatrixNM<int,5,1>& INDX,ChMatrixNM<double,5,1>& B){
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
	// Temporary function of LU decomposition 2
	//virtual void LUDCMP55(ChMatrixNM<double,5,5>&A,double N,double NP,ChMatrixNM<int,5,1>& INDX,double D){
 //     int NMAX=3000;
	//  double TINY1=1.0e-20;
	//  ChMatrixNM<double,3000,1> VV; 
 //     D=1.0;
 //     for(int I=0;I<N;I++){
 //       double AAMAX=0.0;
 //       for(int J=0;J<N;J++){
	//		if((abs(A(I,J)))>AAMAX){AAMAX=(abs(A(I,J)));};
	//	}
	//	if(AAMAX==0.0){ GetLog() <<"SINGULAR MATRIX."; system("pause"); }
 //       VV(I)=1.0/AAMAX;     
	//  }

 //     for(int J=0;J<N;J++){          
 //         if(J>0){       
	//		  for(int I=0;I<J-1;I++){        
	//			  double SUM=A(I,J);         
	//			  if(I>0){       
	//				  for(int K=0;K<I-1;K++){        
	//				    SUM=SUM-A(I,K)*A(K,J); 
	//				  }
	//				  A(I,J)=SUM;          
	//			  }              
	//		  }
	//	  }

 //     double AAMAX=0.0;           
	//  int IMAX;
	//  for(int I=J;I<N;I++){          
	//	  double SUM=A(I,J);            
	//	  if(J>0){       
	//		  for (int K=0;K<J-1;K++){        
	//		    SUM=SUM-A(I,K)*A(K,J); 
	//		  }
	//		  A(I,J)=SUM; 
	//	  }                 
	//	  double DUM=VV(I)*(abs(SUM)); 
	//	  if(DUM>=AAMAX){ 
	//		  IMAX=I;                
	//		  AAMAX=DUM;             
	//	  }
	//  }

 //     if(J!=IMAX){    
	//	  for(int K=0;K<N;K++){          
	//		  double DUM=A(IMAX,K);         
	//		  A(IMAX,K)=A(J,K);      
	//		  A(J,K)=DUM;     
	//	  }
	//	  D=-D;       
	//	  VV(IMAX)=VV(J);        
	//  }

 //     INDX(J)=IMAX;          
 //     if(J!=N){       
	//	  if(A(J,J)==0.0){A(J,J)=TINY1;}
	//	  double DUM=1./A(J,J);                
	//	  for(int I=J+1;I<N;I++){                 
	//	    A(I,J)=A(I,J)*DUM;              
	//	  }
	//  }
	//  }

	//  if(A(N,N)==0.0){A(N,N)=TINY1;} 
	//}
	virtual void LUDCMP55(ChMatrixNM<double,5,5>&A,double N,double NP,ChMatrixNM<int,5,1>& INDX,double D){
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
    virtual void Basis_M(ChMatrixNM<double,6,5>& M, double x, double y, double z)
				{
					M.Reset();
					M(0,0) = x;
					M(1,1) = y;
					M(2,2) = x;
					M(2,3) = y;
					M(3,4) = z;
				}

			//
			// Functions for interface to ChElementShell base
			// 

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) {
        // this is not a corotational element, so just do:
        EvaluateSectionPoint(u,v,displ, u_displ);
        u_rotaz = VNULL; // no angles.. this is ANCF (or maybe return here the slope derivatives?)
    }

    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) {
        // this is not a corotational element, so just do:
        EvaluateSectionPoint(u,v,displ, point);
        rot = QUNIT; // or maybe use gram-schmidt to get csys of section from slopes?
    }

    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point)
                {
                    ChVector<> u_displ;
					
					ChMatrixNM<double,1,8> N;

					double x = u; // because ShapeFunctions() works in -1..1 range
                    double y = v; // because ShapeFunctions() works in -1..1 range
					double z = 0;

					this->ShapeFunctions(N, x, y, z);
					
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> pD = this->nodes[3]->GetPos();

					point.x = N(0)*pA.x + N(2)*pB.x + N(4)*pC.x + N(6)*pD.x;
                    point.y = N(0)*pA.y + N(2)*pB.y + N(4)*pC.y + N(6)*pD.y;
                    point.z = N(0)*pA.z + N(2)*pB.z + N(4)*pC.z + N(6)*pD.z;
                }

//////////////////////////////////////////////////////////////////////////
/// Inverse matrix for 5x5 ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
    virtual void Inverse55(ChMatrixNM<double,5,5>& A,ChMatrixNM<double,5,5>& B)
                {
					double a1=B(0,0); double a2=B(0,1); double a3=B(0,2); double a4=B(0,3); double a5=B(0,4);
					double b1=B(1,0); double b2=B(1,1); double b3=B(1,2); double b4=B(1,3); double b5=B(1,4);
					double c1=B(2,0); double c2=B(2,1); double c3=B(2,2); double c4=B(2,3); double c5=B(2,4);
					double d1=B(3,0); double d2=B(3,1); double d3=B(3,2); double d4=B(3,3); double d5=B(3,4);
					double e1=B(4,0); double e2=B(4,1); double e3=B(4,2); double e4=B(4,3); double e5=B(4,4);
	A[0][0] = (b2*c3*d4*e5-b2*c3*d5*e4-b2*c4*d3*e5+b2*c4*d5*e3+b2*c5*d3*e4-b2*c5*\
d4*e3-b3*c2*d4*e5+b3*c2*d5*e4+b3*c4*d2*e5-b3*c4*d5*e2-b3*c5*d2*e4+b3*c5*d4*e2+b\
4*c2*d3*e5-b4*c2*d5*e3-b4*c3*d2*e5+b4*c3*d5*e2+b4*c5*d2*e3-b4*c5*d3*e2-b5*c2*d3\
*e4+b5*c2*d4*e3+b5*c3*d2*e4-b5*c3*d4*e2-b5*c4*d2*e3+b5*c4*d3*e2)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[0][1] = -(a2*c3*d4*e5-a2*c3*d5*e4-a2*c4*d3*e5+a2*c4*d5*e3+a2*c5*d3*e4-a2*c5\
*d4*e3-a3*c2*d4*e5+a3*c2*d5*e4+a3*c4*d2*e5-a3*c4*d5*e2-a3*c5*d2*e4+a3*c5*d4*e2+\
a4*c2*d3*e5-a4*c2*d5*e3-a4*c3*d2*e5+a4*c3*d5*e2+a4*c5*d2*e3-a4*c5*d3*e2-a5*c2*d\
3*e4+a5*c2*d4*e3+a5*c3*d2*e4-a5*c3*d4*e2-a5*c4*d2*e3+a5*c4*d3*e2)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[0][2] = (a2*b3*d4*e5-a2*b3*d5*e4-a2*b4*d3*e5+a2*b4*d5*e3+a2*b5*d3*e4-a2*b5*\
d4*e3-a3*b2*d4*e5+a3*b2*d5*e4+a3*b4*d2*e5-a3*b4*d5*e2-a3*b5*d2*e4+a3*b5*d4*e2+a\
4*b2*d3*e5-a4*b2*d5*e3-a4*b3*d2*e5+a4*b3*d5*e2+a4*b5*d2*e3-a4*b5*d3*e2-a5*b2*d3\
*e4+a5*b2*d4*e3+a5*b3*d2*e4-a5*b3*d4*e2-a5*b4*d2*e3+a5*b4*d3*e2)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[0][3] = -(a2*b3*c4*e5-a2*b3*c5*e4-a2*b4*c3*e5+a2*b4*c5*e3+a2*b5*c3*e4-a2*b5\
*c4*e3-a3*b2*c4*e5+a3*b2*c5*e4+a3*b4*c2*e5-a3*b4*c5*e2-a3*b5*c2*e4+a3*b5*c4*e2+\
a4*b2*c3*e5-a4*b2*c5*e3-a4*b3*c2*e5+a4*b3*c5*e2+a4*b5*c2*e3-a4*b5*c3*e2-a5*b2*c\
3*e4+a5*b2*c4*e3+a5*b3*c2*e4-a5*b3*c4*e2-a5*b4*c2*e3+a5*b4*c3*e2)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[0][4] = (a2*b3*c4*d5-a2*b3*c5*d4-a2*b4*c3*d5+a2*b4*c5*d3+a2*b5*c3*d4-a2*b5*\
c4*d3-a3*b2*c4*d5+a3*b2*c5*d4+a3*b4*c2*d5-a3*b4*c5*d2-a3*b5*c2*d4+a3*b5*c4*d2+a\
4*b2*c3*d5-a4*b2*c5*d3-a4*b3*c2*d5+a4*b3*c5*d2+a4*b5*c2*d3-a4*b5*c3*d2-a5*b2*c3\
*d4+a5*b2*c4*d3+a5*b3*c2*d4-a5*b3*c4*d2-a5*b4*c2*d3+a5*b4*c3*d2)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[1][0] = -(b1*c3*d4*e5-b1*c3*d5*e4-b1*c4*d3*e5+b1*c4*d5*e3+b1*c5*d3*e4-b1*c5\
*d4*e3-b3*c1*d4*e5+b3*c1*d5*e4+b3*c4*d1*e5-b3*c4*d5*e1-b3*c5*d1*e4+b3*c5*d4*e1+\
b4*c1*d3*e5-b4*c1*d5*e3-b4*c3*d1*e5+b4*c3*d5*e1+b4*c5*d1*e3-b4*c5*d3*e1-b5*c1*d\
3*e4+b5*c1*d4*e3+b5*c3*d1*e4-b5*c3*d4*e1-b5*c4*d1*e3+b5*c4*d3*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[1][1] = (a1*c3*d4*e5-a1*c3*d5*e4-a1*c4*d3*e5+a1*c4*d5*e3+a1*c5*d3*e4-a1*c5*\
d4*e3-a3*c1*d4*e5+a3*c1*d5*e4+a3*c4*d1*e5-a3*c4*d5*e1-a3*c5*d1*e4+a3*c5*d4*e1+a\
4*c1*d3*e5-a4*c1*d5*e3-a4*c3*d1*e5+a4*c3*d5*e1+a4*c5*d1*e3-a4*c5*d3*e1-a5*c1*d3\
*e4+a5*c1*d4*e3+a5*c3*d1*e4-a5*c3*d4*e1-a5*c4*d1*e3+a5*c4*d3*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[1][2] = -(a1*b3*d4*e5-a1*b3*d5*e4-a1*b4*d3*e5+a1*b4*d5*e3+a1*b5*d3*e4-a1*b5\
*d4*e3-a3*b1*d4*e5+a3*b1*d5*e4+a3*b4*d1*e5-a3*b4*d5*e1-a3*b5*d1*e4+a3*b5*d4*e1+\
a4*b1*d3*e5-a4*b1*d5*e3-a4*b3*d1*e5+a4*b3*d5*e1+a4*b5*d1*e3-a4*b5*d3*e1-a5*b1*d\
3*e4+a5*b1*d4*e3+a5*b3*d1*e4-a5*b3*d4*e1-a5*b4*d1*e3+a5*b4*d3*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[1][3] = (a1*b3*c4*e5-a1*b3*c5*e4-a1*b4*c3*e5+a1*b4*c5*e3+a1*b5*c3*e4-a1*b5*\
c4*e3-a3*b1*c4*e5+a3*b1*c5*e4+a3*b4*c1*e5-a3*b4*c5*e1-a3*b5*c1*e4+a3*b5*c4*e1+a\
4*b1*c3*e5-a4*b1*c5*e3-a4*b3*c1*e5+a4*b3*c5*e1+a4*b5*c1*e3-a4*b5*c3*e1-a5*b1*c3\
*e4+a5*b1*c4*e3+a5*b3*c1*e4-a5*b3*c4*e1-a5*b4*c1*e3+a5*b4*c3*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[1][4] = -(a1*b3*c4*d5-a1*b3*c5*d4-a1*b4*c3*d5+a1*b4*c5*d3+a1*b5*c3*d4-a1*b5\
*c4*d3-a3*b1*c4*d5+a3*b1*c5*d4+a3*b4*c1*d5-a3*b4*c5*d1-a3*b5*c1*d4+a3*b5*c4*d1+\
a4*b1*c3*d5-a4*b1*c5*d3-a4*b3*c1*d5+a4*b3*c5*d1+a4*b5*c1*d3-a4*b5*c3*d1-a5*b1*c\
3*d4+a5*b1*c4*d3+a5*b3*c1*d4-a5*b3*c4*d1-a5*b4*c1*d3+a5*b4*c3*d1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[2][0] = (b1*c2*d4*e5-b1*c2*d5*e4-b1*c4*d2*e5+b1*c4*d5*e2+b1*c5*d2*e4-b1*c5*\
d4*e2-b2*c1*d4*e5+b2*c1*d5*e4+b2*c4*d1*e5-b2*c4*d5*e1-b2*c5*d1*e4+b2*c5*d4*e1+b\
4*c1*d2*e5-b4*c1*d5*e2-b4*c2*d1*e5+b4*c2*d5*e1+b4*c5*d1*e2-b4*c5*d2*e1-b5*c1*d2\
*e4+b5*c1*d4*e2+b5*c2*d1*e4-b5*c2*d4*e1-b5*c4*d1*e2+b5*c4*d2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[2][1] = -(a1*c2*d4*e5-a1*c2*d5*e4-a1*c4*d2*e5+a1*c4*d5*e2+a1*c5*d2*e4-a1*c5\
*d4*e2-a2*c1*d4*e5+a2*c1*d5*e4+a2*c4*d1*e5-a2*c4*d5*e1-a2*c5*d1*e4+a2*c5*d4*e1+\
a4*c1*d2*e5-a4*c1*d5*e2-a4*c2*d1*e5+a4*c2*d5*e1+a4*c5*d1*e2-a4*c5*d2*e1-a5*c1*d\
2*e4+a5*c1*d4*e2+a5*c2*d1*e4-a5*c2*d4*e1-a5*c4*d1*e2+a5*c4*d2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[2][2] = (a1*b2*d4*e5-a1*b2*d5*e4-a1*b4*d2*e5+a1*b4*d5*e2+a1*b5*d2*e4-a1*b5*\
d4*e2-a2*b1*d4*e5+a2*b1*d5*e4+a2*b4*d1*e5-a2*b4*d5*e1-a2*b5*d1*e4+a2*b5*d4*e1+a\
4*b1*d2*e5-a4*b1*d5*e2-a4*b2*d1*e5+a4*b2*d5*e1+a4*b5*d1*e2-a4*b5*d2*e1-a5*b1*d2\
*e4+a5*b1*d4*e2+a5*b2*d1*e4-a5*b2*d4*e1-a5*b4*d1*e2+a5*b4*d2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[2][3] = -(a1*b2*c4*e5-a1*b2*c5*e4-a1*b4*c2*e5+a1*b4*c5*e2+a1*b5*c2*e4-a1*b5\
*c4*e2-a2*b1*c4*e5+a2*b1*c5*e4+a2*b4*c1*e5-a2*b4*c5*e1-a2*b5*c1*e4+a2*b5*c4*e1+\
a4*b1*c2*e5-a4*b1*c5*e2-a4*b2*c1*e5+a4*b2*c5*e1+a4*b5*c1*e2-a4*b5*c2*e1-a5*b1*c\
2*e4+a5*b1*c4*e2+a5*b2*c1*e4-a5*b2*c4*e1-a5*b4*c1*e2+a5*b4*c2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[2][4] = (a1*b2*c4*d5-a1*b2*c5*d4-a1*b4*c2*d5+a1*b4*c5*d2+a1*b5*c2*d4-a1*b5*\
c4*d2-a2*b1*c4*d5+a2*b1*c5*d4+a2*b4*c1*d5-a2*b4*c5*d1-a2*b5*c1*d4+a2*b5*c4*d1+a\
4*b1*c2*d5-a4*b1*c5*d2-a4*b2*c1*d5+a4*b2*c5*d1+a4*b5*c1*d2-a4*b5*c2*d1-a5*b1*c2\
*d4+a5*b1*c4*d2+a5*b2*c1*d4-a5*b2*c4*d1-a5*b4*c1*d2+a5*b4*c2*d1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[3][0] = -(b1*c2*d3*e5-b1*c2*d5*e3-b1*c3*d2*e5+b1*c3*d5*e2+b1*c5*d2*e3-b1*c5\
*d3*e2-b2*c1*d3*e5+b2*c1*d5*e3+b2*c3*d1*e5-b2*c3*d5*e1-b2*c5*d1*e3+b2*c5*d3*e1+\
b3*c1*d2*e5-b3*c1*d5*e2-b3*c2*d1*e5+b3*c2*d5*e1+b3*c5*d1*e2-b3*c5*d2*e1-b5*c1*d\
2*e3+b5*c1*d3*e2+b5*c2*d1*e3-b5*c2*d3*e1-b5*c3*d1*e2+b5*c3*d2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[3][1] = (a1*c2*d3*e5-a1*c2*d5*e3-a1*c3*d2*e5+a1*c3*d5*e2+a1*c5*d2*e3-a1*c5*\
d3*e2-a2*c1*d3*e5+a2*c1*d5*e3+a2*c3*d1*e5-a2*c3*d5*e1-a2*c5*d1*e3+a2*c5*d3*e1+a\
3*c1*d2*e5-a3*c1*d5*e2-a3*c2*d1*e5+a3*c2*d5*e1+a3*c5*d1*e2-a3*c5*d2*e1-a5*c1*d2\
*e3+a5*c1*d3*e2+a5*c2*d1*e3-a5*c2*d3*e1-a5*c3*d1*e2+a5*c3*d2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[3][2] = -(a1*b2*d3*e5-a1*b2*d5*e3-a1*b3*d2*e5+a1*b3*d5*e2+a1*b5*d2*e3-a1*b5\
*d3*e2-a2*b1*d3*e5+a2*b1*d5*e3+a2*b3*d1*e5-a2*b3*d5*e1-a2*b5*d1*e3+a2*b5*d3*e1+\
a3*b1*d2*e5-a3*b1*d5*e2-a3*b2*d1*e5+a3*b2*d5*e1+a3*b5*d1*e2-a3*b5*d2*e1-a5*b1*d\
2*e3+a5*b1*d3*e2+a5*b2*d1*e3-a5*b2*d3*e1-a5*b3*d1*e2+a5*b3*d2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[3][3] = (a1*b2*c3*e5-a1*b2*c5*e3-a1*b3*c2*e5+a1*b3*c5*e2+a1*b5*c2*e3-a1*b5*\
c3*e2-a2*b1*c3*e5+a2*b1*c5*e3+a2*b3*c1*e5-a2*b3*c5*e1-a2*b5*c1*e3+a2*b5*c3*e1+a\
3*b1*c2*e5-a3*b1*c5*e2-a3*b2*c1*e5+a3*b2*c5*e1+a3*b5*c1*e2-a3*b5*c2*e1-a5*b1*c2\
*e3+a5*b1*c3*e2+a5*b2*c1*e3-a5*b2*c3*e1-a5*b3*c1*e2+a5*b3*c2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[3][4] = -(a1*b2*c3*d5-a1*b2*c5*d3-a1*b3*c2*d5+a1*b3*c5*d2+a1*b5*c2*d3-a1*b5\
*c3*d2-a2*b1*c3*d5+a2*b1*c5*d3+a2*b3*c1*d5-a2*b3*c5*d1-a2*b5*c1*d3+a2*b5*c3*d1+\
a3*b1*c2*d5-a3*b1*c5*d2-a3*b2*c1*d5+a3*b2*c5*d1+a3*b5*c1*d2-a3*b5*c2*d1-a5*b1*c\
2*d3+a5*b1*c3*d2+a5*b2*c1*d3-a5*b2*c3*d1-a5*b3*c1*d2+a5*b3*c2*d1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[4][0] = (b1*c2*d3*e4-b1*c2*d4*e3-b1*c3*d2*e4+b1*c3*d4*e2+b1*c4*d2*e3-b1*c4*\
d3*e2-b2*c1*d3*e4+b2*c1*d4*e3+b2*c3*d1*e4-b2*c3*d4*e1-b2*c4*d1*e3+b2*c4*d3*e1+b\
3*c1*d2*e4-b3*c1*d4*e2-b3*c2*d1*e4+b3*c2*d4*e1+b3*c4*d1*e2-b3*c4*d2*e1-b4*c1*d2\
*e3+b4*c1*d3*e2+b4*c2*d1*e3-b4*c2*d3*e1-b4*c3*d1*e2+b4*c3*d2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[4][1] = -(a1*c2*d3*e4-a1*c2*d4*e3-a1*c3*d2*e4+a1*c3*d4*e2+a1*c4*d2*e3-a1*c4\
*d3*e2-a2*c1*d3*e4+a2*c1*d4*e3+a2*c3*d1*e4-a2*c3*d4*e1-a2*c4*d1*e3+a2*c4*d3*e1+\
a3*c1*d2*e4-a3*c1*d4*e2-a3*c2*d1*e4+a3*c2*d4*e1+a3*c4*d1*e2-a3*c4*d2*e1-a4*c1*d\
2*e3+a4*c1*d3*e2+a4*c2*d1*e3-a4*c2*d3*e1-a4*c3*d1*e2+a4*c3*d2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[4][2] = (a1*b2*d3*e4-a1*b2*d4*e3-a1*b3*d2*e4+a1*b3*d4*e2+a1*b4*d2*e3-a1*b4*\
d3*e2-a2*b1*d3*e4+a2*b1*d4*e3+a2*b3*d1*e4-a2*b3*d4*e1-a2*b4*d1*e3+a2*b4*d3*e1+a\
3*b1*d2*e4-a3*b1*d4*e2-a3*b2*d1*e4+a3*b2*d4*e1+a3*b4*d1*e2-a3*b4*d2*e1-a4*b1*d2\
*e3+a4*b1*d3*e2+a4*b2*d1*e3-a4*b2*d3*e1-a4*b3*d1*e2+a4*b3*d2*e1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[4][3] = -(a1*b2*c3*e4-a1*b2*c4*e3-a1*b3*c2*e4+a1*b3*c4*e2+a1*b4*c2*e3-a1*b4\
*c3*e2-a2*b1*c3*e4+a2*b1*c4*e3+a2*b3*c1*e4-a2*b3*c4*e1-a2*b4*c1*e3+a2*b4*c3*e1+\
a3*b1*c2*e4-a3*b1*c4*e2-a3*b2*c1*e4+a3*b2*c4*e1+a3*b4*c1*e2-a3*b4*c2*e1-a4*b1*c\
2*e3+a4*b1*c3*e2+a4*b2*c1*e3-a4*b2*c3*e1-a4*b3*c1*e2+a4*b3*c2*e1)/(a1*b2*c3*d4*\
e5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a\
1*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3\
*c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*\
d2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e\
2-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2\
*b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*\
c4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d\
5*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4\
-a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*\
b1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c\
5*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1\
*e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+\
a3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b\
5*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3\
*d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*\
e3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a\
4*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5\
*c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*\
d2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e\
3-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5\
*b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*\
c2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d\
1*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);
	A[4][4] = (a1*b2*c3*d4-a1*b2*c4*d3-a1*b3*c2*d4+a1*b3*c4*d2+a1*b4*c2*d3-a1*b4*\
c3*d2-a2*b1*c3*d4+a2*b1*c4*d3+a2*b3*c1*d4-a2*b3*c4*d1-a2*b4*c1*d3+a2*b4*c3*d1+a\
3*b1*c2*d4-a3*b1*c4*d2-a3*b2*c1*d4+a3*b2*c4*d1+a3*b4*c1*d2-a3*b4*c2*d1-a4*b1*c2\
*d3+a4*b1*c3*d2+a4*b2*c1*d3-a4*b2*c3*d1-a4*b3*c1*d2+a4*b3*c2*d1)/(a1*b2*c3*d4*e\
5-a1*b2*c3*d5*e4-a1*b2*c4*d3*e5+a1*b2*c4*d5*e3+a1*b2*c5*d3*e4-a1*b2*c5*d4*e3-a1\
*b3*c2*d4*e5+a1*b3*c2*d5*e4+a1*b3*c4*d2*e5-a1*b3*c4*d5*e2-a1*b3*c5*d2*e4+a1*b3*\
c5*d4*e2+a1*b4*c2*d3*e5-a1*b4*c2*d5*e3-a1*b4*c3*d2*e5+a1*b4*c3*d5*e2+a1*b4*c5*d\
2*e3-a1*b4*c5*d3*e2-a1*b5*c2*d3*e4+a1*b5*c2*d4*e3+a1*b5*c3*d2*e4-a1*b5*c3*d4*e2\
-a1*b5*c4*d2*e3+a1*b5*c4*d3*e2-a2*b1*c3*d4*e5+a2*b1*c3*d5*e4+a2*b1*c4*d3*e5-a2*\
b1*c4*d5*e3-a2*b1*c5*d3*e4+a2*b1*c5*d4*e3+a2*b3*c1*d4*e5-a2*b3*c1*d5*e4-a2*b3*c\
4*d1*e5+a2*b3*c4*d5*e1+a2*b3*c5*d1*e4-a2*b3*c5*d4*e1-a2*b4*c1*d3*e5+a2*b4*c1*d5\
*e3+a2*b4*c3*d1*e5-a2*b4*c3*d5*e1-a2*b4*c5*d1*e3+a2*b4*c5*d3*e1+a2*b5*c1*d3*e4-\
a2*b5*c1*d4*e3-a2*b5*c3*d1*e4+a2*b5*c3*d4*e1+a2*b5*c4*d1*e3-a2*b5*c4*d3*e1+a3*b\
1*c2*d4*e5-a3*b1*c2*d5*e4-a3*b1*c4*d2*e5+a3*b1*c4*d5*e2+a3*b1*c5*d2*e4-a3*b1*c5\
*d4*e2-a3*b2*c1*d4*e5+a3*b2*c1*d5*e4+a3*b2*c4*d1*e5-a3*b2*c4*d5*e1-a3*b2*c5*d1*\
e4+a3*b2*c5*d4*e1+a3*b4*c1*d2*e5-a3*b4*c1*d5*e2-a3*b4*c2*d1*e5+a3*b4*c2*d5*e1+a\
3*b4*c5*d1*e2-a3*b4*c5*d2*e1-a3*b5*c1*d2*e4+a3*b5*c1*d4*e2+a3*b5*c2*d1*e4-a3*b5\
*c2*d4*e1-a3*b5*c4*d1*e2+a3*b5*c4*d2*e1-a4*b1*c2*d3*e5+a4*b1*c2*d5*e3+a4*b1*c3*\
d2*e5-a4*b1*c3*d5*e2-a4*b1*c5*d2*e3+a4*b1*c5*d3*e2+a4*b2*c1*d3*e5-a4*b2*c1*d5*e\
3-a4*b2*c3*d1*e5+a4*b2*c3*d5*e1+a4*b2*c5*d1*e3-a4*b2*c5*d3*e1-a4*b3*c1*d2*e5+a4\
*b3*c1*d5*e2+a4*b3*c2*d1*e5-a4*b3*c2*d5*e1-a4*b3*c5*d1*e2+a4*b3*c5*d2*e1+a4*b5*\
c1*d2*e3-a4*b5*c1*d3*e2-a4*b5*c2*d1*e3+a4*b5*c2*d3*e1+a4*b5*c3*d1*e2-a4*b5*c3*d\
2*e1+a5*b1*c2*d3*e4-a5*b1*c2*d4*e3-a5*b1*c3*d2*e4+a5*b1*c3*d4*e2+a5*b1*c4*d2*e3\
-a5*b1*c4*d3*e2-a5*b2*c1*d3*e4+a5*b2*c1*d4*e3+a5*b2*c3*d1*e4-a5*b2*c3*d4*e1-a5*\
b2*c4*d1*e3+a5*b2*c4*d3*e1+a5*b3*c1*d2*e4-a5*b3*c1*d4*e2-a5*b3*c2*d1*e4+a5*b3*c\
2*d4*e1+a5*b3*c4*d1*e2-a5*b3*c4*d2*e1-a5*b4*c1*d2*e3+a5*b4*c1*d3*e2+a5*b4*c2*d1\
*e3-a5*b4*c2*d3*e1-a5*b4*c3*d1*e2+a5*b4*c3*d2*e1);



	}


};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
