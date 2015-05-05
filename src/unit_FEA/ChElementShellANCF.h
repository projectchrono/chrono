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


#include "ChElementBeam.h"
#include "ChShellSection.h"
#include "ChNodeFEAxyzD.h"
#include "core/ChQuadrature.h"


namespace chrono
{
namespace fea
{



/// Simple shell element with four nodes and ANCF gradient-deficient
/// formulation.
/// Based on the formulation in:
///  xxxyyyzzz TO DO

class  ChElementShellANCF : public ChElementShell
{
protected:
	std::vector< ChSharedPtr<ChNodeFEAxyzD> > nodes;
	
	ChSharedPtr<ChShellSectionBasic> section;

	ChMatrixNM<double,24,24> StiffnessMatrix; // stiffness matrix
	ChMatrixNM<double,24,24> MassMatrix;	   // mass matrix


public:

	ChElementShellANCF()
				{
					nodes.resize(4);

				}

	virtual ~ChElementShellANCF() {}

	virtual int GetNnodes()  {return 4;}
	virtual int GetNcoords() {return 4*6;}
	virtual int GetNdofs()   {return 4*6;}

	virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(  ChSharedPtr<ChNodeFEAxyzD> nodeA, 
                            ChSharedPtr<ChNodeFEAxyzD> nodeB,
                            ChSharedPtr<ChNodeFEAxyzD> nodeC,
                            ChSharedPtr<ChNodeFEAxyzD> nodeD) 
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
				}

				

			//
			// FEM functions
			//

				/// Set the section & material of beam element .
				/// It is a shared property, so it can be shared between other beams.
	void   SetSection( ChSharedPtr<ChShellSectionBasic> my_material) { section = my_material; }
				/// Get the section & material of the element
	ChSharedPtr<ChShellSectionBasic> GetSection() {return section;}

				/// Get the first node  
	ChSharedPtr<ChNodeFEAxyzD> GetNodeA() {return nodes[0];}

				/// Get the second node 
	ChSharedPtr<ChNodeFEAxyzD> GetNodeB() {return nodes[1];}

    			/// Get the third node
	ChSharedPtr<ChNodeFEAxyzD> GetNodeC() {return nodes[2];}

    			/// Get the fourth node
	ChSharedPtr<ChNodeFEAxyzD> GetNodeD() {return nodes[3];}


	

				/// Fills the N shape function matrix with the
				/// values of shape functions at parametric coordinates 'x,y'.
				/// Note, xi=0..1, yi=0..1.  
                /// Note, xi is in the'length' direction, i.e. from node 1 to 2.
	virtual void ShapeFunctions(ChMatrix<>& N, double xi,  double yi)
				{
					N(0) = 0.25 * (1-xi) * (1-yi);
					N(1) = 0.25 * (1+xi) * (1-yi);
					N(2) = 0.25 * (1+xi) * (1+yi);
                    N(3) = 0.25 * (1-xi) * (1+yi);
				};

				/// Fills the N shape function derivatives matrix with the
				/// values of shape functions at parametric coordinates 'x,y'.
				/// Note, xi=0..1, yi=0..1.
	virtual void ShapeFunctionsDerivatives(ChMatrix<>& Nd, double xi)
				{
					// ***TODO*** (Hiro: derivatives needed? they were used in beams)
					// Nd(0) = // ***TODO***
					// Nd(1) = // ***TODO***
					// ....
				};

	virtual void ShapeFunctionsDerivatives2(ChMatrix<>& Ndd, double xi)
				{
					// ***TODO*** (Hiro: dderivatives needed? they were used in beams)
					// Ndd(0) = // ***TODO***
					// Ndd(1) = // ***TODO***
					// ....
				};


	virtual void Update() 
				{
					// parent class update:
					ChElementGeneric::Update();
                    
                    // nothing to do here...
				};

				/// Fills the D vector (column matrix) with the current 
				/// field values at the nodes of the element, with proper ordering.
				/// If the D vector has not the size of this->GetNdofs(), it will be resized.
				///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b   etc....}
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
					assert (!section.IsNull());
					
					bool use_numerical_differentiation = true;

					//***TO DO****
                    // Hiro: see the ComputeStiffnessMatrix() function for ANCF beams for a hint..

					// this->StiffnessMatrix = ...;

				}

				/// Computes the MASS MATRIX of the element
				/// Note: in this 'basic' implementation, constant section and 
				/// constant material are assumed 
	virtual void ComputeMassMatrix()
				{	
					assert (!section.IsNull());

					double thickness = section->thickness;
					double rho		 = section->density;

				    // ***TO DO***
					
					//this->MassMatrix = ...;
				}


				/// Setup. Precompute mass and matrices that do not change during the 
				/// simulation, ex. the mass matrix in ANCF is constant

	virtual void SetupInitial() 
				{
					assert (!section.IsNull());


					// Compute mass matrix
					ComputeMassMatrix();

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
					double kr_factor = Kfactor + Rfactor * this->section->rdamping;  

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
					assert (!section.IsNull());

					//***TO DO***
                    // Hiro: see ComputeInternalForces for ANCF beams for hints, ex Gauss quadrature in C::E etc.
					//Fi = ...;
				}


   
			//
			// Beam-specific functions
			//

				/// Gets the xyz displacement of a point on the shell,
                /// and the rotation RxRyRz of section reference, at parametric coordinates 'u' and 'v'.
                /// Note, u=-1..+1 , v= -1..+1.
                /// Note, 'displ' is the displ.state of nodes, ex. get it as GetField()
                /// Results are not corotated.
	virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz)
				{
					//***TO DO***
                    // Hiro.... I think this is not needed 

					u_displ = VNULL; //(not needed in ANCF? )
					u_rotaz = VNULL; //(not needed in ANCF? )
				}
	
				/// Gets the absolute xyz position of a point on the shell,
                /// and the absolute rotation of section reference,  at parametric coordinates 'u' and 'v'.
                /// Note, u=-1..+1 , v= -1..+1.
                /// Note, 'displ' is the displ.state of nodes, ex. get it as GetField()
                /// Results are corotated.
    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot)
				{
					//***TO DO***
                    // Hiro: look into EvaluateSectionFrame() of ANCF beam for hints

					//point.x = ...
					//point.y = ...

					//rot = ...
				}

                    /// Gets the absolute xyz position of a point on the shell,
                    /// at parametric coordinates 'u' and 'v'.
                    /// Note, u=-1..+1 , v= -1..+1.
                    /// Note, 'displ' is the displ.state of nodes, ex. get it as GetField()
                    /// Results are corotated.
    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point)
                {
                    ChVector<> u_displ;
					
					ChMatrixNM<double,1,4> N;

					double xi = (u+1.0)*0.5; // because ShapeFunctions() works in 0..1 range
                    double yi = (v+1.0)*0.5; // because ShapeFunctions() works in 0..1 range

					this->ShapeFunctions(N, xi,yi);
					
					ChVector<> pA = this->nodes[0]->GetPos();
					ChVector<> pB = this->nodes[1]->GetPos();
					ChVector<> pC = this->nodes[2]->GetPos();
					ChVector<> pD = this->nodes[3]->GetPos();

					point.x = N(0)*pA.x + N(1)*pB.x + N(2)*pC.x + N(3)*pD.x;
                    point.y = N(0)*pA.y + N(1)*pB.y + N(2)*pC.y + N(3)*pD.y;
                    point.z = N(0)*pA.z + N(1)*pB.z + N(2)*pC.z + N(3)*pD.z;
                }


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






