//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTHREEBBSHAFT_H
#define CHLCPCONSTRAINTTHREEBBSHAFT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintThreeBBShaft.h
//
//   A class for representing a constraint between 
//   two bodies (2x6dof in space) and a 1D dof (a shaft)
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintThree.h"
#include "ChLcpVariablesBody.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


/// A class for representing a constraint between 
/// two bodies (2x6dof in space) and a 1D dof (a shaft)

class ChApi ChLcpConstraintThreeBBShaft : public ChLcpConstraintThree
{
	CH_RTTI(ChLcpConstraintThreeBBShaft, ChLcpConstraintThree)

			//
			// DATA
			//

protected:


				/// The [Cq_a] jacobian of the constraint
	ChMatrixNM<float,1,6> Cq_a;
				/// The [Cq_b] jacobian of the constraint
	ChMatrixNM<float,1,6> Cq_b;
				/// The [Cq_c] jacobian of the constraint
	ChMatrixNM<float,1,1> Cq_c;


				// Auxiliary data: will be used by iterative constraint solvers:

				/// The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
	ChMatrixNM<float,6,1> Eq_a;
				/// The [Eq_b] product [Eq_b]=[invM_b]*[Cq_b]'
	ChMatrixNM<float,6,1> Eq_b;
				/// The [Eq_c] product [Eq_c]=[invM_c]*[Cq_c]'
	ChMatrixNM<float,1,1> Eq_c;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintThreeBBShaft()
					{
					};

						/// Construct and immediately set references to variables
	ChLcpConstraintThreeBBShaft(ChLcpVariablesBody* mvariables_a, ChLcpVariablesBody* mvariables_b, ChLcpVariables* mvariables_c)
					{
						assert(mvariables_c->Get_ndof()==1);
						SetVariables(mvariables_a, mvariables_b, mvariables_c);
					};

						/// Copy constructor
	ChLcpConstraintThreeBBShaft(const ChLcpConstraintThreeBBShaft& other)  
			: ChLcpConstraintThree(other)
					{
						Cq_a = other.Cq_a;
						Cq_b = other.Cq_b;
						Cq_c = other.Cq_c;
						Eq_a = other.Eq_a;
						Eq_b = other.Eq_b;
						Eq_c = other.Eq_c;
					}

	virtual ~ChLcpConstraintThreeBBShaft()
					{
					};


	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintThreeBBShaft(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintThreeBBShaft& operator=(const ChLcpConstraintThreeBBShaft& other);



			//
			// FUNCTIONS
			//

				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_a() {return &Cq_a;}
				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_b() {return &Cq_b;}
				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_c() {return &Cq_c;}

				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_a() {return &Eq_a;}
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_b() {return &Eq_b;}
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_c() {return &Eq_c;}


				/// Set references to the constrained objects,
				/// If first two variables aren't from ChLcpVariablesBody class, an assert failure happens.
	void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b, ChLcpVariables* mvariables_c);


				/// This function updates the following auxiliary data:
				///  - the Eq_a and Eq_b and Eq_c matrices
				///  - the g_i product
				/// This is often called by LCP solvers at the beginning
				/// of the solution process.
				/// Most often, inherited classes won't need to override this.
	virtual void Update_auxiliary();
	

				///  This function must computes the product between
				/// the row-jacobian of this constraint '[Cq_i]' and the
				/// vector of variables, 'v'. that is    CV=[Cq_i]*v
				///  This is used for some iterative LCP solvers.
	virtual double Compute_Cq_q() 
					{
						double ret = 0;

						if (variables_a->IsActive())
						 for (int i= 0; i < 6; i++)
							ret += Cq_a.ElementN(i) * variables_a->Get_qb().ElementN(i);

						if (variables_b->IsActive())
						 for (int i= 0; i < 6; i++)
							ret += Cq_b.ElementN(i) * variables_b->Get_qb().ElementN(i);

						if (variables_c->IsActive())
							ret += Cq_c.ElementN(0) * variables_c->Get_qb().ElementN(0);

						return ret;
					}

		

				///  This function must increment the vector of variables
				/// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
				///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
				///  This is used for some iterative LCP solvers.

	virtual void Increment_q(const double deltal)
					{
						if (variables_a->IsActive())
						 for (int i= 0; i < Eq_a.GetRows(); i++)
							variables_a->Get_qb()(i) += Eq_a.ElementN(i) * deltal;

						if (variables_b->IsActive())
						 for (int i= 0; i < Eq_b.GetRows(); i++)
							variables_b->Get_qb()(i) += Eq_b.ElementN(i) * deltal;

						if (variables_c->IsActive())
							variables_c->Get_qb()(0) += Eq_c.ElementN(0) * deltal;
					};


				/// Computes the product of the corresponding block in the 
				/// system matrix by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' vector must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect; 
	virtual void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const
					{
						if (variables_a->IsActive())
						 for (int i= 0; i < Cq_a.GetRows(); i++)
							result += vect(variables_a->GetOffset()+i) * Cq_a.ElementN(i);

						if (variables_b->IsActive())
						 for (int i= 0; i < Cq_b.GetRows(); i++)
							result += vect(variables_b->GetOffset()+i) * Cq_b.ElementN(i);

						if (variables_c->IsActive())
							result += vect(variables_c->GetOffset()) * Cq_c.ElementN(0);  
					};

				/// Computes the product of the corresponding transposed blocks in the 
				/// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'. 
				/// NOTE: the 'result' vector must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect; 
	virtual void MultiplyTandAdd(ChMatrix<double>& result, double l)
					{
						if (variables_a->IsActive())
						 for (int i= 0; i < Cq_a.GetRows(); i++)
							result(variables_a->GetOffset()+i) += Cq_a.ElementN(i) * l;

						if (variables_b->IsActive())
						 for (int i= 0; i < Cq_b.GetRows(); i++)
							result(variables_b->GetOffset()+i) += Cq_b.ElementN(i) * l;

						if (variables_c->IsActive())
							result(variables_c->GetOffset()) += Cq_c.ElementN(0) * l;  
					};

				/// Puts the jacobian parts into the 'insrow' row of a sparse matrix,
				/// where both portions of the jacobian are shifted in order to match the 
				/// offset of the corresponding ChLcpVariable.
				/// This is used only by the ChLcpSimplex solver (iterative solvers 
				/// don't need to know jacobians explicitly)
	virtual void Build_Cq(ChSparseMatrix& storage, int insrow)
					{
						if (variables_a->IsActive())
							storage.PasteMatrixFloat(&Cq_a, insrow, variables_a->GetOffset());
						if (variables_b->IsActive())
							storage.PasteMatrixFloat(&Cq_b, insrow, variables_b->GetOffset());
						if (variables_c->IsActive())
							storage.PasteMatrixFloat(&Cq_c, insrow, variables_c->GetOffset());
					}
	virtual void Build_CqT(ChSparseMatrix& storage, int inscol)
					{
						if (variables_a->IsActive())
							storage.PasteTranspMatrixFloat(&Cq_a, variables_a->GetOffset(), inscol);
						if (variables_b->IsActive())
							storage.PasteTranspMatrixFloat(&Cq_b, variables_b->GetOffset(), inscol);
						if (variables_c->IsActive())
							storage.PasteTranspMatrixFloat(&Cq_c, variables_c->GetOffset(), inscol);
					}


			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____



#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  
