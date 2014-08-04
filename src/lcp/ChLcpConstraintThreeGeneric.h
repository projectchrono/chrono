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

#ifndef CHLCPCONSTRAINTTHREEGENERIC_H
#define CHLCPCONSTRAINTTHREEGENERIC_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintThreeGeneric.h
//
//    An 'easy' derived class for representing a
//   constraint between three ChLcpVariable items.
//   Used with for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintThree.h"
#include "ChLcpVariables.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the base ChLcpConstraintThree(),
/// which is a base for triple constraints. So here this class implements
/// the functionality for a constraint between a TRHEE
/// objects of type ChLcpVariables(), with generic number of scalar
/// variables each (ex.ChLcpVariablesGeneric() or ChLcpVariablesBody() )
/// and defines three jacobian matrices, whose column number automatically
/// matches the number of elements in variables vectors.
///  Before starting the LCP solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChLcpConstraintThreeGeneric : public ChLcpConstraintThree
{
	CH_RTTI(ChLcpConstraintThreeGeneric, ChLcpConstraintThree)

			//
			// DATA
			//

protected:
				/// The [Cq_a] jacobian of the constraint
	ChMatrixDynamic<float>* Cq_a;
				/// The [Cq_b] jacobian of the constraint
	ChMatrixDynamic<float>* Cq_b;
				/// The [Cq_c] jacobian of the constraint
	ChMatrixDynamic<float>* Cq_c;


				// Auxiliary data: will be used by iterative constraint solvers:

				/// The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
	ChMatrixDynamic<float>* Eq_a;
				/// The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'
	ChMatrixDynamic<float>* Eq_b;
				/// The [Eq_a] product [Eq_c]=[invM_b]*[Cq_c]'
	ChMatrixDynamic<float>* Eq_c;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintThreeGeneric()
					{
						Cq_a = Cq_b = Cq_c = Eq_a = Eq_b = Eq_c = NULL;
					};

						/// Construct and immediately set references to variables
	ChLcpConstraintThreeGeneric(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b, ChLcpVariables* mvariables_c)
					{
						Cq_a = Cq_b = Cq_c = Eq_a = Eq_b = Eq_c = NULL;
						SetVariables(mvariables_a, mvariables_b, mvariables_c);
					};

						/// Copy constructor
	ChLcpConstraintThreeGeneric(const ChLcpConstraintThreeGeneric& other) 
			: ChLcpConstraintThree(other)
					{
						Cq_a = Cq_b = Cq_c = Eq_a = Eq_b = Eq_c = NULL;
						if(other.Cq_a) Cq_a = new ChMatrixDynamic<float>(*other.Cq_a);
						if(other.Cq_b) Cq_b = new ChMatrixDynamic<float>(*other.Cq_b);
						if(other.Cq_c) Cq_c = new ChMatrixDynamic<float>(*other.Cq_c);
						if(other.Eq_a) Eq_a = new ChMatrixDynamic<float>(*other.Eq_a);
						if(other.Eq_b) Eq_b = new ChMatrixDynamic<float>(*other.Eq_b);
						if(other.Eq_c) Eq_c = new ChMatrixDynamic<float>(*other.Eq_c);
					}

	virtual ~ChLcpConstraintThreeGeneric()
					{
						if (Cq_a) delete Cq_a;
						if (Cq_b) delete Cq_b;
						if (Cq_c) delete Cq_c;
						if (Eq_a) delete Eq_a;
						if (Eq_b) delete Eq_b;
						if (Eq_c) delete Eq_c;
					};


	virtual ChLcpConstraintThreeGeneric* new_Duplicate () {return new ChLcpConstraintThreeGeneric(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintThreeGeneric& operator=(const ChLcpConstraintThreeGeneric& other);



			//
			// FUNCTIONS
			//

				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_a() {return Cq_a;}
				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_b() {return Cq_b;}
				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_c() {return Cq_c;}

				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_a() {return Eq_a;}
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_b() {return Eq_b;}
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_c() {return Eq_c;}


				/// Set references to the constrained objects, each of ChLcpVariables type,
				/// automatically creating/resizing jacobians if needed.
	virtual void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b, ChLcpVariables* mvariables_c);


				/// This function updates the following auxiliary data:
				///  - the Eq_a and Eq_b and Eq_c  matrices
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
						 for (int i= 0; i < Cq_a->GetColumns(); i++)
							ret += Cq_a->ElementN(i) * variables_a->Get_qb().ElementN(i);

						if (variables_b->IsActive())
						 for (int i= 0; i < Cq_b->GetColumns(); i++)
							ret += Cq_b->ElementN(i) * variables_b->Get_qb().ElementN(i);

						if (variables_c->IsActive())
						 for (int i= 0; i < Cq_c->GetColumns(); i++)
							ret += Cq_c->ElementN(i) * variables_c->Get_qb().ElementN(i);

						return ret;
					}



				///  This function must increment the vector of variables
				/// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
				///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
				///  This is used for some iterative LCP solvers.

	virtual void Increment_q(const double deltal)
					{
						if (variables_a->IsActive())
						 for (int i= 0; i < Eq_a->GetRows(); i++)
							variables_a->Get_qb()(i) += Eq_a->ElementN(i) * deltal;

						if (variables_b->IsActive())
						 for (int i= 0; i < Eq_b->GetRows(); i++)
							variables_b->Get_qb()(i) += Eq_b->ElementN(i) * deltal;

						if (variables_c->IsActive())
						 for (int i= 0; i < Eq_c->GetRows(); i++)
							variables_c->Get_qb()(i) += Eq_c->ElementN(i) * deltal;
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
						 for (int i= 0; i < Cq_a->GetRows(); i++)
							result += vect(variables_a->GetOffset()+i) * Cq_a->ElementN(i);

						if (variables_b->IsActive())
						 for (int i= 0; i < Cq_b->GetRows(); i++)
							result += vect(variables_b->GetOffset()+i) * Cq_b->ElementN(i);

						if (variables_c->IsActive())
						 for (int i= 0; i < Cq_c->GetRows(); i++)
							result += vect(variables_c->GetOffset()+i) * Cq_c->ElementN(i); 
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
						 for (int i= 0; i < Cq_a->GetRows(); i++)
							result(variables_a->GetOffset()+i) += Cq_a->ElementN(i) * l;

						if (variables_b->IsActive())
						 for (int i= 0; i < Cq_b->GetRows(); i++)
							result(variables_b->GetOffset()+i) += Cq_b->ElementN(i) * l;

						if (variables_c->IsActive())
						 for (int i= 0; i < Cq_c->GetRows(); i++)
							result(variables_c->GetOffset()+i) += Cq_c->ElementN(i) * l; 
					};

				/// Puts the three jacobian parts into the 'insrow' row of a sparse matrix,
				/// where both portions of the jacobian are shifted in order to match the 
				/// offset of the corresponding ChLcpVariable.
				/// This is used only by the ChLcpSimplex solver (iterative solvers 
				/// don't need to know jacobians explicitly)
	virtual void Build_Cq(ChSparseMatrix& storage, int insrow)
					{
						if (variables_a->IsActive())
							storage.PasteMatrixFloat(Cq_a, insrow, variables_a->GetOffset());
						if (variables_b->IsActive())
							storage.PasteMatrixFloat(Cq_b, insrow, variables_b->GetOffset());
						if (variables_b->IsActive())
							storage.PasteMatrixFloat(Cq_c, insrow, variables_c->GetOffset());
					}
	virtual void Build_CqT(ChSparseMatrix& storage, int inscol)
					{
						if (variables_a->IsActive())
							storage.PasteTranspMatrixFloat(Cq_a, variables_a->GetOffset(), inscol);
						if (variables_b->IsActive())
							storage.PasteTranspMatrixFloat(Cq_b, variables_b->GetOffset(), inscol);
						if (variables_c->IsActive())
							storage.PasteTranspMatrixFloat(Cq_c, variables_c->GetOffset(), inscol);
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
