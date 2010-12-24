#ifndef CHLCPCONSTRAINTTWOBODIES_H
#define CHLCPCONSTRAINTTWOBODIES_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoBodies.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariableBody items.
//   Used with LCP systems including inequalities,
//   equalities, nonlinearities, etc.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwo.h"
#include "ChLcpVariablesBody.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the base ChLcpConstraintTwo(),
/// that is implements the functionality for a constraint between 
/// a couple of two objects of type ChLcpVariablesBody(). 
/// Important note, it reports 
/// IsGPUcompatible() as 'true' because here we're sure that 
/// jacobians are 1x6 and 1x6, as required by the GPU solver.

class ChApi ChLcpConstraintTwoBodies : public ChLcpConstraintTwo
{
	CH_RTTI(ChLcpConstraintTwoBodies, ChLcpConstraintTwo)

			//
			// DATA
			//

protected:
				/// The [Cq_a] jacobian of the constraint
	ChMatrixNM<float,1,6> Cq_a;
				/// The [Cq_b] jacobian of the constraint
	ChMatrixNM<float,1,6> Cq_b;


				// Auxiliary data: will be used by iterative constraint solvers:

				/// The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
	ChMatrixNM<float,6,1> Eq_a;
				/// The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'
	ChMatrixNM<float,6,1> Eq_b;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoBodies()
					{
					};

						/// Construct and immediately set references to variables
	ChLcpConstraintTwoBodies(ChLcpVariablesBody* mvariables_a, ChLcpVariablesBody* mvariables_b)
					{
						SetVariables(mvariables_a, mvariables_b);
					};

						/// Copy constructor
	ChLcpConstraintTwoBodies(const ChLcpConstraintTwoBodies& other)  
			: ChLcpConstraintTwo(other)
					{
						Cq_a = other.Cq_a;
						Cq_b = other.Cq_b;
						Eq_a = other.Eq_a;
						Eq_b = other.Eq_b;
					}

	virtual ~ChLcpConstraintTwoBodies()
					{
					};


	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoBodies(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoBodies& operator=(const ChLcpConstraintTwoBodies& other);



			//
			// FUNCTIONS
			//
				
				/// Return true because this is the constraint type supported by GPU solver
	virtual bool IsGPUcompatible() {return true;}


				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_a() {return &Cq_a;}
				/// Access jacobian matrix
	virtual ChMatrix<float>* Get_Cq_b() {return &Cq_b;}

				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_a() {return &Eq_a;}
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<float>* Get_Eq_b() {return &Eq_b;}


				/// Set references to the constrained objects, each of ChLcpVariablesBody type,
				/// automatically creating/resizing jacobians if needed.
				/// If variables aren't from ChLcpVariablesBody class, an assert failure happens.
	void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b);


				/// This function updates the following auxiliary data:
				///  - the Eq_a and Eq_b matrices
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

						return ret;
					}

		

				///  This function must increment the vector of variables
				/// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
				///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
				///  This is used for some iterative LCP solvers.

	virtual void Increment_q(const double deltal)
					{
						if (variables_a->IsActive())
						 for (int i= 0; i < 6; i++)
							variables_a->Get_qb()(i) += Eq_a.ElementN(i) * deltal;

						if (variables_b->IsActive())
						 for (int i= 0; i < 6; i++)
							variables_b->Get_qb()(i) += Eq_b.ElementN(i) * deltal;
					};


				/// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
				/// where both portions of the jacobian are shifted in order to match the 
				/// offset of the corresponding ChLcpVariable.The same is done
				/// on the 'insrow' column, so that the sparse matrix is kept symmetric.
				/// This is used only by the ChLcpSimplex solver (iterative solvers 
				/// don't need to know jacobians explicitly)
	virtual void Build_Cq(ChSparseMatrix& storage, int insrow)
					{
						if (variables_a->IsActive())
							storage.PasteMatrixFloat(&Cq_a, insrow, variables_a->GetOffset());
						if (variables_b->IsActive())
							storage.PasteMatrixFloat(&Cq_b, insrow, variables_b->GetOffset());
					}
	virtual void Build_CqT(ChSparseMatrix& storage, int inscol)
					{
						if (variables_a->IsActive())
							storage.PasteTranspMatrixFloat(&Cq_a, variables_a->GetOffset(), inscol);
						if (variables_b->IsActive())
							storage.PasteTranspMatrixFloat(&Cq_b, variables_b->GetOffset(), inscol);
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
