#ifndef CHLCPCONSTRAINTTWOGPUCONTT_H
#define CHLCPCONSTRAINTTWOGPUCONTT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoGPUcontT.h
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
#include "ChLcpVariables.h"

namespace chrono
{


/// Base class for friction constraints (see specialized children classes
/// for more details - this is mostly an interface)

class ChLcpConstraintTwoGPUcontT : public ChLcpConstraintTwo
{
	CH_RTTI(ChLcpConstraintTwoGPUcontT, ChLcpConstraintTwo)

			//
			// DATA
			//

protected:


public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoGPUcontT()
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintTwoGPUcontT( ChLcpVariables* mvariables_a,
								ChLcpVariables* mvariables_b)
					{
						SetVariables(mvariables_a, mvariables_b);
						mode = CONSTRAINT_FRIC; 
					};

						/// Copy constructor
	ChLcpConstraintTwoGPUcontT(const ChLcpConstraintTwoGPUcontT& other) 
				: ChLcpConstraintTwo(other)
					{
					}

	virtual ~ChLcpConstraintTwoGPUcontT() {};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoGPUcontT(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoGPUcontT& operator=(const ChLcpConstraintTwoGPUcontT& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwo::operator=(other);

						return *this;
					}


			//
			// FUNCTIONS
			//

					/// Tells that this constraint is not linear, that is: it cannot
					/// be solved with a plain simplex solver.
	virtual bool IsLinear() {return false;}

					/// The constraint is satisfied?
	virtual double Violation(double mc_i); 

				/// The projection should be done directly on the GPU custom kernel code,
				/// so this won't be ever called.
	virtual void Project() {assert (false);};

				/// Computations with the following data are done directly on the GPU 
				/// custom kernel code, so this won't be ever called. 
	virtual ChMatrix<float>* Get_Cq_a() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Cq_b() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Eq_a() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Eq_b() {assert (false); return 0;}
	virtual double Compute_Cq_q() {assert (false); return 0;}
	virtual void Increment_q(const double deltal) {assert (false);};
	virtual void Build_Cq(ChSparseMatrix& storage, int insrow) {assert (false);};

				/// Set references to the constrained objects
	virtual void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b)
					{
						valid = true;
						variables_a = mvariables_a;
						variables_b = mvariables_b;
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




#endif  // END of ChLcpConstraintTwoGPUcontT.h
