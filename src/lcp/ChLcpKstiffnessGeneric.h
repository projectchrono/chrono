#ifndef CHLCPKSTIFFNESSGENERIC_H
#define CHLCPKSTIFFNESSGENERIC_H

//////////////////////////////////////////////////
//
//   ChLcpKstiffnessGeneric.h
//
//    Base class for representing a block-sparse stiffness
//   matrix between N vector-variables in a VI problem
//   Used for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "lcp/ChLcpKstiffness.h"


namespace chrono
{

/// Class that represent nxn sparse blocks to put into K global
/// matrix, that is blocks that connect N 'variables'
/// and build a matrix K in a sparse variational inequality VI(Z*x-d,K):
///
///  | M+K -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq   -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M+K  Cq'|*| q|-| f|=|0|  
///                                           |Cq    E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones
/// Note that 'stiffness' blocks do not necessarily have a physical 
/// interpretation as stiffness, for example they can represent hessians.
/// Note that all stiffness blocks in K, all masses and constraint
/// jacobians Cq are not really assembled in large matrices, so to
/// exploit sparsity).


class ChApi ChLcpKstiffnessGeneric : public ChLcpKstiffness
{
	CH_RTTI(ChLcpKstiffnessGeneric, ChLcpKstiffness)

private:
			//
			// DATA
			//

	ChMatrixDynamic<double>* K;

	std::vector<ChLcpVariables*> variables;

public:

			//
			// CONSTRUCTORS
			//

	ChLcpKstiffnessGeneric()
				{
					K=0;
				}

	ChLcpKstiffnessGeneric(std::vector<ChLcpVariables*> mvariables)
				{
					K=0;
					this->SetVariables(mvariables);
				}

	ChLcpKstiffnessGeneric(ChLcpVariables* mvariableA, ChLcpVariables* mvariableB)
				{
					K=0;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(mvariableA);
					mvars.push_back(mvariableB);
					this->SetVariables(mvars);
				}

	virtual ~ChLcpKstiffnessGeneric()
				{
					if (K) delete K; K=0;
				};

	
				/// Assignment operator: copy from other object
	ChLcpKstiffnessGeneric& operator=(const ChLcpKstiffnessGeneric& other);


			//
			// FUNCTIONS
			//

				/// Set references to the constrained objects, each of ChLcpVariables type,
				/// automatically creating/resizing K matrix if needed.
	void SetVariables(std::vector<ChLcpVariables*> mvariables);

				/// Returns the number of referenced ChLcpVariables items
	virtual unsigned int GetNvars() const {return variables.size();}

				/// Access the m-th vector variable object
	virtual ChLcpVariables* GetVariableN(unsigned int m_var) {return variables[m_var];}



				/// Access the K stiffness matrix as a single block,
				/// referring only to the referenced ChVariable objects 
	virtual ChMatrix<double>* Get_K()
				{
					return K;
				}

				/// Computes the product of the corresponding blocks in the 
				/// system matrix (ie. the K matrix blocks) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect.
	virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect);

				/// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) as index.
	virtual void DiagonalAdd(ChMatrix<double>& result);

				/// Writes the K matrix associated to these variables into 
				/// a global 'storage' matrix, at the offsets of variables. 
				/// Most solvers do not need this: the sparse 'storage' matrix is used for testing, for
				/// direct solvers, for dumping full matrix to Matlab for checks, etc.
	virtual void Build_K(ChSparseMatrix& storage, bool add=true);

};




} // END_OF_NAMESPACE____



#endif  // END of ChLcpKstiffnessGeneric.h
