#ifndef CHELEMENTBASE_H
#define CHELEMENTBASE_H

//////////////////////////////////////////////////
//
//   ChElementBase.h
//
//   Finite elements
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChApidll.h"
#include "core/ChMath.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "fem/ChNodeFEMbase.h"


namespace chrono
{
namespace fem
{




/// Base class for all finite elements, that can be
/// used in the ChMesh physics item.

class ChApi ChElementBase
{
protected:

public:

	ChElementBase() {};
	virtual ~ChElementBase() {};

	virtual int GetNcoords() =0;
	virtual int GetNnodes() =0;
	virtual ChNodeFEMbase* GetNodeN(int n) =0;
	

			//
			// FEM functions
			//

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor. Corotational
				/// elements can take the local Kl & Rl matrices and rotate them.
				/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeKRmatricesGlobal (ChMatrix<>& H, double Kfactor, double Rfactor=0)  = 0;

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor.
				/// This is usually called only once in the simulation. 
				/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeKRmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0) = 0;

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector, whith n.rows = n.of dof of element.
				/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi) = 0;

				/// Setup (optional). Precompute matrices that do not change during the 
				/// simulation, such as the local stiffness of each element, if needed, etc.
	virtual void Setup() {};


			//
			// Functions for interfacing to the LCP solver
			// 

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but inherited classes must specialize this.
	virtual void InjectKmatrices(ChLcpSystemDescriptor& mdescriptor) =0;

				/// Adds the current stiffness K and damping R matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K and R matrices are load with scaling 
				/// values Kfactor and Rfactor. 
	virtual void KmatricesLoad(double Kfactor, double Rfactor) =0;

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) {};


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






