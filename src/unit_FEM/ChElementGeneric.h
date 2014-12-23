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
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHELEMENTGENERIC_H
#define CHELEMENTGENERIC_H


#include "lcp/ChLcpKblockGeneric.h"
#include "lcp/ChLcpVariablesNode.h"
#include "ChElementBase.h"


namespace chrono
{
namespace fem
{



/// Class for all elements whose stiffness matrix can be seen
/// as a NxN block-matrix to be splitted between N nodes.
/// Helps reducing the complexity of inherited FEM elements because
/// it implements some bookkeeping for the interface with LCP solver.
/// This means that most FEM elements inherited from ChElementGeneric
/// need to implement at most the following two fundamental methods: 
///	ComputeKRMmatricesGlobal(), ComputeInternalForces()

class ChApiFem ChElementGeneric : public ChElementBase
{
protected:
	ChLcpKblockGeneric Kmatr;

public:

	ChElementGeneric() {};
	virtual ~ChElementGeneric() {};

				/// Access the proxy to stiffness, for sparse LCP solver
	ChLcpKblockGeneric& Kstiffness() {return Kmatr;}

			//
			// Functions for interfacing to the state bookkeeping
			//

				/// (This is a default (a bit unoptimal) book keeping so that in children classes you can avoid 
				/// implementing this EleIntLoadResidual_F function, unless you need faster code)
	virtual void EleIntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );

				/// (This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid 
				/// implementing this EleIntLoadResidual_Mv function, unless you need faster code.)
	virtual void EleIntLoadResidual_Mv(const unsigned int off,	ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c);

			//
			// Functions for interfacing to the LCP solver
			//

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKblock in this object (for further passing it to a LCP solver)
	virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor)
				{
					mdescriptor.InsertKblock(&Kmatr);
				}

				/// Adds the current stiffness K and damping R and mass M matrices in encapsulated
				/// ChLcpKblock item(s), if any. The K, R, M matrices are load with scaling 
				/// values Kfactor, Rfactor, Mfactor. 
	virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
				{
					this->ComputeKRMmatricesGlobal(*this->Kmatr.Get_K(), Kfactor, Rfactor, Mfactor);
				}

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
				/// (This is a default (a bit unoptimal) book keeping so that in children classes you can avoid 
				/// implementing this VariablesFbLoadInternalForces function, unless you need faster code)
	virtual void VariablesFbLoadInternalForces(double factor=1.);


				/// Adds M*q (internal masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
				/// (This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid 
				/// implementing this VariablesFbIncrementMq function, unless you need faster code.)
	virtual void VariablesFbIncrementMq();


};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






