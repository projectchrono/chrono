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


#include "lcp/ChLcpKstiffnessGeneric.h"
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

class ChApiFem ChElementGeneric : public ChElementBase
{
protected:
	ChLcpKstiffnessGeneric Kmatr;

public:

	ChElementGeneric() {};
	virtual ~ChElementGeneric() {};

				/// Access the proxy to stiffness, for sparse LCP solver
	ChLcpKstiffnessGeneric& Kstiffness() {return Kmatr;}


			//
			// Functions for interfacing to the LCP solver
			//

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but inherited classes must specialize this.
	virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor)
				{
					mdescriptor.InsertKstiffness(&Kmatr);
				}

				/// Adds the current stiffness K and damping R and mass M matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K, R, M matrices are load with scaling 
				/// values Kfactor, Rfactor, Mfactor. 
	virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
				{
					this->ComputeKRMmatricesGlobal(*this->Kmatr.Get_K(), Kfactor, Rfactor, Mfactor);
				}

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) 
				{
					// (This is a default (unoptimal) book keeping so that in children classes you can avoid 
					// implementing this VariablesFbLoadInternalForces function, unless you need faster code)
					ChMatrixDynamic<> mFi(this->GetNcoords(), 1);
					this->ComputeInternalForces(mFi);
					mFi.MatrScale(factor);
					int stride = 0;
					for (int in=0; in < this->GetNnodes(); in++)
					{
						int nodedofs = GetNodeN(in)->Get_ndof();
						GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
						stride += nodedofs;
					}
				};

				/// Adds M*q (internal masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	virtual void VariablesFbIncrementMq() 
				{
					// This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid 
					// implementing this VariablesFbIncrementMq function, unless you need faster code)

					ChMatrixDynamic<> mMi(this->GetNcoords(), this->GetNcoords());
					this->ComputeKRMmatricesGlobal(mMi, 0, 0, 1.0); // fill M mass matrix 
					
					ChMatrixDynamic<> mqi(this->GetNcoords(), 1);
					int stride = 0;
					for (int in=0; in < this->GetNnodes(); in++)
					{
						int nodedofs = GetNodeN(in)->Get_ndof();
						mqi.PasteMatrix(&GetNodeN(in)->Variables().Get_qb(), stride, 0);
						stride += nodedofs;
					}

					ChMatrixDynamic<> mFi(this->GetNcoords(), 1);
					mFi.MatrMultiply(mMi, mqi);

					stride = 0;
					for (int in=0; in < this->GetNnodes(); in++)
					{
						int nodedofs = GetNodeN(in)->Get_ndof();
						GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
						stride += nodedofs;
					}

				}

};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






