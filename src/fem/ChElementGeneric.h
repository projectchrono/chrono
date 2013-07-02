#ifndef CHELEMENTGENERIC_H
#define CHELEMENTGENERIC_H

//////////////////////////////////////////////////
//
//   ChElementGeneric.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "lcp/ChLcpKstiffnessGeneric.h"
#include "lcp/ChLcpVariablesNode.h"
#include "fem/ChElementBase.h"


namespace chrono
{
namespace fem
{



/// Class for all elements whose stiffness matrix can be seen
/// as a NxN block-matrix to be splitted between N nodes.
/// Helps reducing the complexity of inherited FEM elements because
/// it implements some bookkeeping for the interface with LCP solver.

class ChApi ChElementGeneric : public ChElementBase
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
	virtual void InjectKmatrices(ChLcpSystemDescriptor& mdescriptor)
				{
					mdescriptor.InsertKstiffness(&Kmatr);
				}

				/// Adds the current stiffness K and damping R matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K and R matrices are load with scaling 
				/// values Kfactor and Rfactor. 
	virtual void KmatricesLoad(double Kfactor, double Rfactor)
				{
					this->ComputeKRmatricesGlobal(*this->Kmatr.Get_K(), Kfactor, Rfactor);
				}

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) 
				{
					// (This is a default (unoptimal) book keeping so that in children classes you can avoid 
					// implementing this VariablesFbLoadInternalForces function, unless you need faster code)
					ChMatrixDynamic<> mFi(this->GetNcoords(), 1);
					this->ComputeInternalForces(mFi);
					int stride = 0;
					for (int in=0; in < this->GetNnodes(); in++)
					{
						int nodedofs = GetNodeN(in)->Get_ndof();
						GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
						stride += nodedofs;
					}
				};

};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






