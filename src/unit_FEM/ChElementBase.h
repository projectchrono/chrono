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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChApidll.h"
#include "physics/ChContinuumMaterial.h"
#include "core/ChMath.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "ChNodeFEMbase.h"


namespace chrono
{
namespace fem
{




/// Base class for all finite elements, that can be
/// used in the ChMesh physics item.

class ChApiFem ChElementBase
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
	virtual void ComputeKRmatricesGlobal (ChMatrix<>& H, double Kfactor, double Rfactor=0) = 0;

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

				/// Initial setup: This is used mostly to precompute matrices 
				/// that do not change during the simulation, i.e. the local stiffness of each element, if any, etc.
	virtual void SetupInitial() {};


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






