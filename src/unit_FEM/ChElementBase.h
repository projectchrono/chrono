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

#ifndef CHELEMENTBASE_H
#define CHELEMENTBASE_H


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
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M, 
				/// scaled by Mfactor. 
				/// Corotational elements can take the local Kl & Rl matrices and rotate them.
				/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeKRMmatricesGlobal (ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) = 0;

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector, whith n.rows = n.of dof of element.
				/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi) = 0;

				/// Initial setup: This is used mostly to precompute matrices 
				/// that do not change during the simulation, i.e. the local 
				/// stiffness of each element, if any, the mass, etc.
	virtual void SetupInitial() {};

				/// Update: this is called at least at each time step. If the
				/// element has to keep updated some auxiliary data, such as the rotation
				/// matrices for corotational approach, this is the proper place.
	virtual void Update() {};


			//
			// Functions for interfacing to the LCP solver
			// 

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but inherited classes must specialize this.
	virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) =0;

				/// Adds the current stiffness K and damping R and mass M matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K, R, M matrices are added with scaling 
				/// values Kfactor, Rfactor, Mfactor.  
	virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) =0;

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) {};


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






