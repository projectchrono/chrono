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
// File authors: Alessandro Tasora

#ifndef CHCONTINUUMPOISSON3D_H
#define CHCONTINUUMPOISSON3D_H


#include "ChApiFEM.h"
#include "physics/ChContinuumMaterial.h"


namespace chrono
{
namespace fem
{


/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0

class ChApiFem ChContinuumPoisson3D : public ChContinuumMaterial
{
protected:

	ChMatrixDynamic<> ConstitutiveMatrix;		// constitutive matrix


public:

			
	ChContinuumPoisson3D()
			{
				ConstitutiveMatrix.Resize(3,3);
				ConstitutiveMatrix.FillDiag(1.0);
			}

	virtual ~ChContinuumPoisson3D() {};
	

			/// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
	ChMatrixDynamic<>& Get_ConstitutiveMatrix () {return ConstitutiveMatrix;}

			/// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
	virtual double Get_DtMultiplier () {return 0;}

};




}//___end of namespace fem___
}//___end of namespace chrono___

#endif
