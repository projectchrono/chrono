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

#ifndef CHELEMENT3D_H
#define CHELEMENT3D_H

#include "ChElementGeneric.h"
#include "ChGaussIntegrationRule.h"
#include "ChPolarDecomposition.h"
#include "ChMatrixCorotation.h"

namespace chrono{
	namespace fem{


		/// Class for all 3-Dimensional elements. 

class ChApiFem ChElement3D : public ChElementGeneric
{
protected:
	double Volume;

public:

	double GetVolume() {return Volume;}
};




	}//___end of namespace fem___
}//___end of namespace chrono___

#endif