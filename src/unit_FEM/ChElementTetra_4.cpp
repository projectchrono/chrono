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

#include "ChElementTetra_4.h"


namespace chrono
{
namespace fem
{



ChElementTetra_4::ChElementTetra_4()
{

	nodes.resize(4);
	this->MatrB.Resize(6,12);
	this->StiffnessMatrix.Resize(12,12);
}


ChElementTetra_4::~ChElementTetra_4()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
