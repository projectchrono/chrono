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

#ifndef CHNODEFEMBASE_H
#define CHNODEFEMBASE_H


#include "ChApiFEM.h"
#include "physics/ChNodeBase.h"


namespace chrono
{
namespace fem
{


/// Base class for a generic finite element node
/// that can be stored in ChMesh containers.
/// Children classes must implement specialized versions.

class ChApiFem ChNodeFEMbase  :  public chrono::ChNodeBase
{
public:

				/// Set the rest position as the actual position.
	virtual void Relax () =0;

				/// Sets the 'fixed' state of the node. 
				/// If true, its current field value is not changed by solver.
	void SetFixed  (bool mev) { Variables().SetDisabled(mev); }
				/// Gets the 'fixed' state of the node. 
				/// If true, its current field value is not changed by solver.
    bool GetFixed()  {return Variables().IsDisabled(); }

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






