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

#ifndef CHNODEFEABASE_H
#define CHNODEFEABASE_H


#include "ChApiFEA.h"
#include "physics/ChNodeBase.h"


namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

// Forward
class ChMesh;


/// Base class for a generic finite element node
/// that can be stored in ChMesh containers.
/// Children classes must implement specialized versions.
class ChApiFea ChNodeFEAbase  :  public virtual ChNodeBase
{
public:

    ChNodeFEAbase() {
    }

				/// Set the rest position as the actual position.
	virtual void Relax () =0;

				/// Reset to no speed and acceleration.
	virtual void SetNoSpeedNoAcceleration () =0;

				/// Sets the 'fixed' state of the node. 
				/// If true, its current field value is not changed by solver.
	virtual void SetFixed  (bool mev) =0;  
				/// Gets the 'fixed' state of the node. 
				/// If true, its current field value is not changed by solver.
    virtual bool GetFixed()  =0;  


protected:

    //
    // DATA
    //

};

/// @} fea_nodes

} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






