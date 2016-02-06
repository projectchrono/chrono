//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLOADER_H
#define CHLOADER_H


#include "core/ChVectorDynamic.h"
#include "physics/ChLoadable.h"
#include "chrono/core/ChQuadrature.h"

namespace chrono {



/// Class for loads applied to a single ChLoadable object.
/// Loads can be forces, torques, pressures, thermal loads, etc. depending
/// on the loaded ChLoadable object. For example if the load references
/// a ChBody, the load is a wrench (force+torque), for a tetrahedron FE it is a force, etc.
/// Objects of this class must be capable of computing the generalized load Q from 
/// the load F.

class ChLoader  {
public:
    ChVectorDynamic<> Q;

    virtual void ComputeQ( ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) = 0;

    virtual std::shared_ptr<ChLoadable> GetLoadable() =0;

    virtual bool IsStiff() {return false;}
};







}  // END_OF_NAMESPACE____

#endif  
