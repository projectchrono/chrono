//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

    

#include "physics/ChBodyFrame.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{





void ChBodyFrame::To_abs_forcetorque(const ChVector<>& force,
                                const ChVector<>& appl_point,
                                int               local,
                                ChVector<>&       resultforce,
                                ChVector<>&       resulttorque)
{
    if (local)
    {
        // local space
		ChVector<> mforce_abs = TrasformDirectionLocalToParent(force);
        resultforce = mforce_abs;
		resulttorque = Vcross (TrasformDirectionLocalToParent(appl_point), mforce_abs) ;
    }
    else
    {
        // absolute space
        resultforce = force;
        resulttorque = Vcross (Vsub(appl_point, coord.pos), force) ;
    }
}
void ChBodyFrame::To_abs_torque(const ChVector<>& torque,
                           int               local,
                           ChVector<>&       resulttorque)
{
    if (local)
    {
        // local space
		resulttorque = this->TrasformDirectionLocalToParent(torque);
    }
    else
    {
        // absolute space
        resulttorque = torque;
    }
}



} // END_OF_NAMESPACE____


/////////////////////
