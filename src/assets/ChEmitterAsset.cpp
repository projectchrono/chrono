//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "assets/ChEmitterAsset.h"
#include "physics/ChBody.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChEmitterAsset> a_registration_ChEmitterAsset;


void ChEmitterAsset::Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) {

    ChSystem* msys = updater->GetSystem();
    
    if (!msys) 
        return;

    double mt = msys->GetChTime();

    if (mt < this->last_t)
        last_t = mt;
    
    double dt = mt - last_t;
    last_t = mt;

    if (dt == 0)
        return;
    //***TODO*** a better way to deduce dt, ex. by using flags in Update(),
    // to know if Update really follows a t+dt update, etc. 

    // Create the particles!

    ChFrameMoving<> mframe(coords);

    // special case: the owner is a body? so use also speed info
    if(ChBody* mbody = dynamic_cast<ChBody*>(updater)) {

        ChFrameMoving<> bodyframe = mbody->GetFrame_REF_to_abs();
        ChFrameMoving<> relcoords;
        bodyframe.TransformParentToLocal(mframe, relcoords);
        relcoords.SetCoord_dt(CSYSNULL);
        relcoords.SetCoord_dtdt(CSYSNULL);
        bodyframe.TransformLocalToParent(relcoords, mframe);
    }

    this->memitter.EmitParticles(*msys, dt, mframe);
}


}  // END_OF_NAMESPACE____
