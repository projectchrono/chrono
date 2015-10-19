//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Ramp.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Ramp.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Ramp> a_registration_ramp;

void ChFunction_Ramp::Copy(ChFunction_Ramp* source) {
    Set_y0(source->y0);
    Set_ang(source->ang);
}

ChFunction* ChFunction_Ramp::new_Duplicate() {
    ChFunction_Ramp* m_func;
    m_func = new ChFunction_Ramp;
    m_func->Copy(this);
    return (m_func);
}


}  // END_OF_NAMESPACE____

// eof
