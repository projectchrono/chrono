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
//   ChFunction_Sine.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Sine.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sine> a_registration_sine;

void ChFunction_Sine::Copy(ChFunction_Sine* source) {
    Set_phase(source->phase);
    Set_freq(source->freq);
    Set_amp(source->amp);
}

ChFunction* ChFunction_Sine::new_Duplicate() {
    ChFunction_Sine* m_func;
    m_func = new ChFunction_Sine;
    m_func->Copy(this);
    return (m_func);
}


}  // END_OF_NAMESPACE____

// eof
