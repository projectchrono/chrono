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
//   ChFunction_Const.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Const> a_registration_const;

void ChFunction_Const::Copy(ChFunction_Const* source) {
    Set_yconst(source->C);
}

ChFunction* ChFunction_Const::new_Duplicate() {
    ChFunction_Const* m_func;
    m_func = new ChFunction_Const;
    m_func->Copy(this);
    return (m_func);
}


}  // END_OF_NAMESPACE____

// eof
