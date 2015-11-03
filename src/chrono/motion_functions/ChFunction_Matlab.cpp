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
//   ChFunction_Matlab.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Matlab.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Matlab> a_registration_matlab;

ChFunction_Matlab::ChFunction_Matlab() {
    strcpy(this->mat_command, "x*2+x^2");
}

void ChFunction_Matlab::Copy(ChFunction_Matlab* source) {
    strcpy(this->mat_command, source->mat_command);
}

ChFunction* ChFunction_Matlab::new_Duplicate() {
    ChFunction_Matlab* m_func;
    m_func = new ChFunction_Matlab;
    m_func->Copy(this);
    return (m_func);
}

double ChFunction_Matlab::Get_y(double x) {
    double ret = 0;

#ifdef CH_MATLAB
    char m_eval_command[CHF_MATLAB_STRING_LEN + 20];

    // no function: shortcut!
    if (*this->mat_command == NULL)
        return 0.0;

    // set string as "x=[x];ans=[mat_command]"
    sprintf(m_eval_command, "x=%g;ans=%s;", x, this->mat_command);

    // EVAL string, retrieving y = "ans"
    ret = CHGLOBALS().Mat_Eng_Eval(m_eval_command);

#else
    ret = 0.0;
#endif

    return ret;
}


}  // END_OF_NAMESPACE____

// eof
