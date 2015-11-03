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

#ifndef CHFUNCT_MATLAB_H
#define CHFUNCT_MATLAB_H

//////////////////////////////////////////////////
//
//   ChFunction_Matlab.h
//
//   Function objects,
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Base.h"

namespace chrono {

#define FUNCT_MATLAB 13

#define CHF_MATLAB_STRING_LEN 200

/// MATLAB FUNCTION:
/// y = matlab evaluation of function y=f(x)
///

class ChApi ChFunction_Matlab : public ChFunction {
    CH_RTTI(ChFunction_Matlab, ChFunction);

  private:
    char mat_command[CHF_MATLAB_STRING_LEN];  // matlab command
  public:
    ChFunction_Matlab();
    ~ChFunction_Matlab(){};
    void Copy(ChFunction_Matlab* source);
    ChFunction* new_Duplicate();

    void Set_Command(const char* m_command) { strcpy(mat_command, m_command); };
    char* Get_Command() { return mat_command; };

    double Get_y(double x);
    double Get_y_dx(double x) {
        return ((Get_y(x + BDF_STEP_HIGH) - Get_y(x)) / BDF_STEP_HIGH);
    }  //((Get_y(x+BDF_STEP_VERYLOW) - Get_y(x)) / BDF_STEP_VERYLOW);};  // return 0;
    double Get_y_dxdx(double x) { return ((Get_y_dx(x + BDF_STEP_HIGH) - Get_y_dx(x)) / BDF_STEP_HIGH); };  // return 0;

    int Get_Type() { return (FUNCT_MATLAB); }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(mat_command);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(mat_command);
    }

};

}  // END_OF_NAMESPACE____

#endif
