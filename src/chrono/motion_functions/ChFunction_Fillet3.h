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

#ifndef CHFUNCT_FILLET3_H
#define CHFUNCT_FILLET3_H

//////////////////////////////////////////////////
//
//   ChFunction_Fillet3.h
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

#define FUNCT_FILLET3 10

/// A CUBIC FILLET (cubic poly with C0 C1 boundary conditions)
///  - y1 = y at the beginning
///  - dy1 = y' at the beginning
///  - y2 = y at the end
///  - dy2 = y' at the end

class ChApi ChFunction_Fillet3 : public ChFunction {
    CH_RTTI(ChFunction_Fillet, ChFunction);

  private:
    double end;
    double y1;
    double y2;
    double dy1;
    double dy2;

    double c1, c2, c3, c4;  // used internally...

  public:
    ChFunction_Fillet3() {
        y1 = y2 = dy1 = dy2 = c1 = c2 = c3 = c4 = 0;
        end = 1.0;
    }
    ~ChFunction_Fillet3(){};
    void Copy(ChFunction_Fillet3* source);
    ChFunction* new_Duplicate();

    void Set_end(double m_end) {
        if (m_end < 0)
            m_end = 0;
        end = m_end;
        SetupCoefficients();
    }
    double Get_end() { return end; }

    int SetupCoefficients();

    void Set_y1(double my1) {
        y1 = my1;
        SetupCoefficients();
    }
    void Set_y2(double my2) {
        y2 = my2;
        SetupCoefficients();
    }
    void Set_dy1(double mdy1) {
        dy1 = mdy1;
        SetupCoefficients();
    }
    void Set_dy2(double mdy2) {
        dy2 = mdy2;
        SetupCoefficients();
    }

    double Get_y1() { return y1; }
    double Get_y2() { return y2; }
    double Get_dy1() { return dy1; }
    double Get_dy2() { return dy2; }

    double Get_y(double x);
    double Get_y_dx(double x);
    double Get_y_dxdx(double x);

    void Estimate_x_range(double& xmin, double& xmax) {
        xmin = 0.0;
        xmax = end;
    };
    int Get_Type() { return (FUNCT_FILLET3); }

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
        marchive << CHNVP(end);
        marchive << CHNVP(y1);
        marchive << CHNVP(y2);
        marchive << CHNVP(dy1);
        marchive << CHNVP(dy2);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(end);
        marchive >> CHNVP(y1);
        marchive >> CHNVP(y2);
        marchive >> CHNVP(dy1);
        marchive >> CHNVP(dy2);
        SetupCoefficients();
    }

};

}  // END_OF_NAMESPACE____

#endif
