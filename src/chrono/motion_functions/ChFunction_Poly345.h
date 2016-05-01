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

#ifndef CHFUNCT_POLY345_H
#define CHFUNCT_POLY345_H

//////////////////////////////////////////////////
//
//   ChFunction_Poly345.h
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

#define FUNCT_POLY345 9

/// A RAMP, AS 3-4-5 POLYNOMIAL FUNCTION:
///   - h   = height, amount of displacement
///   - end = duration of motion,

class ChApi ChFunction_Poly345 : public ChFunction {
    CH_RTTI(ChFunction_Poly345, ChFunction);

  private:
    double h;
    double end;

  public:
    ChFunction_Poly345() {
        h = 1;
        end = 1;
    }
    ChFunction_Poly345(double m_h, double m_end) {
        h = m_h;
        Set_end(m_end);
    };
    ~ChFunction_Poly345(){};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(ChFunction_Poly345* source);
#pragma GCC diagnostic pop
    ChFunction* new_Duplicate() override;

    void Set_end(double m_end) {
        if (m_end < 0)
            m_end = 0;
        end = m_end;
    }
    void Set_h(double m_h) { h = m_h; }

    double Get_end() { return end; }
    double Get_h() { return h; }

    double Get_y(double x) override;
    double Get_y_dx(double x) override;
    double Get_y_dxdx(double x) override;

    double Get_Ca_pos() override { return 5.8; }
    double Get_Ca_neg() override { return 5.8; }
    double Get_Cv() override { return 1.9; }

    void Estimate_x_range(double& xmin, double& xmax) override {
        xmin = 0.0;
        xmax = end;
    }

    int Get_Type() override { return (FUNCT_POLY345); }

    OPT_VARIABLES_START
    "h", "end", OPT_VARIABLES_END

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(h);
        marchive << CHNVP(end);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override
    {
        // version number
        // int version =
        marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(h);
        marchive >> CHNVP(end);
    }


};

}  // END_OF_NAMESPACE____

#endif
