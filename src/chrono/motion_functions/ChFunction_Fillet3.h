// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_FILLET3_H
#define CHFUNCT_FILLET3_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Cubic fillet function (cubic poly with C0 C1 boundary conditions).
///
///  - y1 = y at the beginning
///  - dy1 = y' at the beginning
///  - y2 = y at the end
///  - dy2 = y' at the end

class ChApi ChFunction_Fillet3 : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Fillet)

  private:
    double end;
    double y1;
    double y2;
    double dy1;
    double dy2;

    double c1, c2, c3, c4;  // used internally...

  public:
    ChFunction_Fillet3() : y1(0), y2(0), dy1(0), dy2(0), end(1), c1(0), c2(0), c3(0), c4(0) {}
    ChFunction_Fillet3(const ChFunction_Fillet3& other);
    ~ChFunction_Fillet3() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Fillet3* Clone() const override { return new ChFunction_Fillet3(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_FILLET3; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_end(double m_end) {
        if (m_end < 0)
            m_end = 0;
        end = m_end;
        SetupCoefficients();
    }

    double Get_end() { return end; }

    void SetupCoefficients();

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

    virtual void Estimate_x_range(double& xmin, double& xmax) const override {
        xmin = 0.0;
        xmax = end;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Fillet3>();
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
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Fillet3>();
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

}  // Eend namespace chrono

#endif
