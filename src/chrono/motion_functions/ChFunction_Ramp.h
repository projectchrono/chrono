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

#ifndef CHFUNCT_RAMP_H
#define CHFUNCT_RAMP_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Linear function (like a straight ramp):
/// y = y0 + x * speed

class ChApi ChFunction_Ramp : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Ramp)

  private:
    double y0;
    double ang;

  public:
    ChFunction_Ramp() : y0(0), ang(1) {}
    ChFunction_Ramp(double m_y0, double m_ang) : y0(m_y0), ang(m_ang) {}
    ChFunction_Ramp(const ChFunction_Ramp& other);
    ~ChFunction_Ramp() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Ramp* Clone() const override { return new ChFunction_Ramp(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_RAMP; }

    virtual double Get_y(double x) const override { return (y0 + (x * ang)); }
    virtual double Get_y_dx(double x) const override { return (ang); }
    virtual double Get_y_dxdx(double x) const override { return 0; }

    /// The value for x=0;
    void Set_y0(double m_y0) { y0 = m_y0; }
    double Get_y0() { return y0; }

    /// The angular coefficient.
    void Set_ang(double m_ang) { ang = m_ang; }
    double Get_ang() { return ang; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Ramp>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(y0);
        marchive << CHNVP(ang);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Ramp>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(y0);
        marchive >> CHNVP(ang);
    }
};

CH_CLASS_VERSION(ChFunction_Ramp,0)

}  // end namespace chrono

#endif
