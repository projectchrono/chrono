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

#ifndef CHFUNCT_POLY345_H
#define CHFUNCT_POLY345_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Ramp function, as a 3-4-5 polynomial:
///
///   - h   = height, amount of displacement
///   - end = duration of motion,

class ChApi ChFunction_Poly345 : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Poly345)

  private:
    double h;
    double end;

  public:
    ChFunction_Poly345() : h(1), end(1) {}
    ChFunction_Poly345(double m_h, double m_end);
    ChFunction_Poly345(const ChFunction_Poly345& other);
    ~ChFunction_Poly345(){}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Poly345* Clone() const override { return new ChFunction_Poly345(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_POLY345; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_end(double m_end) {
        if (m_end < 0)
            m_end = 0;
        end = m_end;
    }
    void Set_h(double m_h) { h = m_h; }

    double Get_end() const { return end; }
    double Get_h() const { return h; }

    virtual double Get_Ca_pos() const override { return 5.8; }
    virtual double Get_Ca_neg() const override { return 5.8; }
    virtual double Get_Cv() const override { return 1.9; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override {
        xmin = 0.0;
        xmax = end;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Poly345>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(h);
        marchive << CHNVP(end);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Poly345>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(h);
        marchive >> CHNVP(end);
    }
};

CH_CLASS_VERSION(ChFunction_Poly345,0)

}  // end namespace chrono

#endif
