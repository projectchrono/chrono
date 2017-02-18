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

#ifndef CHFUNCT_SIGMA_H
#define CHFUNCT_SIGMA_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Sigma function:
///   y = polynomial smooth ramp

class ChApi ChFunction_Sigma : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Sigma)

  private:
    double amp;
    double start;
    double end;

  public:
    ChFunction_Sigma() : amp(1), start(0), end(1) {}
    ChFunction_Sigma(double m_amp, double m_start, double m_end) : amp(m_amp), start(m_start), end(m_end) {}
    ChFunction_Sigma(const ChFunction_Sigma& other);
    ~ChFunction_Sigma() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Sigma* Clone() const override { return new ChFunction_Sigma(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_SIGMA; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_start(double m_start) { start = m_start; }
    void Set_end(double m_end) { end = m_end; }
    void Set_amp(double m_amp) { amp = m_amp; }
    double Get_start() const { return start; }
    double Get_end() const { return end; }
    double Get_amp() const { return amp; }

    virtual double Get_Ca_pos() const override { return 6.0; }
    virtual double Get_Ca_neg() const override { return 6.0; }
    virtual double Get_Cv() const override { return 1.5; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Sigma>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(amp);
        marchive << CHNVP(start);
        marchive << CHNVP(end);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Sigma>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(amp);
        marchive >> CHNVP(start);
        marchive >> CHNVP(end);
    }
};

CH_CLASS_VERSION(ChFunction_Sigma,0)

}  // end namespace chrono

#endif
