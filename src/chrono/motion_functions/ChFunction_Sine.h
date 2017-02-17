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

#ifndef CHFUNCT_SINE_H
#define CHFUNCT_SINE_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// SINE FUNCTION:
/// y = sin (phase + w*x )     w=2*PI*freq

class ChApi ChFunction_Sine : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Sine)

  private:
    double amp;
    double phase;
    double freq;
    double w;

  public:
    ChFunction_Sine() : amp(1), phase(0), freq(1), w(2 * CH_C_PI) {}
    ChFunction_Sine(double m_phase, double m_freq, double m_amp)
        : amp(m_amp), phase(m_phase), freq(m_freq), w(2 * CH_C_PI * m_freq) {}
    ChFunction_Sine(const ChFunction_Sine& other);
    ~ChFunction_Sine() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Sine* Clone() const override { return new ChFunction_Sine(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_SINE; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_phase(double m_phase) { phase = m_phase; };
    void Set_freq(double m_freq) {
        freq = m_freq;
        w = 2 * CH_C_PI * freq;
    }
    void Set_w(double m_w) {
        w = m_w;
        freq = w / (2 * CH_C_PI);
    }
    void Set_amp(double m_amp) { amp = m_amp; }

    double Get_phase() const { return phase; }
    double Get_freq() const { return freq; }
    double Get_w() const { return w; }
    double Get_amp() const { return amp; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Sine>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(amp);
        marchive << CHNVP(phase);
        marchive << CHNVP(freq);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Sine>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(amp);
        marchive >> CHNVP(phase);
        marchive >> CHNVP(freq);
    }
};

}  // end namespace chrono

#endif
