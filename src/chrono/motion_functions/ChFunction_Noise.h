// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_NOISE_H
#define CHFUNCT_NOISE_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Noise function:
/// y = multi-octave noise with cubic interpolation
class ChApi ChFunction_Noise : public ChFunction {
  private:
    double amp;
    double freq;
    double amp_ratio;
    int octaves;

  public:
    ChFunction_Noise() : amp(1), freq(1), amp_ratio(0.5), octaves(2) {}
    ChFunction_Noise(const ChFunction_Noise& other);
    ~ChFunction_Noise() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Noise* Clone() const override { return new ChFunction_Noise(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_NOISE; }

    virtual double Get_y(double x) const override;

    void Set_Amp(double mamp) { amp = mamp; }
    double Get_Amp() const { return amp; }

    void Set_Freq(double mf) { freq = mf; }
    double Get_Freq() const { return freq; }

    void Set_AmpRatio(double ma) { amp_ratio = ma; }
    double Get_AmpRatio() const { return amp_ratio; }

    void Set_Octaves(int mo) { octaves = mo; }
    int Get_Octaves() const { return octaves; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_Noise, 0)

}  // end namespace chrono

#endif
