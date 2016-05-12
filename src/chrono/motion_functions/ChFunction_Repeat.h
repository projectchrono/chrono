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

#ifndef CHFUNCT_REPEAT_H
#define CHFUNCT_REPEAT_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

/// Repeat function:
///     y = __/__/__/
///
/// Repeats a 'window' of a function, periodically.

class ChApi ChFunction_Repeat : public ChFunction {
    CH_RTTI(ChFunction_Repeat, ChFunction);

  private:
    double window_start;   ///< window begin position
    double window_length;  ///< window length
    std::shared_ptr<ChFunction> fa;

  public:
    ChFunction_Repeat();
    ChFunction_Repeat(const ChFunction_Repeat& other);
    ~ChFunction_Repeat() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Repeat* Clone() const override { return new ChFunction_Repeat(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_REPEAT; }

    virtual double Get_y(double x) const override;

    void Set_window_start(double m_v) { window_start = m_v; }
    double Get_window_start() const { return window_start; }

    void Set_window_length(double m_v) { window_length = m_v; }
    double Get_window_length() const { return window_length; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(window_start);
        marchive << CHNVP(window_length);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(window_start);
        marchive >> CHNVP(window_length);
    }
};

}  // end namespace chrono

#endif
