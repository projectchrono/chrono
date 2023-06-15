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

#ifndef CHFUNCT_MOCAP_H
#define CHFUNCT_MOCAP_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Motion capture (sample) function:
/// `y = (linear interpolated array of samples)`
class ChApi ChFunction_Mocap : public ChFunction {
  private:
    ChArray<> array_y;
    ChArray<> array_y_dt;
    ChArray<> array_y_dtdt;

    double samp_freq;
    int samples;
    double timetot;

  public:
    ChFunction_Mocap();
    ChFunction_Mocap(int m_samples, double freq);
    ChFunction_Mocap(const ChFunction_Mocap& other);
    ~ChFunction_Mocap() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Mocap* Clone() const override { return new ChFunction_Mocap(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_MOCAP; }
    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_samp_freq(double m_fr);
    void Set_samples(int m_samples);

    double Get_samp_freq() const { return samp_freq; }
    int Get_samples() const { return samples; }
    double Get_timetot() const { return ((double)samples / samp_freq); }
    double Get_timeslice() const { return (1 / samp_freq); }

    const ChArray<>& Get_array_y() const { return array_y; }
    const ChArray<>& Get_array_y_dt() const { return array_y_dt; }
    const ChArray<>& Get_array_y_dtdt() const { return array_y_dtdt; }

    void Set_array_y(const ChArray<>& m_array_y);
    void Set_array_y_dt(const ChArray<>& m_array_y_dt);      // *** TO DO
    void Set_array_y_dtdt(const ChArray<>& m_array_y_dtdt);  // *** TO DO

    bool Parse_array_AOA();    // *** TO DO
    bool Parse_array_Elite();  // *** TO DO

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    void Compute_array_dt(const ChArray<>& array_A, ChArray<>& array_A_dt) const;
    double LinInterp(const ChArray<>& array, double x, double x_max) const;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_Mocap, 0)

}  // end namespace chrono

#endif
