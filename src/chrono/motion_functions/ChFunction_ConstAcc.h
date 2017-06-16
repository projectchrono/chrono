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

#ifndef CHFUNCT_CONSTACC_H
#define CHFUNCT_CONSTACC_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Constant acceleration function:
///
///   h = height, amount of displacement
///   end = duration of motion,
///   av  = fraction of 1st acceleration end  (0..1)
///   aw  = fraction of 2nd acceleration start (0..1) , with aw>av;

class ChApi ChFunction_ConstAcc : public ChFunction {

    CH_FACTORY_TAG(ChFunction_ConstAcc)

  private:
    double h;
    double av;
    double aw;
    double end;

  public:
    ChFunction_ConstAcc() : h(1), av(0.5), aw(0.5), end(1) {}
    ChFunction_ConstAcc(double m_h, double m_av, double m_aw, double m_end);
    ChFunction_ConstAcc(const ChFunction_ConstAcc& other);
    ~ChFunction_ConstAcc() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_ConstAcc* Clone() const override { return new ChFunction_ConstAcc(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_CONSTACC; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_end(double m_end) {
        if (m_end < 0)
            m_end = 0;
        end = m_end;
    }

    void Set_av(double m_av) {
        if (m_av < 0)
            m_av = 0;
        if (m_av > 1)
            m_av = 1;
        av = m_av;
        if (av > aw)
            av = aw;
    }
    void Set_aw(double m_aw) {
        if (m_aw < 0)
            m_aw = 0;
        if (m_aw > 1)
            m_aw = 1;
        aw = m_aw;
        if (aw < av)
            aw = av;
    }
    void Set_h(double m_h) { h = m_h; }
    void Set_avw(double m_av, double m_aw) {
        av = 0;
        aw = 1;
        Set_av(m_av);
        Set_aw(m_aw);
    }

    double Get_end() const { return end; }
    double Get_av() const { return av; }
    double Get_aw() const { return aw; }
    double Get_h() const { return h; }

    virtual double Get_Ca_pos() const override;
    virtual double Get_Ca_neg() const override;
    virtual double Get_Cv() const override;

    virtual void Estimate_x_range(double& xmin, double& xmax) const override {
        xmin = 0.0;
        xmax = end;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_ConstAcc>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(h);
        marchive << CHNVP(end);
        marchive << CHNVP(aw);
        marchive << CHNVP(av);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_ConstAcc>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(h);
        marchive >> CHNVP(end);
        marchive >> CHNVP(aw);
        marchive >> CHNVP(av);
    }
};

CH_CLASS_VERSION(ChFunction_ConstAcc,0)

}  // end namespace chrono

#endif
