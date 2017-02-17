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

#ifndef CHFUNCT_MIRROR_H
#define CHFUNCT_MIRROR_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

/// Mirror function:
///    y = __/\__
///
/// Mirrors a function about a vertical axis.

class ChApi ChFunction_Mirror : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Mirror)

  private:
    std::shared_ptr<ChFunction> fa;
    double mirror_axis;  ///< symmetry axis position on x

  public:
    ChFunction_Mirror();
    ChFunction_Mirror(const ChFunction_Mirror& other);
    ~ChFunction_Mirror() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Mirror* Clone() const override { return new ChFunction_Mirror(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_MIRROR; }

    virtual double Get_y(double x) const override;

    void Set_mirror_axis(double m_axis) { mirror_axis = m_axis; }
    double Get_mirror_axis() { return mirror_axis; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Mirror>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(mirror_axis);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Mirror>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(mirror_axis);
    }
};

CH_CLASS_VERSION(ChFunction_Mirror,0)

}  // end namespace chrono

#endif
