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

#ifndef CHFUNCT_DERIVE_H
#define CHFUNCT_DERIVE_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

/// Derivative of a function: y = df/dx
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.

class ChApi ChFunction_Derive : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Derive);

  private:
    std::shared_ptr<ChFunction> fa;
    int order;  ///< 1= derive one time, 2= two times, etc.

  public:
    ChFunction_Derive() : order(1) {}
    ChFunction_Derive(const ChFunction_Derive& other);
    ~ChFunction_Derive() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Derive* Clone() const override { return new ChFunction_Derive(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_DERIVE; }

    virtual double Get_y(double x) const override;

    void Set_order(int m_order) { order = m_order; }
    int Get_order() { return order; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Derive>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(order);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Derive>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(order);
    }
};

CH_CLASS_VERSION(ChFunction_Derive,0)


}  // END_OF_NAMESPACE____

#endif
