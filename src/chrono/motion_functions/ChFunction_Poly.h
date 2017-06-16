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

#ifndef CHFUNCT_POLY_H
#define CHFUNCT_POLY_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// POLYNOMIAL FUNCTION:
/// y = a + bx + cx^2 + dx^3 + ...

class ChApi ChFunction_Poly : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Poly)

  private:
    static const int POLY_COEFF_ARRAY = 6;
    double coeff[POLY_COEFF_ARRAY];  ///< vector of coefficients
    int order;                       ///< 0= const, 1= linear, etc...

  public:
    ChFunction_Poly();
    ChFunction_Poly(const ChFunction_Poly& other);
    ~ChFunction_Poly() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Poly* Clone() const override { return new ChFunction_Poly(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_POLY; }
    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void Set_coeff(double m_coeff, int m_ind) {
        if (m_ind >= POLY_COEFF_ARRAY) {
            return;
        }
        coeff[m_ind] = m_coeff;
    }

    void Set_order(int m_order) {
        if (m_order >= POLY_COEFF_ARRAY) {
            m_order = (POLY_COEFF_ARRAY - 1);
        }
        order = m_order;
    }

    double Get_coeff(int m_ind) const {
        if (m_ind >= POLY_COEFF_ARRAY) {
            return 0;
        }
        return coeff[m_ind];
    }

    int Get_order() const { return order; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Poly>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(coeff);
        marchive << CHNVP(order);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Poly>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(coeff);
        marchive >> CHNVP(order);
    }
};

CH_CLASS_VERSION(ChFunction_Poly,0)

}  // end namespace chrono

#endif
