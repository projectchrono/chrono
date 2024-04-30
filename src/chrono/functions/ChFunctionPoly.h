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

#ifndef CHFUNCT_POLY_H
#define CHFUNCT_POLY_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Polynomial function.
/// `y = a + b*x + c*x^2 + d*x^3 + ...`
class ChApi ChFunctionPoly : public ChFunction {
  private:
    std::vector<double> m_coeffs;

  public:
    ChFunctionPoly();
    ChFunctionPoly(const ChFunctionPoly& other);
    ~ChFunctionPoly() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionPoly* Clone() const override { return new ChFunctionPoly(*this); }

    virtual Type GetType() const override { return ChFunction::Type::POLY; }
    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    /// Set the polynomial coefficients.
    /// The order of the polynome is equal to the size of the provided vector of coefficients
    void SetCoefficients(const std::vector<double>& coeffs) {
        if (coeffs.size() < 1)
            throw std::invalid_argument(
                "ChFunctionPoly::SetCoefficients: coefficients vector should have at least one element.");

        m_coeffs = coeffs;
    }

    /// Get the polynomial coefficients.
    std::vector<double> GetCoefficients() const { return m_coeffs; }

    /// Get the order of the polynomial.
    /// This equals the number of coefficients.
    size_t GetOrder() const { return m_coeffs.size(); }

    /// Get the degree of the polynomial.
    size_t GetDegree() const { return m_coeffs.size() - 1; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPoly, 0)

}  // end namespace chrono

#endif
