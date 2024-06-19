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
// Authors: Dario Mangoni
// =============================================================================

#ifndef CHFUNCT_LAMBDA_H
#define CHFUNCT_LAMBDA_H

#include <functional>
#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Lambda function wrapper
/// Allows the usage of C++ lambda functions as ChFunction objects.
class ChFunctionLambda : public ChFunction {
  public:
    ChFunctionLambda() {}
    ~ChFunctionLambda() {}

    virtual ChFunctionLambda* Clone() const override { return new ChFunctionLambda(*this); }

    void SetFunction(std::function<double(double)> function) { m_function = function; }

    std::function<double(double)> GetFunction() const { return m_function; }

    virtual double GetVal(double x) const override { return m_function(x); }

    virtual Type GetType() const override { return ChFunction::Type::LAMBDA; }

  protected:
    std::function<double(double)> m_function;
};

/// @} chrono_functions

}  // end namespace chrono

#endif  //! CHFUNCT_LAMBDA_H
