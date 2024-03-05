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

#ifndef CHFUNCT_MIRROR_H
#define CHFUNCT_MIRROR_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Mirror function.
///
/// `y(x - x_axis) = y(x + x_axis)`
/// Mirrors a function about a vertical axis.
class ChApi ChFunctionMirror : public ChFunction {
  private:
    std::shared_ptr<ChFunction> m_operand_fun;
    double m_mirror_axis;  ///< symmetry axis position on x

  public:
    ChFunctionMirror();
    ChFunctionMirror(const ChFunctionMirror& other);
    ~ChFunctionMirror() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionMirror* Clone() const override { return new ChFunctionMirror(*this); }

    virtual Type GetType() const override { return ChFunction::Type::MIRROR; }

    virtual double GetVal(double x) const override;

    /// Set the symmetry axis position on x.
    void SetMirrorAxis(double axis) { m_mirror_axis = axis; }

    /// Get the symmetry axis position on x.
    double GetMirrorAxis() { return m_mirror_axis; }

    void SetOperandFunction(std::shared_ptr<ChFunction> operand_fun) { m_operand_fun = operand_fun; }
    std::shared_ptr<ChFunction> GetOperandFunction() { return m_operand_fun; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionMirror, 0)

}  // end namespace chrono

#endif
