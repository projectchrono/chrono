// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// This file contains the definition of the ChSoaMobilityForce base class and
// derived classes. An ChSoaMobilityForce is a force element which applies a hinge
// force on a single degree of freedom (DOF) of a mobilized body.
//
// =============================================================================

#ifndef CH_SOA_MOBILITY_FORCE_H
#define CH_SOA_MOBILITY_FORCE_H

#include <string>

#include "chrono/core/ChApiCE.h"

namespace chrono {
namespace soa {

class ChSoaMobilizedBody;

/// @addtogroup chrono_soa
/// @{

/// Base class for mobility forces.
/// These are scalar forces acting on a degree of freedom of a mobilized body.
class ChApi ChSoaMobilityForce {
  public:
    virtual ~ChSoaMobilityForce() {}

    bool isEnabled() const { return m_enabled; }

    void enable(bool val) { m_enabled = val; }

    virtual double evaluate(double q, double u) = 0;

  protected:
    ChSoaMobilityForce() : m_enabled(true) {}

    bool m_enabled;
};

// -----------------------------------------------------------------------------

/// Linear spring mobility force acting on a degree of freedom.
/// The spring constant and free state must be conmensurate with the type of DOF (translational or rotational).
class ChApi ChMobilitySpringForce : public ChSoaMobilityForce {
  public:
    ChMobilitySpringForce(double k, double x0) : m_k(k), m_x0(x0) {}

    virtual double evaluate(double q, double u) override { return -m_k * (q - m_x0); }

    double getSpringConst() const { return m_k; }
    double getSpringFreeVal() const { return m_x0; }
    void setSpringConst(double k) { m_k = k; }
    void setSpringFreeVal(double x0) { m_x0 = x0; }

  private:
    double m_k;
    double m_x0;
};

// -----------------------------------------------------------------------------

/// Linear damper mobility force acting on a degree of freedom.
/// The spring constant and free state must be conmensurate with the type of DOF (translational or rotational).
class ChApi ChMobilityDamperForce : public ChSoaMobilityForce {
  public:
    ChMobilityDamperForce(double c) : m_c(c) {}

    virtual double evaluate(double q, double u) override { return -m_c * u; }

    double getDampingCoef() const { return m_c; }
    void setDampingCoef(double c) { m_c = c; }

  private:
    double m_c;
};

// -----------------------------------------------------------------------------

/// Linear spring-damper mobility force acting on a degree of freedom.
/// The spring constant and free state must be conmensurate with the type of DOF (translational or rotational).
class ChApi ChMobilitySpringDamperForce : public ChSoaMobilityForce {
  public:
    ChMobilitySpringDamperForce(double k, double x0, double c) : m_k(k), m_x0(x0), m_c(c) {}

    virtual double evaluate(double q, double u) override { return -m_k * (q - m_x0) - m_c * u; }

    double getSpringConst() const { return m_k; }
    double getSpringFreeVal() const { return m_x0; }
    double getDampingCoef() const { return m_c; }
    void setSpringConst(double k) { m_k = k; }
    void setSpringFreeVal(double x0) { m_x0 = x0; }
    void setDampingCoef(double c) { m_c = c; }

  private:
    double m_k;
    double m_x0;
    double m_c;
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
