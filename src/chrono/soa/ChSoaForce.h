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
// Definition of the ChForce base class and derived classes. 
// - A ChForce acts on one or more mobilized bodies.
// - Forces are managed by the SOA assembly.
//
// =============================================================================

#ifndef CH_SOA_FORCE_H
#define CH_SOA_FORCE_H

#include "chrono/core/ChApiCE.h"
#include "chrono/soa/ChSoaMobilizedBody.h"

namespace chrono {
namespace soa {

class ChSoaAssembly;

// =============================================================================

/// Base class for a force on a single mobilized body or between a pair of two mobilized bodies.
class ChApi ChSoaForce {
  public:
    virtual ~ChSoaForce() {}

    bool isEnabled() const { return m_enabled; }

    void enable(bool val) { m_enabled = val; }

    virtual void apply() = 0;

  protected:
    ChSoaForce() : m_assembly(nullptr), m_enabled(true) {}

    ChSoaAssembly* m_assembly;

  private:
    friend class ChSoaAssembly;
    bool m_enabled;
};

// =============================================================================

/// Base class for body forces.
/// These are forces acting on a single body.
class ChApi ChSoaBodyForce : public ChSoaForce {
  public:
    virtual ~ChSoaBodyForce() {}

  protected:
    ChSoaBodyForce(std::shared_ptr<ChSoaMobilizedBody> body) : m_body(body) {}
    std::shared_ptr<ChSoaMobilizedBody> m_body;
};

class ChSoaConstantForce : public ChSoaBodyForce {
  public:
    ChSoaConstantForce(std::shared_ptr<ChSoaMobilizedBody> body, const ChVector3d& location, const ChVector3d& force);
    virtual void apply() override;

  private:
    ChVector3d m_location;
    ChVector3d m_force;
};

class ChSoaConstantTorque : public ChSoaBodyForce {
  public:
    ChSoaConstantTorque(std::shared_ptr<ChSoaMobilizedBody> body, const ChVector3d& torque);
    virtual void apply() override;

  private:
    ChVector3d m_torque;
};

// =============================================================================

/// Linear spring-damper force between two mobilized bodies.
class ChSoaSpringDamperForce : public ChSoaForce {
  public:
    ChSoaSpringDamperForce(std::shared_ptr<ChSoaMobilizedBody> body1,
                           std::shared_ptr<ChSoaMobilizedBody> body2,
                           const ChVector3d& loc1,
                           const ChVector3d& loc2,
                           double l0,
                           double k,
                           double c);
    virtual void apply() override;

  private:
    std::shared_ptr<ChSoaMobilizedBody> m_body1;
    std::shared_ptr<ChSoaMobilizedBody> m_body2;
    ChVector3d m_loc1;
    ChVector3d m_loc2;
    double m_l0;
    double m_k;
    double m_c;
    double m_l;
};

}  // namespace soa
}  // namespace chrono

#endif
