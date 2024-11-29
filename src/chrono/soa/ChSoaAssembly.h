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
// =============================================================================

#ifndef CH_SOA_ASSEMBLY_H
#define CH_SOA_ASSEMBLY_H

#include "chrono/core/ChApiCE.h"
#include "chrono/soa/ChMobilizedBody.h"

namespace chrono {
namespace soa {

/// @addtogroup chrono_soa
/// @{

/// Definition of a Chrono subcomponent representing an assembly modeled using SOA relative coordinate formulation.
class ChApi ChSoaAssembly {
  public:
    void AddBody(std::shared_ptr<ChMobilizedBody> body) {
        body->m_assembly = this;
        m_bodies.push_back(body);
    }

    void RemoveBody(std::shared_ptr<ChMobilizedBody> body) {
        auto itr = std::find(std::begin(m_bodies), std::end(m_bodies), body);
        assert(itr != m_bodies.end());
        m_bodies.erase(itr);
    }

    //// TEMPORARY
    const ChVectorDynamic<>& getCurState() const { return m_states; }
    double getCurState(int which) const { return m_states(which); }
    void setCurState(int which, double val) { m_states(which) = val; }

    const ChVectorDynamic<>& getCurStateDeriv() const { return m_state_derivs; }
    double getCurStateDeriv(int which) const { return m_state_derivs(which); }
    void setCurStateDeriv(int which, double val) { m_state_derivs(which) = val; }

  private:
    std::vector<std::shared_ptr<ChMobilizedBody>> m_bodies;

    //// TEMPORARY
    ChVectorDynamic<> m_states;
    ChVectorDynamic<> m_state_derivs;
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
