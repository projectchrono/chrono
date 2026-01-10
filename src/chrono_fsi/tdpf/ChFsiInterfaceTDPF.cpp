// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Custom FSI interface for coupling a TDPF-based fluid system with a Chrono MBS
//
// =============================================================================

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/tdpf/ChFsiInterfaceTDPF.h"

namespace chrono {
namespace fsi {
namespace tdpf {

ChFsiInterfaceTDPF::ChFsiInterfaceTDPF(ChSystem* sysMBS, ChFsiFluidSystemTDPF* sysTDPF)
    : ChFsiInterface(sysMBS, sysTDPF), m_sysTDPF(sysTDPF) {}

ChFsiInterfaceTDPF::~ChFsiInterfaceTDPF() {}

void ChFsiInterfaceTDPF::ExchangeSolidStates() {
    size_t num_bodies = m_fsi_bodies.size();
    for (size_t i = 0; i < num_bodies; i++) {
        const auto& body = *m_fsi_bodies[i]->body.get();
        m_sysTDPF->m_hc_state.bodies[i].position = body.GetPos().eigen();
        m_sysTDPF->m_hc_state.bodies[i].orientation_rpy = body.GetRot().GetCardanAnglesXYZ().eigen();
        m_sysTDPF->m_hc_state.bodies[i].linear_velocity = body.GetPosDt().eigen();
        m_sysTDPF->m_hc_state.bodies[i].angular_velocity = body.GetAngVelParent().eigen();
    }
}

void ChFsiInterfaceTDPF::ExchangeSolidForces() {
    size_t num_bodies = m_fsi_bodies.size();
    for (size_t i = 0; i < num_bodies; i++) {
        // Get body wrench from TDPF system
        auto force = m_sysTDPF->m_hc_forces[i].segment(0, 3);
        auto torque = m_sysTDPF->m_hc_forces[i].segment(3, 3);

        // Apply to Chrono body
        auto& body = *m_fsi_bodies[i]->body.get();
        body.EmptyAccumulator(m_fsi_bodies[i]->fsi_accumulator);
        body.AccumulateForce(m_fsi_bodies[i]->fsi_accumulator, force, body.GetPos(), false);
        body.AccumulateTorque(m_fsi_bodies[i]->fsi_accumulator, torque, false);

        // Cache forces on FSI bodies
        m_fsi_bodies[i]->fsi_force = force;
        m_fsi_bodies[i]->fsi_torque = torque;
    }
}

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
