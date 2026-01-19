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

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace tdpf {

ChFsiInterfaceTDPF::ChFsiInterfaceTDPF(ChSystem* sysMBS, ChFsiFluidSystemTDPF* sysTDPF)
    : ChFsiInterface(sysMBS, sysTDPF), m_sysTDPF(sysTDPF) {}

ChFsiInterfaceTDPF::~ChFsiInterfaceTDPF() {}

void ChFsiInterfaceTDPF::ExchangeSolidStates() {
    if (m_verbose)
        cout << "ChFsiInterfaceTDPF::ExchangeSolidStates" << endl;

    size_t num_bodies = m_fsi_bodies.size();
    for (size_t i = 0; i < num_bodies; i++) {
        const auto& body = *m_fsi_bodies[i]->body;
        const auto& p = body.GetPos();
        const auto& r = body.GetRot().GetCardanAnglesXYZ();
        const auto& pd = body.GetPosDt();
        const auto& rd = body.GetAngVelParent();
        m_sysTDPF->m_hc_state.bodies[i].position = p.eigen();
        m_sysTDPF->m_hc_state.bodies[i].orientation_rpy = r.eigen();
        m_sysTDPF->m_hc_state.bodies[i].linear_velocity = pd.eigen();
        m_sysTDPF->m_hc_state.bodies[i].angular_velocity = rd.eigen();
        if (m_verbose) {
            cout << i << " " << body.GetName() << endl;
            cout << "  pos:   " << p << endl;
            cout << "  rot:   " << r << endl;
            cout << "  l_vel: " << pd << endl;
            cout << "  a_vel: " << rd << endl;
        }
    }
}

void ChFsiInterfaceTDPF::ExchangeSolidForces() {
    if (m_verbose)
        cout << "ChFsiInterfaceTDPF::ExchangeSolidForces" << endl;

    size_t num_bodies = m_fsi_bodies.size();
    for (size_t i = 0; i < num_bodies; i++) {
        // Get body wrench from TDPF system
        auto force = ChVector3d(m_sysTDPF->m_hc_forces[i].segment(0, 3));
        auto torque = ChVector3d(m_sysTDPF->m_hc_forces[i].segment(3, 3));

        // Apply to Chrono body
        auto& body = *m_fsi_bodies[i]->body;
        body.EmptyAccumulator(m_fsi_bodies[i]->fsi_accumulator);
        body.AccumulateForce(m_fsi_bodies[i]->fsi_accumulator, force, body.GetPos(), false);
        body.AccumulateTorque(m_fsi_bodies[i]->fsi_accumulator, torque, false);

        // Cache forces on FSI bodies
        m_fsi_bodies[i]->fsi_force = force;
        m_fsi_bodies[i]->fsi_torque = torque;

        if (m_verbose) {
            cout << i << " " << body.GetName() << endl;
            cout << "  F: " << force << endl;
            cout << "  T: " << torque << endl;
        }
    }
}

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
