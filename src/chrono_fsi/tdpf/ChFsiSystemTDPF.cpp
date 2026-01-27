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
// Implementation of FSI system using a TDPF fluid solver.
//
// =============================================================================

#include <iostream>
#include <algorithm>

#include "chrono/utils/ChUtils.h"

#include "chrono/physics/ChLoadHydrodynamics.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiInterfaceTDPF.h"

#include "chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.h"

namespace chrono {
namespace fsi {
namespace tdpf {

ChFsiSystemTDPF::ChFsiSystemTDPF(ChSystem* sysMBS, ChFsiFluidSystemTDPF* sysTDPF, bool use_generic_interface)
    : ChFsiSystem(sysMBS, sysTDPF), m_sysTDPF(sysTDPF), m_generic_fsi_interface(use_generic_interface) {
    if (use_generic_interface) {
        std::cout << "Create an FSI system using a generic FSI interface" << std::endl;
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceGeneric>(sysMBS, sysTDPF);
    } else {
        std::cout << "Create an FSI system using a custom TDPF FSI interface" << std::endl;
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceTDPF>(sysMBS, sysTDPF);
    }
}

ChFsiSystemTDPF::~ChFsiSystemTDPF() {}

ChFsiFluidSystemTDPF& ChFsiSystemTDPF::GetFluidSystemTDPF() const {
    ChAssertAlways(m_sysTDPF);
    return *m_sysTDPF;
}

void ChFsiSystemTDPF::SetHydroFilename(const std::string& filename) {
    m_sysTDPF->SetHydroFilename(filename);
}

void ChFsiSystemTDPF::Initialize() {
    // Initialize the TDPF solver and the FSI interface
    ChFsiSystem::Initialize();

    // Handle added mass info (applied via a Chrono ChLoadHydrodynamics)
    auto num_bodies = m_fsi_interface->GetNumBodies();
    if (num_bodies > 0) {
        auto& fsi_bodies = m_fsi_interface->GetBodies();
        const auto& body_info = m_sysTDPF->m_impl->m_hydro_data.GetBodyInfos();

        ChBodyAddedMassBlocks body_blocks;
        for (size_t i = 0; i < num_bodies; i++) {
            body_blocks.push_back({fsi_bodies[i]->body, body_info[i].inf_added_mass});
        }

        auto hydro_load = chrono_types::make_shared<ChLoadHydrodynamics>(body_blocks);
        hydro_load->SetVerbose(m_verbose);
        fsi_bodies[0]->body->GetSystem()->Add(hydro_load);
    }
}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
