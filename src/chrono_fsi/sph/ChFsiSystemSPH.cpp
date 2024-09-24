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
// Author: Radu Serban
// =============================================================================
//
// Implementation of FSI system using an SPH fluid solver.
//
// =============================================================================

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

namespace chrono {
namespace fsi {

using namespace sph;

ChFsiSystemSPH::ChFsiSystemSPH(ChSystem* sysMBS)
    : ChFsiSystem(sysMBS) {
    auto sysSPH = chrono_types::make_shared<ChFluidSystemSPH>();
    auto interfaceSPH = chrono_types::make_shared<ChFsiInterfaceSPH>(*sysSPH->m_data_mgr, m_verbose);
    sysSPH->SetFsiInterface(interfaceSPH);

    m_sysCFD = sysSPH;
    m_fsi_interface = interfaceSPH;
}

ChFsiSystemSPH::~ChFsiSystemSPH() {}

ChFluidSystemSPH& ChFsiSystemSPH::GetFluidSystemSPH() const {
    return *std::static_pointer_cast<ChFluidSystemSPH>(m_sysCFD);
}

ChFsiInterfaceSPH& ChFsiSystemSPH::GetFsiInterface() const {
    return *std::static_pointer_cast<ChFsiInterfaceSPH>(m_fsi_interface);
}

}  // end namespace fsi
}  // end namespace chrono
