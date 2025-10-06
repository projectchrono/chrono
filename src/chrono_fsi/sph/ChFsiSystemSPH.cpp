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

////#define DEBUG_LOG

#include <iostream>
#include <algorithm>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiInterfaceSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

ChFsiSystemSPH::ChFsiSystemSPH(ChSystem* sysMBS, ChFsiFluidSystemSPH* sysSPH, bool use_generic_interface)
    : ChFsiSystem(sysMBS, sysSPH), m_sysSPH(sysSPH), m_generic_fsi_interface(use_generic_interface) {
    if (use_generic_interface) {
        std::cout << "Create an FSI system using a generic FSI interface" << std::endl;
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceGeneric>(sysMBS, sysSPH);
    } else {
        std::cout << "Create an FSI system using a custom SPH FSI interface" << std::endl;
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceSPH>(sysMBS, sysSPH);
    }
}

ChFsiSystemSPH::~ChFsiSystemSPH() {}

ChFsiFluidSystemSPH& ChFsiSystemSPH::GetFluidSystemSPH() const {
    ChAssertAlways(m_sysSPH);
    return *m_sysSPH;
}

std::shared_ptr<FsiBody> ChFsiSystemSPH::AddFsiBody(std::shared_ptr<ChBody> body,
                                                    const std::vector<ChVector3d>& bce,
                                                    const ChFrame<>& rel_frame,
                                                    bool check_embedded) {
    ChAssertAlways(m_sysSPH);

    // Add the FSI body with no geometry
    auto fsi_body = ChFsiSystem::AddFsiBody(body, nullptr, check_embedded);

    // Explicitly set the BCE marker locations
    auto& fsisph_body = m_sysSPH->m_bodies.back();

    fsisph_body.bce_ids.resize(bce.size(), (int)fsisph_body.fsi_body->index);

    ChFramed abs_frame = body->GetFrameRefToAbs() * rel_frame;
    std::transform(bce.begin(), bce.end(), std::back_inserter(fsisph_body.bce_coords),
                   [&rel_frame](const ChVector3d& v) { return rel_frame.TransformPointLocalToParent(v); });
    std::transform(bce.begin(), bce.end(), std::back_inserter(fsisph_body.bce),
                   [&abs_frame](const ChVector3d& v) { return abs_frame.TransformPointLocalToParent(v); });

    return fsi_body;
}

void ChFsiSystemSPH::AddFsiBoundary(const std::vector<ChVector3d>& bce, const ChFrame<>& frame) {
    ChAssertAlways(m_sysSPH);
    m_sysSPH->AddBCEBoundary(bce, frame);
}

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
