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

#include <H5Cpp.h>

#include "chrono/utils/ChUtils.h"

#include "chrono/physics/ChLoadHydrodynamics.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiInterfaceTDPF.h"

#include "hydroc/io/h5_reader.h"

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

void ChFsiSystemTDPF::Initialize() {
    ChFsiSystem::Initialize();

    auto num_bodies = m_fsi_interface->GetNumBodies();
    auto& fsi_bodies = m_fsi_interface->GetBodies();

    // Handle added mass info (applied via a Chrono ChLoadHydrodynamics)
    ChBodyAddedMassBlocks body_blocks;

    // Read hydro data from input file
    if (!m_hydro_filename.empty()) {
        auto h5_file_info = H5FileInfo(m_hydro_filename, num_bodies);
        try {
            auto hydro_data = H5FileInfo(m_hydro_filename, num_bodies).ReadH5Data();
            const auto& body_info = hydro_data.GetBodyInfos();
            for (size_t i = 0; i < num_bodies; i++) {
                body_blocks.insert(std::pair(fsi_bodies[i]->body, body_info[i].inf_added_mass));
            }
        } catch (const H5::Exception& e) {
            std::ostringstream oss;
            oss << "Unable to open/read HDF5 hydro data file: " << m_hydro_filename << "\n";
            oss << "HDF5 error: " << e.getDetailMsg() << "\n";
            throw std::runtime_error(oss.str());
        }
    } else {
        throw std::runtime_error("No HD5 file provided");
    }

    // Create hydrodynamics load for added mass
    ChAssertAlways(!body_blocks.empty());
    auto hydro_load = chrono_types::make_shared<ChLoadHydrodynamics>(body_blocks);
    hydro_load->SetVerbose(false);
    fsi_bodies[0]->body->GetSystem()->Add(hydro_load);
}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
