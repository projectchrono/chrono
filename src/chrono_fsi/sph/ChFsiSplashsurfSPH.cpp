// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/ChFsiSplashsurfSPH.h"
#include "chrono_fsi/sph/utils/UtilsPrintSph.cuh"

namespace chrono {
namespace fsi {
namespace sph {

ChFsiSplashsurfSPH::ChFsiSplashsurfSPH(const ChFsiFluidSystemSPH& sysSPH)
    : m_sysSPH(sysSPH), m_radius(-1), m_cube_size(0.5), m_smoothing_length(1.5), m_surface_threshold(0.6) {}

void ChFsiSplashsurfSPH::WriteParticleFileJSON(const std::string& filename) {
    writeParticleFileJSON(filename, *m_sysSPH.m_data_mgr);
}

void ChFsiSplashsurfSPH::WriteReconstructedSurface(const std::string& in_filename,
                                                   const std::string& out_filename,
                                                   bool quiet) {
#ifndef CHRONO_HAS_SPLASHSURF
    std::cout << "Error: splashsurf not available." << std::endl;
    throw std::runtime_error("splashsurf not available");
#else
    ChAssertAlways(m_radius > 0);

    // Create Splashsurf arguments
    std::string arguments;
    arguments += in_filename + " ";
    arguments += " -r " + std::to_string(m_radius);
    arguments += " -l " + std::to_string(m_smoothing_length);
    arguments += " -c " + std::to_string(m_cube_size);
    arguments += " -t " + std::to_string(m_surface_threshold);
    if (quiet)
        arguments += " -q ";
    arguments += " --subdomain-grid=on ";
    arguments += " " + m_arguments + " ";
    arguments += " -o " + out_filename;

    // Execute system command
    std::string syscmd;
    #ifdef _WIN32
    // /b avoids showing the black cmd window
    syscmd = "start /b " + std::string(SPLASHSURF_EXECUTABLE) + " reconstruct " + arguments;
    /*int err =*/system(syscmd.c_str());
    #else
    // Unix like systems:
    syscmd = std::string(SPLASHSURF_EXECUTABLE) + " reconstruct " + arguments;
    syscmd += " &";  // to launch and forget
    /*int err =*/system(syscmd.c_str());
    #endif
#endif
}

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
