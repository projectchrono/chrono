// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// File description
//
// =============================================================================

#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#include <thread>
#include <chrono>

namespace chrono {
namespace synchrono {

SynDDSListener::SynDDSListener() : m_matched(0) {}

void SynDDSListener::WaitForMatches(unsigned int num_matches, unsigned int rate) {
    while (num_matches > m_matched)
        std::this_thread::sleep_for(std::chrono::milliseconds(rate));
}

}  // namespace synchrono
}  // namespace chrono