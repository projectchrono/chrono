// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================

#include "utils/ChUtilsData.h"

#include "models/hmmwv/tire/HMMWV_PacejkaTire.h"

namespace hmmwv {

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string HMMWV_PacejkaTire::m_defaultOutFilename = "PacTire";
const std::string HMMWV_PacejkaTire::m_defaultPacTireParamFile = utils::GetModelDataFile("hmmwv/pactest.tir");


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_PacejkaTire::HMMWV_PacejkaTire(ChTerrain& terrain)
: ChPacejkaTire(terrain, m_defaultPacTireParamFile)
{
}

HMMWV_PacejkaTire::HMMWV_PacejkaTire(ChTerrain&         terrain,
                                     const std::string& pacTire_paramFile)
: ChPacejkaTire(terrain, pacTire_paramFile)
{
}

// copy constructor, only right/left tyreside may be different
HMMWV_PacejkaTire::HMMWV_PacejkaTire(const HMMWV_PacejkaTire& tire,
                                     ChWheelId                which)
: ChPacejkaTire(tire, which)
{
}


}  // end namespace hmmwv
