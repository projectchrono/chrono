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

#include "HMMWV_PacejkaTire.h"
#include "utils/ChUtilsData.h"

namespace hmmwv {

using namespace chrono;
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string HMMWV_PacejkaTire::m_defaultOutFilename = "PacTire";
const std::string HMMWV_PacejkaTire::m_defaultPacTireParamFile = utils::GetModelDataFile("hmmwv/pactest.tir");


HMMWV_PacejkaTire::HMMWV_PacejkaTire(ChTerrain& terrain, const ChBodyState& ICs):
    ChPacejkaTire(m_defaultPacTireParamFile,terrain,ICs,false)
{


}

HMMWV_PacejkaTire::HMMWV_PacejkaTire(const std::string& pacTire_paramFile, ChTerrain& terrain,
															 const ChBodyState& ICs, 
															 const double step_size, 
															 const bool use_transient_slip,
															 const double Fz_override):
    ChPacejkaTire(pacTire_paramFile, terrain, ICs, step_size, use_transient_slip, Fz_override)
{


}

// copy constructor, only right/left tyreside may be different
HMMWV_PacejkaTire::HMMWV_PacejkaTire(const HMMWV_PacejkaTire& tire, chrono::ChWheelId which):
	ChPacejkaTire(tire, which)
{

}


}  // end namespace hmmwv
