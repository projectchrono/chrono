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

#include "HMMWV9_PacTire.h"
#include "utils/ChUtilsData.h"

namespace hmmwv {

using namespace chrono;
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string HMMWV9_PacTire::m_defaultOutFilename = "PacTire";
const std::string HMMWV9_PacTire::m_defaultPacTireParamFile = utils::GetModelDataFile("hmmwv/pactest.tir");

HMMWV9_PacTire::HMMWV9_PacTire(ChTerrain& terrain,	const std::string& pacTire_paramFile):
ChPacejkaTire(terrain)
{

	if(pacTire_paramFile == "none")
		m_paramFile = m_defaultPacTireParamFile;
	else
		m_paramFile = pacTire_paramFile;

}



std::string HMMWV9_PacTire::getPacTireParamFile()
{
	return m_paramFile;
}

}  // end namespace hmmwv
