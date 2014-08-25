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

namespace pactest{

using namespace chrono;
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string HMMWV9_PacTire::outFileNameDefault = "PacTire";
const std::string HMMWV9_PacTire::default_PacFile = utils::GetModelDataFile("hmmwv/pactest.tir");

HMMWV9_PacTire::HMMWV9_PacTire(ChTerrain& terrain,	const std::string& pacTire_paramFile):
ChPacTire(terrain)
{

	if(pacTire_paramFile == "none")
		this->m_paramFile = default_PacFile;
	else
		this->m_paramFile = pacTire_paramFile;

}



const std::string& HMMWV9_PacTire::get_pacTire_paramFile()
{
	return this->m_paramFile;

}

}		// end namespace pactest