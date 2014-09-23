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
//
// Concrete class for using a pac tire model
// =============================================================================

#ifndef HMMWV_PACEJKATIRE_H
#define HMMWV_PACEJKATIRE_H

#include "subsys/tire/ChPacejkaTire.h"
#include "subsys/ChVehicle.h"
#include "subsys/terrain/FlatTerrain.h"

namespace hmmwv {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//			calling update will calculate the current F_x, F_y and M_z
class HMMWV_PacejkaTire : public chrono::ChPacejkaTire {
public:

	 // @brief use the default input pacTire param file, don't use transient slip
  HMMWV_PacejkaTire(chrono::ChTerrain& terrain, const chrono::ChBodyState& ICs);

	// @brief assume flat terrain at height = 0
	HMMWV_PacejkaTire(const std::string& pacTire_paramFile, chrono::ChTerrain& terrain,
		const chrono::ChBodyState& ICs, const double step_size = 0.01,
		const bool use_transient_slip = true,
		const double Fz_override = -1);

	// @brief if the tire paramter file is the same, only the tyreside will be different
	HMMWV_PacejkaTire(const HMMWV_PacejkaTire& tire, chrono::ChWheelId which);


protected:

  // some default values
  static const std::string m_defaultOutFilename;
  static const std::string m_defaultPacTireParamFile;

};


} // end namespace hmmwv


#endif
