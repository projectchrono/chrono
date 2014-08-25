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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// and/or contact geometry (which are associated with the suspension's spindle
// body).
// =============================================================================

#ifndef HMMWV9_PACTIRE_H
#define HMMWV9_PACTIRE_H

#include "subsys/tire/ChPacTire.h"

namespace pactest {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//			calling update will calculate the current F_x, F_y and M_z
class HMMWV9_PacTire : public chrono::ChPacTire {
public:
	HMMWV9_PacTire(chrono::ChTerrain& terrain,
		const std::string& m_pacTire_paramFile = "none");

protected:
  
  // pactire input parameter file
  std::string m_paramFile;

  // some default values
  static const std::string outFileNameDefault;
  static const std::string default_PacFile;

  // define the pactire parameter file to use
  virtual const std::string& get_pacTire_paramFile();
};


} // end namespace pactest


#endif	// end #ifdef HMMWV9_PACTIRE_H
