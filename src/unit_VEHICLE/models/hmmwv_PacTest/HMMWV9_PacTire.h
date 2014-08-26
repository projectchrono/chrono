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

#ifndef HMMWV9_PACTIRE_H
#define HMMWV9_PACTIRE_H

#include "subsys/tire/ChPacejkaTire.h"

namespace hmmwv {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//			calling update will calculate the current F_x, F_y and M_z
class HMMWV9_PacTire : public chrono::ChPacejkaTire {
public:
	HMMWV9_PacTire(chrono::ChTerrain& terrain,
		const std::string& m_pacTire_paramFile = "none");

protected:
  
  // pactire input parameter file
  std::string m_paramFile;

  // some default values
  static const std::string m_defaultOutFilename;
  static const std::string m_defaultPacTireParamFile;

  // define the pactire parameter file to use
  virtual std::string getPacTireParamFile();
};


} // end namespace hmmwv


#endif
