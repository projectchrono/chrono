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
// Base class for a Pacjeka type Magic formula 2002 tire model
//
// =============================================================================

#ifndef CH_PACEJKATIRE_H
#define CH_PACEJKATIRE_H

#include <string>
#include <fstream>

#include "physics/ChBody.h"

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"
// has the data structs for the pacTire parameters
#include "ChPac2002_data.h"

namespace chrono {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//      calling update will calculate the current F_x, F_y and M_z
class CH_SUBSYS_API ChPacejkaTire : public ChTire {
public:

  ChPacejkaTire(const ChTerrain& terrain, const std::string& pacTire_paramFile);

	// @brief copy constructor, only tyreside will be different
	ChPacejkaTire(const ChPacejkaTire& tire, chrono::ChWheelId which);

  virtual ~ChPacejkaTire() {}
  
  // @brief return the most recently computed forces
  // TODO: does Pac model return forces/moments in the wheel local ref frame, or w.r.t. global coords?
  virtual ChTireForce GetTireForce() const;

  // @brief set the PacTire w/ new spindle state data.
  virtual void Update(double              time,
                      const ChBodyState&  wheel_state);

  // @brief have the tire do a timestep, prepare 

   // @brief write output data to a file
  void WriteOutData(ChSharedBodyPtr    spindle,
                    const double       time,
                    const std::string& outFileName);

protected:

  // @brief where to find the input parameter file
  virtual std::string getPacTireParamFile();

  // @brief specify the file name to read the Pactire input from
  void Initialize();

  // @brief look for this data file
  virtual void loadPacTireParamFile(void);

  // @brief once Pac tire input text file has been succesfully opened, read 
  //    the input data, and populate the data struct
  virtual void readPacTireInput(std::ifstream& inFile);

	// @brief functions for reading each section in the paramter file
	virtual void readSection_UNITS(std::ifstream& inFile);
	virtual void readSection_MODEL(std::ifstream& inFile);
	virtual void readSection_DIMENSION(std::ifstream& inFile);
	virtual void readSection_SHAPE(std::ifstream& inFile);
	virtual void readSection_VERTICAL(std::ifstream& inFile);
	virtual void readSection_RANGES(std::ifstream& inFile);
	virtual void readSection_SCALING_COEFFICIENTS(std::ifstream& inFile);
	virtual void readSection_LONGITUDINAL_COEFFICIENTS(std::ifstream& inFile);
	virtual void readSection_OVERTURNING_COEFFICIENTS(std::ifstream& inFile);
	virtual void readSection_LATERAL_COEFFICIENTS(std::ifstream& inFile);
	virtual void readSection_ROLLING_COEFFICIENTS(std::ifstream& inFile);
	virtual void readSection_ALIGNING_COEFFICIENTS(std::ifstream& inFile);

// ----- Data members

  // based on pure slip
  ChTireForce m_FM_pure;

  // based on combined slip
  ChTireForce m_FM_combined;

  // all pactire models require an input parameter file
  std::string m_paramFile;
  // write output data to this file
  std::string m_outFile;

  // have the tire model parameters been defined/read from file yet?
  // must call load_pacFile_tire
  bool m_params_defined;

  // model parameter factors stored here
  struct Pac2002_data m_params;
};


} // end namespace chrono


#endif
