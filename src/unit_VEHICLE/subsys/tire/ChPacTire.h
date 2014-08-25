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
// =============================================================================

#ifndef CHPACTIRE_H
#define CHPACTIRE_H

#include "physics/ChBody.h"
#include <string>

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//			calling update will calculate the current F_x, F_y and M_z
class CH_SUBSYS_API ChPacTire : public ChTire {
public:

  ChPacTire(const ChTerrain& terrain);
  virtual ~ChPacTire() {}
  
  // @brief return the most recently computed forces
  // TODO: does Pac model return forces/moments in the wheel local ref frame, or w.r.t. global coords?
  virtual ChTireForce GetTireForce() const;

  // @brief set the PacTire w/ new spindle state data.
  virtual void Update(double              time,
                      const ChBodyState&  wheel_state);

  // @brief	have the tire do a timestep, prepare 

   // @brief write output data to a file
  void WriteOutData(ChSharedBodyPtr spindle, const double time, 
	  const std::string outFileName);

protected:
  
  // @brief where to find the input parameter file
  virtual const std::string& get_pacTire_paramFile() = 0;

  // @brief specify the file name to read the Pactire input from
  void Initialize();

  // @brief look for this data file
  virtual void load_pacTire_paramFile(void);

  // @brief once Pac tire input text file has been succesfully opened, read 
  //		the input data as strings. Each vector contains a list of input data
  //		corresponding to the sections of the input file.
  //		Section descriptors are in m_inFile_sections
  virtual void read_pactire_file(std::ifstream& m_inFile,
	 std::vector<std::list<std::string> >& m_inFile_data,
	 std::vector<std::string>& m_inFile_sections);


// ----- Data members

  // based on pure slip
  ChTireForce FM_pure;

  // based on combined slip
  ChTireForce FM_combined;

  // write output data to this file
  std::string m_outFile;

  // have the tire model parameters been defined/read from file yet?
  // must call load_pacFile_tire
  bool params_defined;
};


} // end namespace chrono


#endif	// end #ifdef PACTIRE_H
