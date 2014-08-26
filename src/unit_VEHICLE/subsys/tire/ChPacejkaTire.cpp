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

#include "ChPacejkaTire.h"

#include "subsys/ChVehicle.h"
#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

namespace chrono{

ChPacejkaTire::ChPacejkaTire(const ChTerrain& terrain)
: ChTire(terrain),
  m_params_defined(false)
{
  // use a test file for now
  this->Initialize();
}

// -------------------
// Class functions
void ChPacejkaTire::Initialize(void)
{
  this->loadPacTireParamFile();
}


void ChPacejkaTire::Update(double              time,
                           const ChBodyState&  wheel_state)
{
  if (!m_params_defined)
  {
    GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
    return;
  }
}

// can always return Force/moment, but up to the user to call Update properly
ChTireForce ChPacejkaTire::GetTireForce() const
{
  return m_FM_pure;
}

// what does the PacTire in file look like?
// see models/data/hmmwv/pactest.tir
void ChPacejkaTire::loadPacTireParamFile()
{
  // try to load the file
  std::ifstream inFile(this->getPacTireParamFile().c_str(), std::ios::in);

  // if not loaded, say something and exit
  if (!inFile.is_open())
  {
    GetLog() << "\n\n !!!!!!! couldn't load the pac tire file: " << getPacTireParamFile().c_str() << "\n\n";
    return;
  }

  // success in opening file, load the data, broken down into sections
  // according to what is found in the PacTire input file
  std::vector<std::list<std::string> > inFile_data;
  std::vector<std::string> inFile_sections;
  this->readPacTireInput(inFile, inFile_data, inFile_sections);

  // now that we have read the data, apply it


  // this bool will allow you to query the pac tire for output
  // Forces, moments based on wheel state info.
  m_params_defined = true;
}

void ChPacejkaTire::readPacTireInput(
        std::ifstream& inFile,
        std::vector<std::list<std::string> >& inFile_data,
        std::vector<std::string>& inFile_sections)
{
  // advance to the first part of the file with data we need to read
  std::string m_line;

  while (std::getline(inFile, m_line))
  {
    // first main break
    if (m_line[0] == '$')
      break;
  }

  // 0:  [UNITS]
  while (std::getline(inFile, m_line)) {
    // made it to the next section
    if (m_line[0] == '$')
      break;
  }
}

void ChPacejkaTire::WriteOutData(ChSharedBodyPtr    spindle,
                                 const double       time,
                                 const std::string& outFileName)
{

}


}  // end namespace chrono
