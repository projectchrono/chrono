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
#include "include/strUtils.h"

namespace chrono{

ChPacejkaTire::ChPacejkaTire(const ChTerrain& terrain, const std::string& pacTire_paramFile)
: ChTire(terrain),
  m_paramFile(pacTire_paramFile),
  m_params_defined(false)
{
  // use a test file for now
  this->Initialize();
}

ChPacejkaTire::ChPacejkaTire(const ChPacejkaTire& tire, const chrono::ChWheelId which)
	: ChTire(tire.m_terrain),
	m_paramFile(tire.m_paramFile),
	m_params_defined(false)
{
	if(which == FRONT_RIGHT || which == REAR_RIGHT)
		m_params.model.tyreside = "RIGHT";
	else
		m_params.model.tyreside = "LEFT";
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



std::string ChPacejkaTire::getPacTireParamFile()
{
	return m_paramFile;
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
    GetLog() << " I bet you have the pacTire param file opened in a text editor somewhere.... \n\n\n";
    return;
  }

  // success in opening file, load the data, broken down into sections
  // according to what is found in the PacTire input file
  this->readPacTireInput(inFile);

  // now that we have read the data, apply it


  // this bool will allow you to query the pac tire for output
  // Forces, moments based on wheel state info.
  m_params_defined = true;
}

void ChPacejkaTire::readPacTireInput(std::ifstream& inFile)
{
  // advance to the first part of the file with data we need to read
  std::string m_line;

  while (std::getline(inFile, m_line))
  {
    // first main break
    if (m_line[0] == '$')
      break;
  }

  // 0:  [UNITS], all token values are strings
  std::getline(inFile, m_line);

  // string util stuff
  std::string tok;    // name of token
  std::string val_str; // temp for string token values
  std::vector<std::string> split;
  double val_d;         // temp for double token values
  int val_i;          // temp for int token values
  while (std::getline(inFile, m_line)) {
    // made it to the next section
    if (m_line[0] == '$')
      break;
    // get the token / value
    // split = utils::splitStr(m_line,'=');
    // tok = utils::splitStr(split[0],' ')[0];
    // val_str = utils::splitStr(split[1],'\'')[1];

    // all pactire Parameters better be in MKS
    // optionally 

  }
}

void ChPacejkaTire::WriteOutData(ChSharedBodyPtr    spindle,
                                 const double       time,
                                 const std::string& outFileName)
{

}


}  // end namespace chrono
