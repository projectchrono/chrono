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

ChPacejkaTire::ChPacejkaTire(const ChTerrain& terrain, 
														 const std::string& pacTire_paramFile)
: ChTire(terrain),
  m_paramFile(pacTire_paramFile),
  m_params_defined(false)
{
  // use a test file for now
  this->Initialize();
}

ChPacejkaTire::ChPacejkaTire(const ChPacejkaTire& tire, 
														 const chrono::ChWheelId which)
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


  // this bool will allow you to query the pac tire for output
  // Forces, moments based on wheel state info.
  m_params_defined = true;
}

void ChPacejkaTire::readPacTireInput(std::ifstream& inFile)
{
  // advance to the first part of the file with data we need to read
  std::string tline;

  while (std::getline(inFile, tline))
  {
    // first main break
    if (tline[0] == '$')
      break;
  }

	// to keep things sane (and somewhat orderly), create a function to read 
	// each subsection. there may be overlap between different PacTire versions,
	// where these section read functions can be reused

  // 0:  [UNITS], all token values are strings
	readSection_UNITS(inFile);

	// 1: [MODEL]
	readSection_MODEL(inFile);

	// 2: [DIMENSION]
	readSection_DIMENSION(inFile);

	// 3: [SHAPE]
	readSection_SHAPE(inFile);

	// 4: [VERTICAL]
	readSection_VERTICAL(inFile);

	// 5-8, ranges for: LONG_SLIP, SLIP_ANGLE, INCLINATION_ANGLE, VETRICAL_FORCE,
	// in that order
	readSection_RANGES(inFile);

	// 9: [SCALING_COEFFICIENTS]
	readSection_SCALING_COEFFICIENTS(inFile);

	// 10: [LONGITUDINAL_COEFFICIENTS]
	readSection_LONGITUDINAL_COEFFICIENTS(inFile);

	// 11: [OVERTURNING_COEFFICIENTS]
	readSection_OVERTURNING_COEFFICIENTS(inFile);

	// 12: [LATERAL_COEFFICIENTS]
	readSection_LATERAL_COEFFICIENTS(inFile);

	// 13: [ROLLING_COEFFICIENTS]
	readSection_ROLLING_COEFFICIENTS(inFile);

	// 14: [ALIGNING_COEFFICIENTS]
	readSection_ALIGNING_COEFFICIENTS(inFile);

}


void ChPacejkaTire::readSection_UNITS(std::ifstream& inFile){
	// skip the first line
	std::string tline;
  std::getline(inFile, tline);

  // string util stuff
  std::string tok;    // name of token
  std::string val_str; // temp for string token values
  std::vector<std::string> split;
  double val_d;         // temp for double token values
  int val_i;          // temp for int token values
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    // get the token / value
    // split = utils::splitStr(m_line,'=');
    // tok = utils::splitStr(split[0],' ')[0];
    // val_str = utils::splitStr(split[1],'\'')[1];

    // all pactire Parameters better be in MKS
    // optionally, set up unit conversion constants here

  }


}

void ChPacejkaTire::readSection_MODEL(std::ifstream& inFile){
	// skip the first line
	std::string tline;
  std::getline(inFile, tline);

  // string util stuff
  std::vector<std::string> split;
	std::string val_str; // string token values

	// token value changes type in this section, do it manually
	std::getline(inFile, tline);
 
  // get the token / value
  split = utils::splitStr(tline,'=');
	m_params.model.property_file_format =  utils::splitStr(split[1],'\'')[1];

	std::getline(inFile, tline);
	m_params.model.use_mode = utils::fromTline<int>(tline);

	std::getline(inFile, tline);
	m_params.model.vxlow =  utils::fromTline<double>(tline);

	std::getline(inFile, tline);
	m_params.model.longvl = utils::fromTline<double>(tline);

	std::getline(inFile, tline);
	split = utils::splitStr(tline,'=');
	m_params.model.tyreside = utils::splitStr(split[1],'\'')[1];

}

void ChPacejkaTire::readSection_DIMENSION(std::ifstream& inFile){
	// skip the first line
	std::string tline;
  std::getline(inFile, tline);
	// if all the data types are the same in a subsection, life is a little easier
	// push each token value to this vector, check the # of items added, then 
	// create the struct by hand only
	std::vector<double> dat;

	while (std::getline(inFile, tline)) {
  // made it to the next section
		if (tline[0] == '$')
			break;
		dat.push_back(utils::fromTline<double>(tline) );
	}

	if( dat.size() != 5) {
		GetLog() << " error reading DIMENSION section of pactire input file!!! \n\n";
		return;
	}

}


void ChPacejkaTire::readSection_SHAPE(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_VERTICAL(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_RANGES(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_SCALING_COEFFICIENTS(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_LONGITUDINAL_COEFFICIENTS(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_OVERTURNING_COEFFICIENTS(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_LATERAL_COEFFICIENTS(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_ROLLING_COEFFICIENTS(std::ifstream& inFile){


}


void ChPacejkaTire::readSection_ALIGNING_COEFFICIENTS(std::ifstream& inFile){


}



void ChPacejkaTire::WriteOutData(ChSharedBodyPtr    spindle,
                                 const double       time,
                                 const std::string& outFileName)
{

}


}  // end namespace chrono
