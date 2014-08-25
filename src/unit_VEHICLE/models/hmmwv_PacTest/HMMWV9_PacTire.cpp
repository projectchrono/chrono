#include "HMMWV9_PacTire.h"
#include "utils/ChUtilsData.h"

namespace pactest{

using namespace chrono;
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string HMMWV9_PacTire::outFileNameDefault = "PacTire";
const std::string HMMWV9_PacTire::default_PacFile = utils::GetModelDataFile("hmmwv/pactest.tir");

HMMWV9_PacTire::HMMWV9_PacTire(ChTerrain& terrain,	const std::string& pacTire_paramFile)
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