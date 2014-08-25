#include "ChPacTire.h"

#include "subsys/ChVehicle.h"
#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

namespace chrono{

ChPacTire::ChPacTire(const ChTerrain& terrain): params_defined(false), ChTire(terrain)
{
	// use a test file for now
	this->Initialize(  );
}

// -------------------
// Class functions
void ChPacTire::Initialize(void)
{
	this->load_pacTire_paramFile( );

}


void ChPacTire::Update(double              time,
                      const ChBodyState&  wheel_state)
{
	if( !this->params_defined)
	{
		GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
		return;
	}



}

// can always return Force/moment, but up to the user to call Update properly
ChTireForce ChPacTire::GetTireForce() const
{
	return this->FM_pure;

}

// what does the PacTire in file look like?
// see models/data/hmmwv/pactest.tir
void ChPacTire::load_pacTire_paramFile(){
	// try to load the file
	std::ifstream inFile(this->get_pacTire_paramFile().c_str(), std::ios::in);
	// if not loaded, say something and exit
	if(! inFile.is_open() ) 
	{
		GetLog() << "\n\n !!!!!!! couldn't load the pac tire file: " << this->get_pacTire_paramFile().c_str() << "\n\n";
		return;
	}
	
	// success in opening file, load the data, broken down into sections
	// according to what is found in the PacTire input file
	std::vector<std::list<std::string> > inFile_data;
	std::vector<std::string> inFile_sections;
	this->read_pactire_file(inFile, inFile_data, inFile_sections);

	// now that we have read the data, apply it


	// this bool will allow you to query the pac tire for output
	// Forces, moments based on wheel state info.
	this->params_defined = true;

}

void ChPacTire::read_pactire_file(std::ifstream& m_inFile,
	 std::vector<std::list<std::string> >& m_inFile_data,
	 std::vector<std::string>& m_inFile_sections)
{
	// advance to the first part of the file with data we need to read
	std::string m_line;
	
	while( std::getline(m_inFile, m_line) )
	{
		// first main break
		if( m_line[0] == '$' )
			break;
	}

	// 0:  [UNITS]
	while( std::getline(m_inFile, m_line) ) {
		// made it to the next section
		if( m_line[0] == '$' )
			break;
	}





}

void ChPacTire::WriteOutData(ChSharedBodyPtr spindle, const double time, 
	const std::string outFileName)
{


}

}		// end namespace chrono
