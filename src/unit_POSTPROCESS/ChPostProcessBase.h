#ifndef CHPOSTPROCESSBASE_H
#define CHPOSTPROCESSBASE_H

//////////////////////////////////////////////////
//
//   ChPostProcessBase.h
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <fstream>
#include <string>
#include <sstream>
#include "physics/ChSystem.h"
#include "ChApiPostProcess.h"

namespace chrono{
 namespace postprocess{

		
/// Base class for post processing implementations
class ChApiPostProcess ChPostProcessBase
{
public:
	ChPostProcessBase(ChSystem* system)
		{
			mSystem=system;
		}
	virtual ~ChPostProcessBase(){}


	virtual void SetSystem(ChSystem* system) { mSystem = system; }
	virtual ChSystem* GetSystem() {return mSystem; }


		/// This function is used to export the script that will
		/// be used (by POV, by Matlab or other scripted tools) to
		/// process all the exported data.
		/// (Must be implemented by children classes)
	virtual void ExportScript(const std::string &filename) = 0;

		/// This function is used at each timestep to export data
		/// formatted in a way that it can be load with the processing script.
		/// The user should call this function in the while() loop 
		/// of the simulation, once per frame. 
		/// (Must be implemented by children classes)
	virtual void ExportData(const std::string &filename) = 0;


protected:			
	ChSystem * mSystem;

};


 } // end namespace
} // end namespace

#endif