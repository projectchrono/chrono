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
using namespace std;

namespace chrono{
	namespace render{

		/// Base class for post processing implementations

		class ChApiPostProcess ChRenderBase{
		public:
			ChPostProcessBase(ChSystem* system){
				mSystem=system;
			}
			~ChPostProcessBase(){}

			virtual void ExportData(const string &filename);
			virtual void ExportScript(const string &filename);

		protected:
			ChSystem * mSystem;
		};


	}
}

#endif