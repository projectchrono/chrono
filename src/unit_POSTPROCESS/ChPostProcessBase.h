#ifndef CHPOSTPROCESSBASE_H
#define CHPOSTPROCESSBASE_H
#include <fstream>
#include <string>
#include <sstream>
#include "physics/ChSystem.h"
#include "ChApiPostProcess.h"
using namespace std;

namespace chrono{
	namespace render{
		using namespace chrono;
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