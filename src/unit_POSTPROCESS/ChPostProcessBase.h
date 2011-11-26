#ifndef CHRENDERBASE_H
#define CHRENDERBASE_H
#include <fstream>
#include <string>
#include <sstream>
#include "physics/ChSystem.h"
using namespace std;

namespace chrono{
	namespace render{
		using namespace chrono;
		class ChApiRender ChRenderBase{
		public:
			ChRenderBase(ChSystem* sys){
				system=sys;
			}
			~ChRenderBase(){}

			virtual void Export(const string &filename);

		protected:
			ofsteam output_file;
			string file_name;
			ChSystem * system;
		};
	}
}

#endif