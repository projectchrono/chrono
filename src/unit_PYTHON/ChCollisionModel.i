%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionModel.h"

using namespace collision;

ZZZZZZZZZZZZ____ERRROR___HEADER_NEVER_LOADED????

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/*
// This was an abstract class, but still we need its wrapper, so:
%feature("notabstract") chrono::collision::ChCollisionModel;
%feature("notabstract") ChCollisionModel;
%feature("director") chrono::collision::ChCollisionModel;
%feature("director") collision::ChCollisionModel;
%feature("director") ChCollisionModel;
%feature("director") chrono::ChCollisionModel;

// Parse the header file to generate wrappers 
%include "../collision/ChCCollisionModel.h"    

%feature("director") chrono::collision::ChCollisionModel;
%feature("director") collision::ChCollisionModel;
%feature("director") ChCollisionModel;
%feature("director") chrono::ChCollisionModel;
*/

/*
%feature("director") ChCollMod;

	class ChCollMod : public chrono::collision::ChCollisionModel
	{
		public:
			ChCollMod() {};
			int Foo();
	};
*/


