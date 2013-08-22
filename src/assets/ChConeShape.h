#ifndef CHOBJCONESHAPE_H
#define CHOBJCONESHAPE_H

///////////////////////////////////////////////////
//
//   ChEllipsoidShape.h
//
//   Class for defining a sphere as an asset shape 
//   that can be visualized in some way.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "assets/ChVisualization.h"
#include "geometry/ChCCone.h"

namespace chrono {

/// Class for referencing a sphere shape that can be 
/// visualized in some way.

class ChApi ChConeShape: public ChVisualization {

	protected:
		//
		// DATA
		//
		geometry::ChCone gcone;

	public:
		//
		// CONSTRUCTORS
		//

		ChConeShape() {
		}
		;
		ChConeShape(geometry::ChCone& mcone) :
				gcone(mcone) {
		}
		;

		virtual ~ChConeShape() {
		}
		;

		//
		// FUNCTIONS
		//

		// Access the sphere geometry
		geometry::ChCone& GetConeGeometry() {
			return gcone;
		}

};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}// END_OF_NAMESPACE____

#endif
