#ifndef CHCYLINDERSHAPE_H
#define CHCYLINDERSHAPE_H

///////////////////////////////////////////////////
//
//   ChCylinderShape.h
//
//   Class for defining a cylinder as an asset shape 
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
#include "geometry/ChCCylinder.h"


namespace chrono
{

/// Class for referencing a cylinder shape that can be 
/// visualized in some way.

class ChApi ChCylinderShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChCylinder gcylinder;	

public:
				//
	  			// CONSTRUCTORS
				//

	ChCylinderShape () {};
	ChCylinderShape (geometry::ChCylinder& mcyl) : gcylinder(mcyl) {};

	virtual ~ChCylinderShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChCylinder& GetCylinderGeometry() {return gcylinder;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
