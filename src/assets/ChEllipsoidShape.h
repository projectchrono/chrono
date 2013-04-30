#ifndef CHOBJELLIPSOIDSHAPE_H
#define CHOBJELLIPSOIDSHAPE_H

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
#include "geometry/ChCEllipsoid.h"


namespace chrono
{

/// Class for referencing a sphere shape that can be 
/// visualized in some way.

class ChApi ChEllipsoidShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChEllipsoid gellipsoid;

public:
				//
	  			// CONSTRUCTORS
				//

	ChEllipsoidShape () {};
	ChEllipsoidShape (geometry::ChEllipsoid& mellipsoid) : gellipsoid(mellipsoid) {};

	virtual ~ChEllipsoidShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChEllipsoid& GetEllipsoidGeometry() {return gellipsoid;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
