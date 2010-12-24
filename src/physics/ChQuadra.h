#ifndef CHQUADRA_H
#define CHQUADRA_H

//////////////////////////////////////////////////
//  
//   ChQuadra.h
//
//   Numerical integration for quadrature.
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


#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "physics/ChFem.h"

namespace chrono 
{



#define CHCLASS_QUADRATURE 7

/////////////////////////////////
//
//  GAUSS QUADRATURE
//
/// Class implementing methods to perform Gauss 
/// quadrature (definite integral over nD intervals)
///

class ChApi ChQuadrature: public ChObj
{
private:

public:
	ChQuadrature() {};
	~ChQuadrature() {};

		// Performs the Gauss quadrature (numerical integration) of a function,
		// passed as parameter. The function must have three xyz parameters, to 
		// allow at least 3d integrals, and must return a pointer to a matrix, ie.
		// the numerically evaluated value/vector/matrix of the function at the
		// xyz point. Note; the xyz parameters must range into the cubic space
		// of limit -1 .. +1  , so the integrated function must be a _normalized_
		// function!. 
 
	void GaussIntegrate (ChFelem* element, 
					ChMatrix<>* result, int orderX, int orderY, int orderZ, int dimensions);
};


} // END_OF_NAMESPACE____

#endif // eof

