#include "ChGaussPoint.h"

namespace chrono 
{
namespace fem
{
			// Constructor. Creates a Gauss point belonging to element elem, with number
			// n, with coordinates c, with weight w
  ChGaussPoint :: ChGaussPoint(/*ChIntegrationRule *ir, ChElement elem*/ int n, ChVector<> *c, double w)
{
    //irule        = ir;
	//element...
    number       = n;
	LocalCoordinates = *c;
	coordinates  = NULL;
	weight       = w;
	MatrB = new ChMatrixDynamic<>(1,1);
}

			// Destructor
ChGaussPoint :: ~ChGaussPoint()
{

    if ( coordinates ) {
        delete coordinates;
    }
}

}//__end namespace fem
}//__end namespace chrono