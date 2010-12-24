#ifndef CHLINKGEOMETRIC_H
#define CHLINKGEOMETRIC_H

///////////////////////////////////////////////////
//
//   ChLinkGeometric.h
//
//   Classes for enforcing constraints (contacts)
//   between two objects given some geometric 
//   condition to be satisfied.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLink.h"


namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_GEOMETRIC	36

///
/// Classes for constraints defined by some generic geometric 
/// condition are inherited from this class. 
/// Note that also in the more advanced and powerful ChLinkLock 
/// classes thare are constraints with geometric interpretation,
/// such as the 'point on line', 'spherical joint' etc., but
/// the difference is that the ChLinkLock constraints are 
/// defined between two ChMarker objects (children of two 
/// ChBody objects) so they are less optimized for speed (since
/// the markers could be moving respect to bodies, etc.). On the
/// other hand, links inherited from this ChLinkGeometric are
/// supposed to be less feature-rich but faster in execution.
///

class ChApi ChLinkGeometric : public ChLink {

	CH_RTTI(ChLinkGeometric,ChLink);

protected:
				//
	  			// DATA
				//

public:
				//
	  			// CONSTRUCTORS
				//

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
