#ifndef CHASSET_H
#define CHASSET_H

///////////////////////////////////////////////////
//
//   ChAsset.h
//
//   Classes for adding user data (such as rendering
//   shapes, reference to files) to physical items
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChShared.h"


namespace chrono
{


/// Classes for adding user data (such as rendering
/// shapes, reference to files) to ChPhysicsItem objects.
/// User can inherit his classes for custom assets from
/// this class. A single asset might be shared.


	class ChApi ChAsset : public ChShared {

protected:
				//
	  			// DATA
				//
	

public:
				//
	  			// CONSTRUCTORS
				//

	ChAsset () {};

	virtual ~ChAsset () {};

				//
	  			// FUNCTIONS
				//

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
