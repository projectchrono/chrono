//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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

	virtual void Update () {};

};


typedef ChSharedPtr<ChAsset> ChSharedAssetPtr;

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
