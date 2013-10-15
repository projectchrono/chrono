//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChControls.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
 

#include "physics/ChControls.h"
  
namespace chrono 
{



/////////////////////////////////////////////////////////
/// 
///   CLASS
///
///


// Register into the object factory, to enable run-time
// dynamic creation and persistence 
ChClassRegisterABSTRACT<ChControls> a_registration_ChControls;



// 
// FILE I/O
//

void ChControls::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number 
	mstream.VersionWrite(1); 
		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data

}
				
void ChControls::StreamIN(ChStreamInBinary& mstream)
{
		// class version number 
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChObj::StreamIN(mstream);

		// stream in all member data

}





} // END_OF_NAMESPACE____ 



////// end
