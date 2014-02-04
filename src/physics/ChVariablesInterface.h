//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHVARIABLESINTERFACE_H
#define CHVARIABLESINTERFACE_H



#include "lcp/ChLcpSystemDescriptor.h"


namespace chrono
{



	/// Interface class for objects that  contain a single ChLcpVariables item
	/// that must be exposed. 
	/// Often used in multiple-inheritance schemes, see ChBody, ChNodeBase

class ChApi ChVariablesInterface
{
public:
			// Access the 'LCP variables' of the node. To be implemented in children classes
	virtual ChLcpVariables& Variables() =0; 
};





} // END_OF_NAMESPACE____

#endif
