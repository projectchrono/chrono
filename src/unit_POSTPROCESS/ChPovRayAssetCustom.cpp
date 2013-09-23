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

//////////////////////////////////////////////////
//  
//   ChPovRayAssetCustom.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

     
#include "ChPovRay.h"
#include "ChPovRayAssetCustom.h"

using namespace chrono;
using namespace postprocess;


void ChPovRayAssetCustom::SetCommands (char mcomm[])
{
	this->custom_command = mcomm;
}


