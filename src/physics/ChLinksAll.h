#ifndef CHLINKSALL_H
#define CHLINKSALL_H

//////////////////////////////////////////////////
//  
//   ChLinksAll.h
//
//   Shortcut to include most headers related to
//   links (joints in 3d space) with a single include..
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


// This header is just a trick to include quickly the 
// following headers, all at once with a single #include "physics/ChLinksAll.h" statement

#include "physics/ChLink.h"
#include "physics/ChLinkLock.h"
#include "physics/ChLinkSpring.h"
#include "physics/ChLinkLinActuator.h"
#include "physics/ChLinkPneumaticActuator.h"
#include "physics/ChLinkEngine.h"
#include "physics/ChLinkScrew.h"
#include "physics/ChLinkGear.h"
#include "physics/ChLinkPulley.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChLinkBrake.h"
#include "physics/ChLinkWheel.h"
#include "physics/ChLinkFastContact.h"
#include "physics/ChLinkClearance.h"
#include "physics/ChLinkPointSpline.h"
//#include "physics/ChLinkPointSurf.h"
#include "physics/ChLinkTrajectory.h"


#endif
