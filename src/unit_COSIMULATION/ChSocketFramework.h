#ifndef CHSOCKETFRAMEWORK_H
#define CHSOCKETFRAMEWORK_H

//////////////////////////////////////////////////
//
//   ChSocketFramework.h
//
//   Custom exception class, for sockets
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
///////////////////////////////////////////////////




#include "ChSocket.h"


/// A single object of this class must be instantiated before using 
/// all classes related to sockets, because it initializes some platform-specific
/// settings. 
/// Delete it after you do not need sockets anymore.

class ChApiCosimulation ChSocketFramework
{
public:
	ChSocketFramework();
	~ChSocketFramework();
};



#endif
        
