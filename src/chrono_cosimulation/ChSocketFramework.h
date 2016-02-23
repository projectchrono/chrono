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

#include "chrono_cosimulation/ChSocket.h"

namespace chrono {
namespace cosimul {

/// A single object of this class must be instantiated before using
/// all classes related to sockets, because it initializes some platform-specific
/// settings.
/// Delete it after you do not need sockets anymore.

class ChApiCosimulation ChSocketFramework {
  public:
    ChSocketFramework();
    ~ChSocketFramework();
};


}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____


#endif
