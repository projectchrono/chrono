// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_SOCKET_COMMUNICATION_H
#define CH_SOCKET_COMMUNICATION_H

#include "chrono/core/ChApiCE.h"

#include "chrono/utils/ChSocket.h"

#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Class for socket communication interface.
/// Typically, a Chrono program can instance an object from this class and use it to communicate with a 3rd party
/// simulation tool at each time step. The communication is based on TCP sockets, where vectors of scalar values are
/// exchanged back and forth. In this case, Chrono will work as a server, waiting for a client to talk with.
class ChApi ChSocketCommunication {
  public:
    /// Create a co-simulation interface.
    ChSocketCommunication(utils::ChSocketFramework& framework,  ///< socket framework
                          int n_in_values,                      ///< number of scalar variables to receive each timestep
                          int n_out_values                      ///< number of scalar variables to send each timestep
    );

    ~ChSocketCommunication();

    /// Wait for a client to connect to the interface, on a given port, and wait until not connected.
    /// The argument 'port' is a free port number, for example 50009.
    bool WaitConnection(int port);

    /// Exchange data with the client, by sending a vector of floating point values over TCP socket connection.
    /// Values are double precision, little endian, 4 bytes each. The simulator time is passed as first argument.
    bool SendData(double time, ChVectorConstRef out_data);

    /// Exchange data with the client, by receiving a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// External time is also received as first value.
    bool ReceiveData(double& time, ChVectorRef in_data);

  private:
    chrono::utils::ChSocketTCP* myServer;
    chrono::utils::ChSocketTCP* myClient;
    int nport;

    int in_n;
    int out_n;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
