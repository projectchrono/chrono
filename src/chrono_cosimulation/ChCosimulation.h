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

#ifndef CHCOSIMULATION_H
#define CHCOSIMULATION_H

#include "chrono_cosimulation/ChApiCosimulation.h"

#include "chrono/utils/ChSocket.h"

#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace cosimul {

/// @addtogroup cosimulation_module
/// @{

/// Class for cosimulation interface.
/// Typically, a C::E program can instance an object
/// from this class and use it to communicate with a 3rd party
/// simulation tool at each time step. The communication is based
/// on TCP sockets, where vectors of scalar values are exchanged
/// back and forth.
/// In this case, C::E will work as a server, waiting for
/// a client to talk with.

class ChApiCosimulation ChCosimulation {
  public:
    /// Create a co-simulation interface.
    ChCosimulation(utils::ChSocketFramework& mframework,  ///< socket framework
                   int n_in_values,                       ///< number of scalar variables to receive each timestep
                   int n_out_values                       ///< number of scalar variables to send each timestep
    );

    ~ChCosimulation();

    /// Wait for a client to connect to the interface,
    /// on a given port, and wait until not connected.
    /// \a aport is a free port number, for example 50009.
    bool WaitConnection(int aport);

    /// Exchange data with the client, by sending a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// Simulator actual time is also passed as first value.
    bool SendData(double mtime, ChVectorConstRef mdata);

    /// Exchange data with the client, by receiving a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// External time is also received as first value.
    bool ReceiveData(double& mtime, ChVectorRef mdata);

  private:
    chrono::utils::ChSocketTCP* myServer;
    chrono::utils::ChSocketTCP* myClient;
    int nport;

    int in_n;
    int out_n;
};

/// @} cosimulation_module

}  // end namespace cosimul
}  // end namespace chrono

#endif
