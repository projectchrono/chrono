#ifndef CHCOSIMULATION_H
#define CHCOSIMULATION_H

//////////////////////////////////////////////////
//
//   ChCosimulation.h
//
//   A system to connect another software to C::E
//   and perform co-simulation
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
///////////////////////////////////////////////////

#include "chrono_cosimulation/ChSocketFramework.h"
#include "chrono_cosimulation/ChSocket.h"
#include "core/ChMatrix.h"

namespace chrono {

/// Namespace with classes for the cosimulation unit.
namespace cosimul {

/// Class for co-simulation interface.
/// Typically, a C::E program can instance an object
/// from this class and use it to communicate with a 3rd party
/// simulation tool at each time step. The communicaiton is based
/// on TCP sockets, where vectors of scalar values are exchanged
/// back and forth.
/// In this case, C::E will work as a server, waiting for
/// a client to talk with.

class ChApiCosimulation ChCosimulation {
  public:
    /// Create a co-simulation interface.
    ChCosimulation(ChSocketFramework& mframework,
                   int n_in_values,  /// number of scalar variables to receive each timestep
                   int n_out_values  /// number of scalar variables to send each timestep
                   );

    ~ChCosimulation();

    /// Wait for a client to connect to the interface,
    /// on a given port, and wait until not connected.
    /// aport is a free port number, for example 50009.
    bool WaitConnection(int aport);

    /// Exchange data with the client, by sending a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// Simulator actual time is also passed as first value.
    bool SendData(double mtime, ChMatrix<double>* mdata);

    /// Exchange data with the client, by receiving a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// External time is also received as first value.
    bool ReceiveData(double& mtime, ChMatrix<double>* mdata);

  private:
    ChSocketTCP* myServer;
    ChSocketTCP* myClient;
    int nport;

    int in_n;
    int out_n;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header
