// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// =============================================================================
//
// Class that handles communication between nodes. A communicator is agnostic
// to the actual data it passes (i.e. doesn't care the data types/contents).
// This class is simply responsible for sending and receiving (and subsequently)
// serializing/deserializing data passed to it.
//
// This is an abstract base class which has several pure virtual functions to
// override. Derived classes are responsible for implementing Synchronous or
// Asynchronous communication and for sending and receiving data.
//
// =============================================================================

#ifndef SYN_COMMUNICATOR_H
#define SYN_COMMUNICATOR_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

#include <vector>
#include <functional>

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication
/// @{

/// Base class communicator used to establish and facilitate communication between nodes
class SYN_API SynCommunicator {
  public:
    ///@brief Default constructor
    ///
    SynCommunicator();

    ///@brief Destructor
    ///
    virtual ~SynCommunicator();

    ///@brief Initialization method typically responsible for establishing a connection.
    /// Although not necessarily true, this method should handle initial peer-to-peer communication.
    /// This could mean a simple handshake or an actual exchange of information used during the simulation.
    ///
    virtual void Initialize();

    ///@brief This method is responsible for continuous synchronous synchronization steps
    /// This method is the blocking form of the communication interface.
    /// This method could use synchronous method calls to implement its communication interface.
    /// For asynchronous method cases, please use/implement Asynchronize()
    ///
    virtual void Synchronize() = 0;

    ///@brief This method is responsible for blocking until an action is received or done.
    /// For example, a process may call Barrier to wait until another process has established
    /// certain classes and initialized certain quantities. This functionality should be implemented
    /// in this method.
    ///
    virtual void Barrier() = 0;

    // -----------------------------------------------------------------------------------------------

    ///@brief Reset the communicator
    /// Will clear out message buffers
    ///
    void Reset();

    ///@brief Add the messages to the outgoing message buffer
    ///
    ///@param messages a list of handles to messages to add to the outgoing buffer
    void AddOutgoingMessages(SynMessageList& messages);

    /// @brief Adds a quit message to the queue telling other nodes to end the simulation
    void AddQuitMessage();

    ///@brief Add the messages to the incoming message buffer
    ///
    ///@param messages a list of handles to messages to add to the incoming buffer
    void AddIncomingMessages(SynMessageList& messages);

    ///@brief Process a data buffer by passing it to the underlying FlatBuffersManager
    ///
    ///@param data the data to process
    void ProcessBuffer(std::vector<uint8_t>& data);

    ///@brief Get the messages received by the communicator
    ///
    ///@return SynMessageList the received messages
    virtual SynMessageList& GetMessages() { return m_incoming_messages; }

    // -----------------------------------------------------------------------------------------------

  protected:
    bool m_initialized;  ///< whether the communicator has been initialized

    SynMessageList m_incoming_messages;           ///< Incoming messages
    SynFlatBuffersManager m_flatbuffers_manager;  ///< flatbuffer manager for this rank
};

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif