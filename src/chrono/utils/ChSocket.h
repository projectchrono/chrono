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
//
// Based on the work of Liyang Yu in the tutorial of Codeproject
//
// =============================================================================

#ifndef CHSOCKET_H
#define CHSOCKET_H

#if (defined(__linux__) || defined(__APPLE__))
    #define UNIX
#endif

#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <functional>

#ifdef UNIX
    #include <sys/socket.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <cerrno>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <sys/types.h>
    #include <sys/ioctl.h>
    #include <cstring>
    #ifdef __APPLE__
        // be sure to have TARGET_OS_MAC defined
        #include "TargetConditionals.h"
    #endif
#else
    #include <winsock2.h>
#endif

#include "chrono/core/ChApiCE.h"

namespace chrono {
namespace utils {

/// Host info data type.
enum class HostType { NAME, ADDRESS };

// =============================================================================
// ChSocketFramework
// =============================================================================
/// Class for initializing platform-specific socket framework.
/// A single instance of this class *must* be created before using any ChSocket-related class.
class ChApi ChSocketFramework {
  public:
    ChSocketFramework();
    ~ChSocketFramework();
};

// =============================================================================
// ChSocket
// =============================================================================
/// Base class for sockets.
/// Sockets have at least an ID and a port number.
class ChApi ChSocket {
  public:
    /// Create a socket and assign it a unique ID (aka. file descriptor).
    /// Store port number for later use.
    ChSocket(int port_number);

    /// Close and destroy socket.
    virtual ~ChSocket();

    /// Get socket ID (aka. file descriptor).
    int GetSocketId() { return socketId; }

    /// Get socket port number.
    int GetPortNumber() { return portNumber; }

    /// Toggle socket options (either ON or OFF).
    void SetDebug(int debug_toggle);
    void SetReuseAddr(int reuse_toggle);
    void SetKeepAlive(int alive_toggle);
    void SetLingerOnOff(bool linger_on);
    void SetSocketBlocking(int blocking_toggle);

    /// Set linger timeout on socket close (seconds).
    void SetLingerSeconds(int seconds);

    /// Retrieve socket options.
    int GetDebug();
    int GetReuseAddr();
    int GetKeepAlive();
    bool GetLingerOnOff();
    int GetLingerSeconds();
    int GetSocketBlocking();

    /// Set the size of the socket send and receive buffer (bytes).
    void SetSendBufSize(int sendbuff_size);
    void SetReceiveBufSize(int recvbuff_size);

    /// Get the size of the socket send and receive buffer (bytes).
    int GetSendBufSize();
    int GetReceiveBufSize();

    /// Stream socket info.
    friend std::ostream& operator<<(std::ostream& os, ChSocket& socket);

  protected:
    /// Only used when the socket is used for client communication.
    /// Once this is done, the next necessary call is SetSocketId(int).
    ChSocket() {}
    void SetSocketId(int socketFd) { socketId = socketFd; }

    int socketId;            ///< socket file descriptor
    int portNumber;          ///< socket port number
    int blocking;            ///< blocking flag
    int bindFlag;            ///< binding flag
    sockaddr_in clientAddr;  ///< address of the client that sent data

  private:
    /// Get the system errors.
#ifdef _WIN32
    void detectErrorOpenWinSocket(int*, std::string&);
    void detectErrorSetSocketOption(int*, std::string&);
    void detectErrorGetSocketOption(int*, std::string&);
#endif

#ifdef UNIX
    char* sGetError() { return strerror(errno); }
#endif
};

// =============================================================================
// ChSocketTCP
// =============================================================================
/// ChSocket for TCP communications.
/// Three modes for sending|receiving are available:
/// - full-manual mode:
///   + user owns a buffer of type std::vector<char>
///     - single call to SendBuffer(const std::vector<char>&) | ReceiveBuffer(std::vector<char>&, int)
///   + user owns a message of type std::string
///     - single call to SendString(const std::string&) | ReceiveString(std::string&)
/// - templated mode:
///   + for sending:
///     - multiple calls to EnqueueData()
///     - single call to SendDataBuffer()
///   + for receiving:
///     - single call to ReceiveDataBuffer(int)
///     - multiple calls to ParseReceivedData()
/// - full-auto mode
///   + single call to SendData() | ReceiveData(), with all the variables involved
class ChApi ChSocketTCP : public ChSocket {
  public:
    /// Constructor used for creating instances dedicated to client communication.
    /// When accept() is successful, a socketId is generated and returned. This socket id is then used
    /// to build a *new* socket using the following constructor.
    /// Therefore, the next necessary call should be SetSocketId() using this newly generated socket fd.
    ChSocketTCP() {}

    /// Create a TCP socket and assign it a unique ID.
    /// Store port number for later use.
    ChSocketTCP(int port_number);

    /// Destroy TCP socket.
    ~ChSocketTCP() {}

    /// Server-side functionalities --------------------------------------------

    /// Bind the socket to an address and a port number [SERVER SIDE].
    /// Return true on success.
    bool BindSocket();

    /// Configure the socket to listen for incoming client connections [SERVER SIDE].
    /// Optionally specify the maximum length of the pending connections queue (backlog).
    /// Subsequent calls to AcceptClient() will actually perform the connection.
    /// Return true on success.
    bool ListenToClient(int max_pending_connections = 5);

    /// Accept an incoming client connection [SERVER SIDE].
    /// If successful, return a new heap-allocated socket with the ID of the accepted connection (which must be managed by the user).
    /// If unsuccessful, return a null pointer.
    /// The resolved host name of the connected client is returned as function argument.
    ChSocketTCP* AcceptClient(std::string& client_host_name);

    /// Client-side functionalities --------------------------------------------

    /// Connect to a server using the port number configured for this socket [CLIENT SIDE].
    /// Target server can be specified either by host name or IP address.
    /// Return true on success.
    bool ConnectToServer(std::string& server_name_or_addr, HostType host_type);

    /// Manual communication functionalities -----------------------------------

    /// Send a string message to the connected host.
    /// This method automatically prepends a fixed-length header, specifying the message size in bytes.
    /// Return the number of bytes sent.
    int SendString(const std::string& message);

    /// Receive a string message from the connected host into the given argument.
    /// This method automatically determines the message size from a fixed-length header, which is not exposed to the user.
    /// Return the number of bytes received.
    int ReceiveString(std::string& message);

    /// Send a buffer of raw bytes to the connected host.
    /// This method does not send any header, so the receiver must know in advance the length of the buffer.
    /// Return the number of bytes sent.
    int SendBuffer(const std::vector<char>& source_buffer);

    /// Receive a buffer of raw bytes from the connected host into the given argument.
    /// This method does not receive any header, so the caller must know in advance the length of the buffer.
    /// The destination buffer is resized to the specified size in bytes before receiving.
    /// Return the number of bytes received.
    int ReceiveBuffer(std::vector<char>& destination_buffer, int byte_size);

    /// Buffered communication functionalities ---------------------------------

    /// Add data to the internal sending buffer.
    /// When operation is completed, collected data can be sent at once using SendDataBuffer().
    template <typename T>
    void EnqueueData(const T& data_to_send) {
        for (auto s = 0; s < sizeof(data_to_send); ++s) {
            m_internal_buffer_send.push_back(reinterpret_cast<const char*>(&data_to_send)[s]);
        }
    }

    /// Clear the internal sending buffer populated by EnqueueData().
    void ClearInternalSendDataBuffer() { m_internal_buffer_send.clear(); }

    /// Send the data collected into the internal sending buffer and clear it.
    /// Return the number of bytes sent.
    int SendDataBuffer();

    /// Receive a given amount of bytes and store them into the internal receiving buffer.
    /// Received data can be subsequently read  using ParseReceivedData().
    /// This method clears the internal reading buffer before operating.
    /// Return the number of bytes received.
    int ReceiveDataBuffer(int data_in_size);

    /// Copy data from the internal receiving buffer, at the specified byte offset, into the given variable.
    /// This method should be called (potentially multiple times) to access data received by ReceiveBuffer().
    template <typename T>
    void ParseReceivedData(T& data_read, int byte_offset = 0) {
        for (auto s = 0; s < sizeof(data_read); ++s) {
            reinterpret_cast<char*>(&data_read)[s] = m_internal_buffer_receive[byte_offset + s];
        }
    }

    /// Send the given variables.
    /// The sending order respects the order of the arguments.
    /// Return the number of bytes sent.
    template <typename DataInType, typename... DataInTypes>
    int SendData(DataInType data_in_1, DataInTypes... data_ins) {
        EnqueueData(data_in_1);
        return SendData(data_ins...);
    }

    /// Receive the given variables.
    /// The receiving order follows the order of the arguments.
    /// It is not possible to receive only part of the streamed content.
    /// To this purpose use ReceiveBuffer(int) together with ParseReceivedData().
    /// Return the number of bytes received.
    template <typename DataInType, typename... DataInTypes>
    int ReceiveData(DataInType& data_in_1, DataInTypes&... data_ins) {
        receive_offset += sizeof(data_in_1);
        int received_size = ReceiveData(data_ins...);
        if (received_size <= 0) {
            return received_size;
        }
        receive_offset -= sizeof(data_in_1);
        ParseReceivedData(data_in_1, receive_offset);
        return received_size;
    }

  private:
    /// SendData variadic specialization for empty arguments.
    int SendData();

    /// ReceiveData variadic specialization for empty arguments.
    int ReceiveData();

#ifdef _WIN32
    /// Windows NT version of the MSG_WAITALL option.
    int XPReceiveMessage(std::string&);
#endif

    /// Error reporting.
    void detectErrorBind(int*, std::string&);
    void detectErrorSend(int*, std::string&);
    void detectErrorRecv(int*, std::string&);
    void detectErrorConnect(int*, std::string&);
    void detectErrorAccept(int*, std::string&);
    void detectErrorListen(int*, std::string&);

    std::vector<char> m_internal_buffer_send = {};     ///< internal buffer to collect data to be sent
    std::vector<char> m_internal_buffer_receive = {};  ///< internal buffer to collect received data
    int receive_offset = 0;                            ///< internal offset for variadic ReceiveData() function
};

// =============================================================================
// ChSocketHostInfo
// =============================================================================
/*
  Liyang Yu, Jan 9th, 2004, version 0.0

  this is to implement the domain and IP address resolution.

  3 cases are considered:

  1. a host name is given (a host name looks like "www.delta.com"), query
     the IP address of the host

  2. an IP address is given (an IP address looks like 10.6.17.184), query
     the host name

  in the above two cases, the IP address and the host name are the same thing:
  since IP address is hard to remember, they are usually aliased by a name, and this
  name is known as the host name.

  3. nothing is given. in other words, we don't know the host name or the IP address.
     in this case, the standard host name for the current processor is used
*/

/// Class for storing information about a TCP host in socket communication, e.g. with an IP address.
class ChApi ChSocketHostInfo {
  public:
    /// Initialize host information using standard parameters.
    ChSocketHostInfo();

    /// Retrieve host information based on the host name or address.
    ChSocketHostInfo(const std::string& host_name, HostType type);

    /// Close and destroy the host information database.
    ~ChSocketHostInfo();

    /// Retrieve the hosts IP address in dot x.y.z.w notation.
    char* GetHostIPAddress();

    /// Retrieve the hosts name.
    char* GetHostName() { return hostPtr->h_name; }

#ifdef UNIX

    /// Retrieves the next host entry in the database.
    char getNextHost();

    /// Opens the host entry database.
    void openHostDb() {
        endhostent();
        searchHostDB = 1;
        sethostent(1);
    }

#endif

  private:
#ifdef _WIN32
    void detectErrorGethostbyname(int*, std::string&);
    void detectErrorGethostbyaddr(int*, std::string&);
#endif

    struct hostent* hostPtr;  ///< entry within the host address database
#ifdef UNIX
    char searchHostDB;  ///< search the host database flag
#endif
};

}  // namespace utils
}  // end namespace chrono

#endif
