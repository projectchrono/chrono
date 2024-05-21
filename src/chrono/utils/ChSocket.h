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

#include "chrono/core/ChApiCE.h"

#ifdef UNIX
    #include <sys/socket.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <cerrno>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <iostream>
    #include <sys/types.h>
    #include <sys/ioctl.h>
    #include <cstdio>
    #include <cstring>
    #ifdef __APPLE__
        // be sure to have TARGET_OS_MAC defined
        #include "TargetConditionals.h"
    #endif
#else
    #include <winsock2.h>
#endif

#include <cstdio>
#include <vector>
#include <functional>

namespace chrono {
namespace utils {

// For getting host info
enum hostType { NAME, ADDRESS };
const int HOST_NAME_LENGTH = 64;

// so far we only consider the TCP socket, UDP will be added in later release
// const int MAX_RECV_LEN = 1048576;
const int MAX_RECV_LEN = 8096;
const int MAX_MSG_LEN = 1024;
const int PORTNUM = 1200;

/// Base class for sockets. Sockets have at least an ID and a port.
class ChApi ChSocket {
  public:
    // given a port number, create a socket
    ChSocket(int);

    virtual ~ChSocket();

    // socket options : ON/OFF
    void setDebug(int);
    void setReuseAddr(int);
    void setKeepAlive(int);
    void setLingerOnOff(bool);
    void setLingerSeconds(int);
    void setSocketBlocking(int);

    // size of the send and receive buffer
    void setSendBufSize(int);
    void setReceiveBufSize(int);

    // retrieve socket option settings
    int getDebug();
    int getReuseAddr();
    int getKeepAlive();
    int getSendBufSize();
    int getReceiveBufSize();
    int getSocketBlocking() { return blocking; }
    int getLingerSeconds();
    bool getLingerOnOff();

    // returns the socket file descriptor
    int getSocketId() { return socketId; }

    // returns the port number
    int getPortNumber() { return portNumber; }

    // show the socket
    friend std::ostream& operator<<(std::ostream&, ChSocket&);

  protected:
    /*
       only used when the socket is used for client communication
       once this is done, the next necessary call is setSocketId(int)
    */
    ChSocket() {}
    void setSocketId(int socketFd) { socketId = socketFd; }

    int portNumber;  // Socket port number
    int socketId;    // Socket file descriptor

    int blocking;  // Blocking flag
    int bindFlag;  // Binding flag

    struct sockaddr_in clientAddr;  // Address of the client that sent data

  private:
// Gets the system error
#ifdef _WIN32
    void detectErrorOpenWinSocket(int*, std::string&);
    void detectErrorSetSocketOption(int*, std::string&);
    void detectErrorGetSocketOption(int*, std::string&);
#endif

#ifdef UNIX
    char* sGetError() { return strerror(errno); }
#endif
};

/// ChSocket for TCP communications.
/// Three modes for sending|receiving are available:
/// - full-manual mode:
///   + the user owns its own buffer of type std::vector<char>
///   + calls SendBuffer(std::vector<char>&) | ReceiveBuffer(std::vector<char>&, int) once
/// - templated mode (sending):
///   + multiple calls to Enqueue()
///   + call SendBuffer() once
/// - templated mode (receiving):
///   + call ReceiveBuffer(int)
///   + multiple calls to ParseReceivedData()
/// - full-auto mode
///   + call SendData() | ReceiveData() with all the variables to be sent
class ChApi ChSocketTCP : public ChSocket {
  public:
    /// Constructor used for creating instances dedicated to client communication.
    /// When accept() is successful, a socketId is generated and returned this socket id is then used to build a new
    /// socket using the following constructor, therefore, the next necessary call should be setSocketId() using this
    /// newly generated socket fd.
    ChSocketTCP() {}
    ~ChSocketTCP() {}

    /// Constructor.  Used to create a new TCP socket given a port.
    ChSocketTCP(int portId) : ChSocket(portId) {}

    /// Send a std::string to the connected host.
    /// Note that the string size is handled automatically because there is
    /// a header that tells the length of the string in bytes.
    int sendMessage(std::string&);

    /// Receive a std::string from the connected host.
    /// Note that the string size is handled automatically because there is
    /// a header that tells the length of the string in bytes.
    int receiveMessage(std::string&);

    /// Send a std::vector<char> (a buffer of bytes) to the connected host,
    /// without the header as in SendMessage (so the receiver must know in advance
    /// the length of the buffer).
    int SendBuffer(std::vector<char>& source_buf  ///< source buffer
    );

    /// Receive a std::vector<char> (a buffer of bytes) from the connected host,
    /// without the header as in SendMessage (so one must know in advance
    /// the length of the buffer). If the receiving buffer size is not =bsize, it
    /// will be resized before receiving.
    int ReceiveBuffer(std::vector<char>& dest_buf,  ///< destination buffer - will be resized
                      int bsize                     ///< size in bytes of expected received buffer.
    );

    /// Add data to an internal sending buffer.
    /// When operation is completed, send all data at once with SendBuffer().
    template <typename T>
    void Enqueue(const T& data_to_send) {
        for (auto s = 0; s < sizeof(data_to_send); ++s) {
            m_internal_buffer_send.push_back(reinterpret_cast<const char*>(&data_to_send)[s]);
        }
    }

    /// Clear the internal send buffer.
    void ClearInternalSendBuffer() { m_internal_buffer_send.clear(); }

    /// Send data collected in internal buffer.
    int SendBuffer();

    /// Receive a given amount of bytes.
    int ReceiveBuffer(int data_in_size);

    /// Read the received buffer, at the specified offset, into the given variable.
    template <typename T>
    void ParseReceivedData(T& data_read, int offset = 0) {
        for (auto s = 0; s < sizeof(data_read); ++s) {
            reinterpret_cast<char*>(&data_read)[s] = m_internal_buffer_receive[offset + s];
        }
    }

    /// Send the given variables.
    /// The sending order respects the order of the arguments.
    template <typename DataInType, typename... DataInTypes>
    int SendData(DataInType data_in_1, DataInTypes... data_ins) {
        Enqueue(data_in_1);
        return SendData(data_ins...);
    }

    /// Receive the given variables.
    /// The receiving order follows the order of the arguments.
    /// It is not possible to receive only part of the streamed content.
    /// To this purpose use ReceiveBuffer(int) together with ParseReceivedData()
    template <typename DataInType, typename... DataInTypes>
    int ReceiveData(DataInType& data_in_1, DataInTypes&... data_ins) {
        receive_id += sizeof(data_in_1);
        int received_size = ReceiveData(data_ins...); 
        if (received_size <= 0) {
            return received_size;
        }
        receive_id -= sizeof(data_in_1);
        ParseReceivedData(data_in_1, receive_id);
        return received_size;
    }

    /// Binds the socket to an address and port number
    /// (a server call)
    void bindSocket();

    /// Accepts a connecting client.  The address of the connected client
    /// is stored in the parameter
    /// (a server call)
    ChSocketTCP* acceptClient(std::string&);

    /// Listens to connecting clients,
    /// (a server call)
    void listenToClient(int numPorts = 5);

    /// Connects to the server, a client call
    virtual void connectToServer(std::string&, hostType);

  private:
#ifdef _WIN32
    // Windows NT version of the MSG_WAITALL option
    int XPReceiveMessage(std::string&);
#endif

    // SendData variadic specialization for empty arguments
    int SendData() { return SendBuffer(); }

    // ReceiveData variadic specialization for empty arguments
    int ReceiveData();

    std::vector<char> m_internal_buffer_send = {};     ///< internal buffer to collect data to be sent
    std::vector<char> m_internal_buffer_receive = {};  ///< internal buffer to collect received data
    int receive_id = 0;

    void detectErrorBind(int*, std::string&);
    void detectErrorSend(int*, std::string&);
    void detectErrorRecv(int*, std::string&);
    void detectErrorConnect(int*, std::string&);
    void detectErrorAccept(int*, std::string&);
    void detectErrorListen(int*, std::string&);
};

/// A single object of this class must be instantiated before using
/// all classes related to sockets, because it initializes some platform-specific settings.
class ChApi ChSocketFramework {
  public:
    ChSocketFramework();
    ~ChSocketFramework();
};

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
  private:
#ifdef UNIX
    char searchHostDB;  ///< search the host database flag
#endif

    struct hostent* hostPtr;  ///< entry within the host address database

  public:
    ChSocketHostInfo();

    /// Retrieves the host entry based on the host name or address.
    ChSocketHostInfo(const std::string& hostName, hostType type);

    /// Destructor.  Closes the host entry database.
    ~ChSocketHostInfo() {
#ifdef UNIX
        endhostent();
#endif
    }

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

    /// Retrieve the hosts IP address in dot x.y.z.w notation.
    char* getHostIPAddress();

    /// Retrieve the hosts name.
    char* getHostName() { return hostPtr->h_name; }

  private:
#ifdef _WIN32
    void detectErrorGethostbyname(int*, std::string&);
    void detectErrorGethostbyaddr(int*, std::string&);
#endif
};

}  // namespace utils
}  // end namespace chrono

#endif
