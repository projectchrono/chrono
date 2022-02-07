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

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "chrono/utils/ChSocket.h"

namespace chrono {
namespace utils {

const int MSG_HEADER_LEN = sizeof(uint64_t);

ChSocket::ChSocket(int pNumber) {
    portNumber = pNumber;
    blocking = 1;

    try {
        if ((socketId = (int)::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "";
            detectErrorOpenWinSocket(&errorCode, errorMsg);
            ChExceptionSocket* openWinSocketException = new ChExceptionSocket(errorCode, errorMsg);
            throw openWinSocketException;
#endif

#ifdef UNIX
            ChExceptionSocket* openUnixSocketException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw openUnixSocketException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    /*
       set the initial address of client that shall be communicated with to
       any address as long as they are using the same port number.
       The clientAddr structure is used in the future for storing the actual
       address of client applications with which communication is going
       to start
    */
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    clientAddr.sin_port = htons(portNumber);
}

ChSocket::~ChSocket() {
#ifdef WINDOWS_XP
    closesocket(socketId);
#else
    close(socketId);
#endif
}

void ChSocket::setDebug(int debugToggle) {
    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_DEBUG, (char*)&debugToggle, sizeof(debugToggle)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "DEBUG option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setReuseAddr(int reuseToggle) {
    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, (char*)&reuseToggle, sizeof(reuseToggle)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "REUSEADDR option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setKeepAlive(int aliveToggle) {
    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_KEEPALIVE, (char*)&aliveToggle, sizeof(aliveToggle)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "ALIVE option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setLingerSeconds(int seconds) {
    struct linger lingerOption;

    if (seconds > 0) {
        lingerOption.l_linger = seconds;
        lingerOption.l_onoff = 1;
    } else
        lingerOption.l_onoff = 0;

    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_LINGER, (char*)&lingerOption, sizeof(struct linger)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "LINGER option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setLingerOnOff(bool lingerOn) {
    struct linger lingerOption;

    if (lingerOn)
        lingerOption.l_onoff = 1;
    else
        lingerOption.l_onoff = 0;

    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_LINGER, (char*)&lingerOption, sizeof(struct linger)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "LINGER option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setSendBufSize(int sendBufSize) {
    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_SNDBUF, (char*)&sendBufSize, sizeof(sendBufSize)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "SENDBUFSIZE option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setReceiveBufSize(int receiveBufSize) {
    try {
        if (setsockopt(socketId, SOL_SOCKET, SO_RCVBUF, (char*)&receiveBufSize, sizeof(receiveBufSize)) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "RCVBUF option:";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

void ChSocket::setSocketBlocking(int blockingToggle) {
    if (blockingToggle) {
        if (getSocketBlocking())
            return;
        else
            blocking = 1;
    } else {
        if (!getSocketBlocking())
            return;
        else
            blocking = 0;
    }

    try {
#ifdef WINDOWS_XP
        if (ioctlsocket(socketId, FIONBIO, (unsigned long*)&blocking) == -1) {
            int errorCode;
            std::string errorMsg = "Blocking option: ";
            detectErrorSetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
        }
#endif

#ifdef UNIX
        if (ioctl(socketId, FIONBIO, (char*)&blocking) == -1) {
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
        }
#endif
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

int ChSocket::getDebug() {
    int myOption;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(myOption);
#else
    int myOptionLen = sizeof(myOption);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_DEBUG, (char*)&myOption, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get DEBUG option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    return myOption;
}

int ChSocket::getReuseAddr() {
    int myOption;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(myOption);
#else
    int myOptionLen = sizeof(myOption);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, (char*)&myOption, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get REUSEADDR option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    return myOption;
}

int ChSocket::getKeepAlive() {
    int myOption;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(myOption);
#else
    int myOptionLen = sizeof(myOption);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_KEEPALIVE, (char*)&myOption, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get KEEPALIVE option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
    return myOption;
}

int ChSocket::getLingerSeconds() {
    struct linger lingerOption;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(struct linger);
#else
    int myOptionLen = sizeof(struct linger);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_LINGER, (char*)&lingerOption, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get LINER option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    return lingerOption.l_linger;
}

bool ChSocket::getLingerOnOff() {
    struct linger lingerOption;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(struct linger);
#else
    int myOptionLen = sizeof(struct linger);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_LINGER, (char*)&lingerOption, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get LINER option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    if (lingerOption.l_onoff == 1)
        return true;
    else
        return false;
}

int ChSocket::getSendBufSize() {
    int sendBuf;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(sendBuf);
#else
    int myOptionLen = sizeof(sendBuf);
#endif

    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_SNDBUF, (char*)&sendBuf, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get SNDBUF option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
    return sendBuf;
}

int ChSocket::getReceiveBufSize() {
    int rcvBuf;
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t myOptionLen = sizeof(rcvBuf);
#else
    int myOptionLen = sizeof(rcvBuf);
#endif
    try {
        if (getsockopt(socketId, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBuf, &myOptionLen) == -1) {
#ifdef WINDOWS_XP
            int errorCode;
            std::string errorMsg = "get RCVBUF option: ";
            detectErrorGetSocketOption(&errorCode, errorMsg);
            ChExceptionSocket* socketOptionException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketOptionException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketOptionException = new ChExceptionSocket(0, "unix: error getting host by name");
            throw unixSocketOptionException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
    return rcvBuf;
}

#ifdef WINDOWS_XP
void ChSocket::detectErrorOpenWinSocket(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("Successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem or the associated service provider has failed.");
    else if (*errCode == WSAEAFNOSUPPORT)
        errMsg.append("The specified address family is not supported.");
    else if (*errCode == WSAEINPROGRESS)
        errMsg.append(
            "A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a "
            "callback function.");
    else if (*errCode == WSAEMFILE)
        errMsg.append("No more socket descriptors are available.");
    else if (*errCode == WSAENOBUFS)
        errMsg.append("No buffer space is available. The socket cannot be created.");
    else if (*errCode == WSAEPROTONOSUPPORT)
        errMsg.append("The specified protocol is not supported.");
    else if (*errCode == WSAEPROTOTYPE)
        errMsg.append("The specified protocol is the wrong type for this socket.");
    else if (*errCode == WSAESOCKTNOSUPPORT)
        errMsg.append("The specified socket type is not supported in this address family.");
    else
        errMsg.append("unknown problems!");
}

void ChSocket::detectErrorSetSocketOption(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEFAULT)
        errMsg.append("optval is not in a valid part of the process address space or optlen parameter is too small.");
    else if (*errCode == WSAEINPROGRESS)
        errMsg.append(
            "A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a "
            "callback function.");
    else if (*errCode == WSAEINVAL)
        errMsg.append("level is not valid, or the information in optval is not valid.");
    else if (*errCode == WSAENETRESET)
        errMsg.append("Connection has timed out when SO_KEEPALIVE is set.");
    else if (*errCode == WSAENOPROTOOPT)
        errMsg.append(
            "The option is unknown or unsupported for the specified provider or socket (see SO_GROUP_PRIORITY "
            "limitations).");
    else if (*errCode == WSAENOTCONN)
        errMsg.append("Connection has been reset when SO_KEEPALIVE is set.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else
        errMsg.append("unknown problem!");
}

void ChSocket::detectErrorGetSocketOption(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEFAULT)
        errMsg.append(
            "One of the optval or the optlen parameters is not a valid part of the user address space, or the optlen "
            "parameter is too small.");
    else if (*errCode == WSAEINPROGRESS)
        errMsg.append(
            "A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a "
            "callback function.");
    else if (*errCode == WSAEINVAL)
        errMsg.append("The level parameter is unknown or invalid.");
    else if (*errCode == WSAENOPROTOOPT)
        errMsg.append("The option is unknown or unsupported by the indicated protocol family.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");

    else
        errMsg.append("unknown problems!");
}

#endif

std::ostream& operator<<(std::ostream& io, ChSocket& s) {
    std::string flagStr = "";

    io << "--------------- Summary of socket settings -------------------" << std::endl;
    io << "   Socket Id:     " << s.getSocketId() << std::endl;
    io << "   port #:        " << s.getPortNumber() << std::endl;
    io << "   debug:         " << (flagStr = s.getDebug() ? "true" : "false") << std::endl;
    io << "   reuse addr:    " << (flagStr = s.getReuseAddr() ? "true" : "false") << std::endl;
    io << "   keep alive:    " << (flagStr = s.getKeepAlive() ? "true" : "false") << std::endl;
    io << "   send buf size: " << s.getSendBufSize() << std::endl;
    io << "   recv bug size: " << s.getReceiveBufSize() << std::endl;
    io << "   blocking:      " << (flagStr = s.getSocketBlocking() ? "true" : "false") << std::endl;
    io << "   linger on:     " << (flagStr = s.getLingerOnOff() ? "true" : "false") << std::endl;
    io << "   linger seconds: " << s.getLingerSeconds() << std::endl;
    io << "----------- End of Summary of socket settings ----------------" << std::endl;
    return io;
}

void ChSocketTCP::bindSocket() {
    try {
        if (::bind(socketId, (struct sockaddr*)&clientAddr, sizeof(struct sockaddr_in)) == -1) {
#ifdef WINDOWS_XP
            int errorCode = 0;
            std::string errorMsg = "error calling bind(): \n";
            detectErrorBind(&errorCode, errorMsg);
            ChExceptionSocket* socketBindException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketBindException;
#endif
#ifdef UNIX
            ChExceptionSocket* unixSocketBindException = new ChExceptionSocket(0, "unix: error calling bind()");
            throw unixSocketBindException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

#ifdef WINDOWS_XP

void ChSocketTCP::detectErrorBind(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEADDRINUSE) {
        errMsg.append("A process on the machine is already bound to the same\n");
        errMsg.append("fully-qualified address and the socket has not been marked\n");
        errMsg.append("to allow address re-use with SO_REUSEADDR. For example,\n");
        errMsg.append("IP address and port are bound in the af_inet case");
    } else if (*errCode == WSAEADDRNOTAVAIL)
        errMsg.append("The specified address is not a valid address for this machine.");
    else if (*errCode == WSAEFAULT) {
        errMsg.append("The name or the namelen parameter is not a valid part of\n");
        errMsg.append("the user address space, the namelen parameter is too small,\n");
        errMsg.append("the name parameter contains incorrect address format for the\n");
        errMsg.append("associated address family, or the first two bytes of the memory\n");
        errMsg.append("block specified by name does not match the address family\n");
        errMsg.append("associated with the socket descriptor s.");
    } else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress, or the\n");
        errMsg.append("service provider is still processing a callback function.");
    } else if (*errCode == WSAEINVAL)
        errMsg.append("The socket is already bound to an address. ");
    else if (*errCode == WSAENOBUFS)
        errMsg.append("Not enough buffers available, too many connections.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else
        errMsg.append("unknown problems!");
}

void ChSocketTCP::detectErrorRecv(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEFAULT)
        errMsg.append("The buf parameter is not completely contained in a valid part of the user address space.");
    else if (*errCode == WSAENOTCONN)
        errMsg.append("The socket is not connected.");
    else if (*errCode == WSAEINTR)
        errMsg.append("The (blocking) call was canceled through WSACancelBlockingCall.");
    else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress, or the\n");
        errMsg.append("service provider is still processing a callback function.");
    } else if (*errCode == WSAENETRESET) {
        errMsg.append("The connection has been broken due to the keep-alive activity\n");
        errMsg.append("detecting a failure while the operation was in progress.");
    } else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else if (*errCode == WSAEOPNOTSUPP) {
        errMsg.append("MSG_OOB was specified, but the socket is not stream-style\n");
        errMsg.append("such as type SOCK_STREAM, out-of-band data is not supported\n");
        errMsg.append("in the communication domain associated with this socket, or\n");
        errMsg.append("the socket is unidirectional and supports only send operations.");
    } else if (*errCode == WSAESHUTDOWN) {
        errMsg.append("The socket has been shut down; it is not possible to recv on a\n");
        errMsg.append("socket after shutdown has been invoked with how set to SD_RECEIVE or SD_BOTH.");
    } else if (*errCode == WSAEWOULDBLOCK)
        errMsg.append("The socket is marked as nonblocking and the receive operation would block.");
    else if (*errCode == WSAEMSGSIZE)
        errMsg.append("The message was too large to fit into the specified buffer and was truncated.");
    else if (*errCode == WSAEINVAL) {
        errMsg.append("The socket has not been bound with bind, or an unknown flag\n");
        errMsg.append("was specified, or MSG_OOB was specified for a socket with\n");
        errMsg.append("SO_OOBINLINE enabled or (for byte stream sockets only) len was zero or negative.");
    } else if (*errCode == WSAECONNABORTED) {
        errMsg.append("The virtual circuit was terminated due to a time-out or\n");
        errMsg.append("other failure. The application should close the socket as it is no longer usable.");
    } else if (*errCode == WSAETIMEDOUT) {
        errMsg.append("The connection has been dropped because of a network\n");
        errMsg.append("failure or because the peer system failed to respond.");
    } else if (*errCode == WSAECONNRESET) {
        errMsg.append("The virtual circuit was reset by the remote side executing a\n");
        errMsg.append("\"hard\" or \"abortive\" close. The application should close\n");
        errMsg.append("the socket as it is no longer usable. On a UDP datagram socket\n");
        errMsg.append("this error would indicate that a previous send operation\n");
        errMsg.append("resulted in an ICMP \"Port Unreachable\" message.");
    } else
        errMsg.append("unknown problems!");
}

void ChSocketTCP::detectErrorConnect(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEADDRINUSE) {
        errMsg.append("The socket's local address is already in use and the socket\n");
        errMsg.append("was not marked to allow address reuse with SO_REUSEADDR. This\n");
        errMsg.append("error usually occurs when executing bind, but could be delayed\n");
        errMsg.append("until this function if the bind was to a partially wild-card\n");
        errMsg.append("address (involving ADDR_ANY) and if a specific address needs\n");
        errMsg.append("to be committed at the time of this function.");
    } else if (*errCode == WSAEINTR)
        errMsg.append("The (blocking) Windows Socket 1.1 call was canceled through WSACancelBlockingCall.");
    else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress, or\n");
        errMsg.append("the service provider is still processing a callback function.");
    } else if (*errCode == WSAEALREADY) {
        errMsg.append("A nonblocking connect call is in progress on the specified socket.\n");
        errMsg.append("Note In order to preserve backward compatibility, this error is\n");
        errMsg.append("reported as WSAEINVAL to Windows Sockets 1.1 applications that\n");
        errMsg.append("link to either WINSOCK.DLL or WSOCK32.DLL.");
    } else if (*errCode == WSAEADDRNOTAVAIL)
        errMsg.append("The remote address is not a valid address (such as ADDR_ANY).");
    else if (*errCode == WSAEAFNOSUPPORT)
        errMsg.append("Addresses in the specified family cannot be used with this socket.");
    else if (*errCode == WSAECONNREFUSED)
        errMsg.append("The attempt to connect was forcefully rejected.");
    else if (*errCode == WSAEFAULT) {
        errMsg.append("The name or the namelen parameter is not a valid part of\n");
        errMsg.append("the user address space, the namelen parameter is too small,\n");
        errMsg.append("or the name parameter contains incorrect address format for\n");
        errMsg.append("the associated address family.");
    } else if (*errCode == WSAEINVAL) {
        errMsg.append("The parameter s is a listening socket, or the destination\n");
        errMsg.append("address specified is not consistent with that of the constrained\n");
        errMsg.append("group the socket belongs to.");
    } else if (*errCode == WSAEISCONN)
        errMsg.append("The socket is already connected (connection-oriented sockets only).");
    else if (*errCode == WSAENETUNREACH)
        errMsg.append("The network cannot be reached from this host at this time.");
    else if (*errCode == WSAENOBUFS)
        errMsg.append("No buffer space is available. The socket cannot be connected.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else if (*errCode == WSAETIMEDOUT)
        errMsg.append("Attempt to connect timed out without establishing a connection.");
    else if (*errCode == WSAEWOULDBLOCK) {
        errMsg.append("The socket is marked as nonblocking and the connection\n");
        errMsg.append("cannot be completed immediately.");
    } else if (*errCode == WSAEACCES) {
        errMsg.append("Attempt to connect datagram socket to broadcast address failed\n");
        errMsg.append("because setsockopt option SO_BROADCAST is not enabled.");
    } else
        errMsg.append("unknown problems!");
}

void ChSocketTCP::detectErrorAccept(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this FUNCTION.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEFAULT)
        errMsg.append("The addrlen parameter is too small or addr is not a valid part of the user address space.");
    else if (*errCode == WSAEINTR)
        errMsg.append("A blocking Windows Sockets 1.1 call was canceled through WSACancelBlockingCall.");
    else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress, or the\n");
        errMsg.append("service provider is still processing a callback function.");
    } else if (*errCode == WSAEINVAL)
        errMsg.append("The listen function was not invoked prior to accept.");
    else if (*errCode == WSAEMFILE)
        errMsg.append("The queue is nonempty upon entry to accept and there are no descriptors available.");
    else if (*errCode == WSAENOBUFS)
        errMsg.append("No buffer space is available.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else if (*errCode == WSAEOPNOTSUPP)
        errMsg.append("The referenced socket is not a type that supports connection-oriented service.");
    else if (*errCode == WSAEWOULDBLOCK)
        errMsg.append("The socket is marked as nonblocking and no connections are present to be accepted.");
    else
        errMsg.append("unknown problems!");
}

void ChSocketTCP::detectErrorListen(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEADDRINUSE) {
        errMsg.append("The socket's local address is already in use and the socket was\n");
        errMsg.append("not marked to allow address reuse with SO_REUSEADDR. This error\n");
        errMsg.append("usually occurs during execution of the bind function, but could\n");
        errMsg.append("be delayed until this function if the bind was to a partially\n");
        errMsg.append("wild-card address (involving ADDR_ANY) and if a specific address\n");
        errMsg.append("needs to be \"committed\" at the time of this function.");
    } else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress, or the service\n");
        errMsg.append("provider is still processing a callback function.");
    } else if (*errCode == WSAEINVAL)
        errMsg.append("The socket has not been bound with bind.");
    else if (*errCode == WSAEISCONN)
        errMsg.append("The socket is already connected.");
    else if (*errCode == WSAEMFILE)
        errMsg.append("No more socket descriptors are available.");
    else if (*errCode == WSAENOBUFS)
        errMsg.append("No buffer space is available.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else if (*errCode == WSAEOPNOTSUPP)
        errMsg.append("The referenced socket is not of a type that supports the listen operation.");
    else
        errMsg.append("unknown problems!");
}

void ChSocketTCP::detectErrorSend(int* errCode, std::string& errMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errMsg.append("A successful WSAStartup must occur before using this function.");
    else if (*errCode == WSAENETDOWN)
        errMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAEACCES) {
        errMsg.append("The requested address is a broadcast address,\n");
        errMsg.append("but the appropriate flag was not set. Call setsockopt\n");
        errMsg.append("with the SO_BROADCAST parameter to allow the use of the broadcast address.");
    } else if (*errCode == WSAEINTR) {
        errMsg.append("A blocking Windows Sockets 1.1 call was canceled\n");
        errMsg.append("through WSACancelBlockingCall.");
    } else if (*errCode == WSAEINPROGRESS) {
        errMsg.append("A blocking Windows Sockets 1.1 call is in progress,\n");
        errMsg.append("or the service provider is still processing a callback function.");
    } else if (*errCode == WSAEFAULT) {
        errMsg.append("The buf parameter is not completely contained in a\n");
        errMsg.append("valid part of the user address space.");
    } else if (*errCode == WSAENETRESET) {
        errMsg.append("The connection has been broken due to the keep-alive\n");
        errMsg.append("activity detecting a failure while the operation was in progress.");
    } else if (*errCode == WSAENOBUFS)
        errMsg.append("No buffer space is available.");
    else if (*errCode == WSAENOTCONN)
        errMsg.append("The socket is not connected.");
    else if (*errCode == WSAENOTSOCK)
        errMsg.append("The descriptor is not a socket.");
    else if (*errCode == WSAEOPNOTSUPP) {
        errMsg.append("MSG_OOB was specified, but the socket is not stream-style\n");
        errMsg.append("such as type SOCK_STREAM, out-of-band data is not supported\n");
        errMsg.append("in the communication domain associated with this socket,\n");
        errMsg.append("or the socket is unidirectional and supports only receive operations.");
    } else if (*errCode == WSAESHUTDOWN) {
        errMsg.append("The socket has been shut down; it is not possible to send\n");
        errMsg.append("on a socket after shutdown has been invoked with how set\n");
        errMsg.append("to SD_SEND or SD_BOTH.");
    } else if (*errCode == WSAEWOULDBLOCK)
        errMsg.append("The socket is marked as nonblocking and the requested operation would block.\n");
    else if (*errCode == WSAEMSGSIZE) {
        errMsg.append("The socket is message oriented, and the message is larger\n");
        errMsg.append("than the maximum supported by the underlying transport.");
    } else if (*errCode == WSAEHOSTUNREACH)
        errMsg.append("The remote host cannot be reached from this host at this time.");
    else if (*errCode == WSAEINVAL) {
        errMsg.append("The socket has not been bound with bind, or an unknown flag\n");
        errMsg.append("was specified, or MSG_OOB was specified for a socket with SO_OOBINLINE enabled.");
    } else if (*errCode == WSAECONNABORTED) {
        errMsg.append("The virtual circuit was terminated due to a time-out or \n");
        errMsg.append("other failure. The application should close the socket as it is no longer usable.");
    } else if (*errCode == WSAECONNRESET) {
        errMsg.append("The virtual circuit was reset by the remote side executing a \"hard\" \n");
        errMsg.append("or \"abortive\" close. For UPD sockets, the remote host was unable to\n");
        errMsg.append("deliver a previously sent UDP datagram and responded with a\n");
        errMsg.append("\"Port Unreachable\" ICMP packet. The application should close\n");
        errMsg.append("the socket as it is no longer usable.");
    } else if (*errCode == WSAETIMEDOUT) {
        errMsg.append("The connection has been dropped, because of a network failure\n");
        errMsg.append("or because the system on the other end went down without notice.");
    } else
        errMsg.append("unknown problems!");
}

#endif

void ChSocketTCP::connectToServer(std::string& serverNameOrAddr, hostType hType) {
    /*
       when this method is called, a client socket has been built already,
       so we have the socketId and portNumber ready.

       a ChSocketHostInfo instance is created, no matter how the server's name is
       given (such as www.yuchen.net) or the server's address is given (such
       as 169.56.32.35), we can use this ChSocketHostInfo instance to get the
       IP address of the server
    */

    ChSocketHostInfo serverInfo(serverNameOrAddr, hType);

    // Store the IP address and socket port number
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr(serverInfo.getHostIPAddress());
    serverAddress.sin_port = htons(portNumber);

    // Connect to the given address
    try {
        if (connect(socketId, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
#ifdef WINDOWS_XP
            int errorCode = 0;
            std::string errorMsg = "error calling connect():\n";
            detectErrorConnect(&errorCode, errorMsg);
            ChExceptionSocket* socketConnectException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketConnectException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketConnectException = new ChExceptionSocket(0, "unix: error calling connect()");
            throw unixSocketConnectException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

ChSocketTCP* ChSocketTCP::acceptClient(std::string& clientHost) {
    int newSocket;  // the new socket file descriptor returned by the accept systme call

    // the length of the client's address
#if defined(TARGET_OS_MAC) || defined(UNIX)
    socklen_t clientAddressLen = sizeof(struct sockaddr_in);
#else
    int clientAddressLen = sizeof(struct sockaddr_in);
#endif
    struct sockaddr_in clientAddress;  // Address of the client that sent data

    // Accepts a new client connection and stores its socket file descriptor
    try {
        if ((newSocket = (int)accept(socketId, (struct sockaddr*)&clientAddress, &clientAddressLen)) == -1) {
#ifdef WINDOWS_XP
            int errorCode = 0;
            std::string errorMsg = "error calling accept(): \n";
            detectErrorAccept(&errorCode, errorMsg);
            ChExceptionSocket* socketAcceptException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketAcceptException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketAcceptException = new ChExceptionSocket(0, "unix: error calling accept()");
            throw unixSocketAcceptException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        return NULL;
    }

    // Get the host name given the address
    char* sAddress = inet_ntoa((struct in_addr)clientAddress.sin_addr);
    ChSocketHostInfo clientInfo(std::string(sAddress), ADDRESS);
    char* hostName = clientInfo.getHostName();
    clientHost += std::string(hostName);

    // Create and return the new ChSocketTCP object
    ChSocketTCP* retSocket = new ChSocketTCP();
    retSocket->setSocketId(newSocket);
    return retSocket;
}

void ChSocketTCP::listenToClient(int totalNumPorts) {
    try {
        if (listen(socketId, totalNumPorts) == -1) {
#ifdef WINDOWS_XP
            int errorCode = 0;
            std::string errorMsg = "error calling listen():\n";
            detectErrorListen(&errorCode, errorMsg);
            ChExceptionSocket* socketListenException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketListenException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketListenException = new ChExceptionSocket(0, "unix: error calling listen()");
            throw unixSocketListenException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }
}

/*
int ChSocketTCP::sendMessage(std::string& message)
{
    int numBytes;  // the number of bytes sent


    char msgLength[MSG_HEADER_LEN+1];
    sprintf(msgLength,"%6d",message.size());
    std::string sendMsg = std::string(msgLength);
    sendMsg += message;

    // Sends the message to the connected host
    try
    {
        if (numBytes = send(socketId,sendMsg.c_str(),sendMsg.size(),0) == -1)
        {
            #ifdef WINDOWS_XP
                int errorCode = 0;
                std::string errorMsg = "error calling send():\n";
                detectErrorSend(&errorCode,errorMsg);
                ChExceptionSocket* socketSendException = new ChExceptionSocket(errorCode,errorMsg);
                throw socketSendException;
            #endif

            #ifdef UNIX
                ChExceptionSocket* unixSocketSendException = new ChExceptionSocket(0,"unix: error calling send()");
                throw unixSocketSendException;
            #endif
        }
    }
    catch(ChExceptionSocket* excp)
    {
        excp->response();
        delete excp;
        exit(1);
    }

    return numBytes;
}
*/
int ChSocketTCP::sendMessage(std::string& message) {
    int numBytes;  // the number of bytes sent

    /*
       for each message to be sent, add a header which shows how long this message
       is. This header, regardless how long the real message is, will always be
       of the length MSG_HEADER_LEN.
    */

    char msgLength[MSG_HEADER_LEN + 1];
    std::string fmt = "%" + std::to_string(MSG_HEADER_LEN) + "ld";
    sprintf(msgLength, fmt.c_str(), static_cast<uint64_t>(message.size()));
    std::string sendMsg(msgLength);
    sendMsg += message;

    // Sends the message to the connected host
    try {
        numBytes = send(socketId, sendMsg.c_str(), (int)sendMsg.size(), 0);
        if (numBytes == -1) {
#ifdef WINDOWS_XP
            int errorCode = 0;
            std::string errorMsg = "error calling send():\n";
            detectErrorSend(&errorCode, errorMsg);
            ChExceptionSocket* socketSendException = new ChExceptionSocket(errorCode, errorMsg);
            throw socketSendException;
#endif

#ifdef UNIX
            ChExceptionSocket* unixSocketSendException = new ChExceptionSocket(0, "unix: error calling send()");
            throw unixSocketSendException;
#endif
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    return numBytes;
}

#ifdef WINDOWS_XP
/*
int ChSocketTCP::XPrecieveMessage(std::string& message)
{
    int numBytes = 0;                 // The number of bytes received
    int currentSize = MSG_HEADER_LEN; // The number of bytes wanted to receive
    int offsetSize = 0;               // The number of bytes currently received

    // retrieve the length of the message received

    char msgLength[MSG_HEADER_LEN+1];
    memset(msgLength,0,sizeof(msgLength));

    try
    {
        while ( numBytes < currentSize )
        {
            numBytes = recv(socketId,msgLength+offsetSize,currentSize,MSG_PEEK);
            if (numBytes == -1)
            {
                int errorCode = 0;
                std::string errorMsg = "error calling recv():\n";
                detectErrorRecv(&errorCode,errorMsg);
                ChExceptionSocket* socketRecvException = new ChExceptionSocket(errorCode,errorMsg);
                throw socketRecvException;
            }
            else if ( numBytes < currentSize )
            {
                offsetSize += numBytes;
                currentSize -= numBytes;
            }
        }

    }
    catch(ChExceptionSocket* excp)
    {
        excp->response();
        delete excp;
        exit(1);
    }

    // recieve the real message
    currentSize = atoi(msgLength);
    offsetSize = 0;

    cout   << "[RECV:message length] " << msgLength << std::endl;
    winLog << "[RECV:message length] " << msgLength << std::endl;

    try
    {
        while ( numBytes < currentSize )
        {
            numBytes = recv(socketId,(char*)(message.c_str())+offsetSize,currentSize,0);
            if (numBytes == -1)
            {
                int errorCode = 0;
                std::string errorMsg = "error calling recv():\n";
                detectErrorRecv(&errorCode,errorMsg);
                ChExceptionSocket* socketRecvException = new ChExceptionSocket(errorCode,errorMsg);
                throw socketRecvException;
            }
            else if ( numBytes < currentSize )
            {
                offsetSize += numBytes;
                currentSize -= numBytes;
            }
        }

    }
    catch(ChExceptionSocket* excp)
    {
        excp->response();
        delete excp;
        exit(1);
    }

    cout   << "[RECV:message] " << message << std::endl;
    winLog << "[RECV:message] " << message << std::endl;

    return atoi(msgLength);
}
*/

int ChSocketTCP::XPrecieveMessage(std::string& message) {
    int received = 0;            // The number of bytes received
    int msgSize = MAX_RECV_LEN;  // The number of bytes wanted to receive
    bool headerFinished = false;

    char charMsg[MAX_RECV_LEN + 1];
    char msgLength[MSG_HEADER_LEN + 1];
    memset(charMsg, 0, sizeof(charMsg));
    memset(msgLength, 0, sizeof(msgLength));

    try {
        while (received < msgSize) {
            int numBytes = recv(socketId, charMsg + received, 1, 0);
            if (numBytes == -1) {
                int errorCode = 0;
                std::string errorMsg = "error calling recv():\n";
                detectErrorRecv(&errorCode, errorMsg);
                ChExceptionSocket* socketRecvException = new ChExceptionSocket(errorCode, errorMsg);
                throw socketRecvException;
            }

            if (!headerFinished) {
                msgLength[received] = *(charMsg + received);
                received++;

                if (received == MSG_HEADER_LEN) {
                    headerFinished = true;
                    received = 0;
                    memset(charMsg, 0, sizeof(charMsg));
                    msgSize = atoi(msgLength);
                }
            } else
                received++;
        }
    } catch (ChExceptionSocket* excp) {
        if (excp->getErrCode() == WSAECONNRESET) {
            std::cout << "!! your party has shut down the connection... \n";
            // winLog << "!! your party has shut down the connection... \n";
            return -99;
        }
        excp->response();
        delete excp;
        exit(1);
    }

    message.append(std::string(charMsg));
    return msgSize;
}

#endif

int ChSocketTCP::receiveMessage(std::string& message) {
#ifdef WINDOWS_XP
    return XPrecieveMessage(message);
#endif

    int64_t numBytes = 0;       // Number of received bytes on the last recv
    int64_t expectedBytes = 0;  // Number of total bytes expected
    int64_t receivedBytes = 0;  // Accumulated number of bytes received thus far

    try {
        // Receive the header
        char header[MSG_HEADER_LEN + 1];
        numBytes = recv(socketId, header, MSG_HEADER_LEN, 0);
        if (numBytes == -1) {
            ChExceptionSocket* unixSocketRecvException = new ChExceptionSocket(0, "unix: error calling recv()");
            throw unixSocketRecvException;
        }
        expectedBytes = atoll(header);
        message.resize(expectedBytes);

        while (receivedBytes < expectedBytes) {
            numBytes = recv(socketId, (char*)(message.c_str()) + receivedBytes, expectedBytes, 0);
            if (numBytes == -1) {
                ChExceptionSocket* unixSocketRecvException = new ChExceptionSocket(0, "unix: error calling recv()");
                throw unixSocketRecvException;
            }

            receivedBytes += numBytes;
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        delete excp;
        exit(1);
    }

    return numBytes;
}

int ChSocketTCP::SendBuffer(std::vector<char>& source_buf) {
    int nbytes = (int)source_buf.size();
    const char* data;
    if (nbytes)
        data = (char*)&(source_buf[0]);  // stl vectors are assured to be sequential
    else
        data = "";  // stub, in case null length messages, stl vector has no [0] element address

    // Sends the message to the connected host
    int sentBytes = send(socketId, data, nbytes, 0);
    if (sentBytes == -1) {
#ifdef WINDOWS_XP
        int errorCode = 0;
        std::string errorMsg = "error calling send():\n";
        detectErrorRecv(&errorCode, errorMsg);
        throw ChExceptionSocket(errorCode, errorMsg);
#endif

#ifdef UNIX
        throw ChExceptionSocket(0, "unix: error calling send()");
#endif
    }

    return sentBytes;
}

int ChSocketTCP::ReceiveBuffer(std::vector<char>& dest_buf, int bsize) {
    int nbytes = bsize;

    dest_buf.resize(nbytes);

    void* data;
    if (nbytes)
        data = (char*)&(dest_buf[0]);  // stl vectors are assured to be sequential. // should not access directly
                                       // std::vector data, but this is efficient!
    else
        data = (char*)"";  // stub, in case null length messages, stl vector has no [0] element address

    int receivedBytes = recv(socketId, (char*)data, nbytes, 0);
    if (receivedBytes == -1) {
#ifdef WINDOWS_XP
        int errorCode = 0;
        std::string errorMsg = "error calling recv():\n";
        detectErrorRecv(&errorCode, errorMsg);
        throw ChExceptionSocket(errorCode, errorMsg);
#endif

#ifdef UNIX
        throw ChExceptionSocket(0, "Error calling recv() in buffer receive:");
#endif
    }

    return receivedBytes;
}

// -----------------
// ChSocketFramework
// -----------------

ChSocketFramework::ChSocketFramework() {
#ifdef WINDOWS_XP

    // Initialize the winsock library
    WSADATA wsaData;

    if (WSAStartup(0x101, &wsaData)) {
        throw ChExceptionSocket(0, "Error: calling WSAStartup()");
    }

#endif
}

ChSocketFramework::~ChSocketFramework() {
#ifdef WINDOWS_XP
    WSACleanup();
#endif
}

// ----------------
// ChSocketHostInfo
// ----------------

ChSocketHostInfo::ChSocketHostInfo() {
#ifdef UNIX
    openHostDb();
// winLog<<"UNIX version ChSocketHostInfo() is called...\n";
#endif

#ifdef WINDOWS_XP

    char sName[HOST_NAME_LENGTH + 1];
    memset(sName, 0, sizeof(sName));
    gethostname(sName, HOST_NAME_LENGTH);

    try {
        hostPtr = gethostbyname(sName);
        if (hostPtr == NULL) {
            int errorCode;
            std::string errorMsg = "";
            detectErrorGethostbyname(&errorCode, errorMsg);
            ChExceptionSocket* gethostbynameException = new ChExceptionSocket(errorCode, errorMsg);
            throw gethostbynameException;
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        exit(1);
    }

#endif
}

ChSocketHostInfo::ChSocketHostInfo(const std::string& hostName, hostType type) {
#ifdef UNIX
    searchHostDB = 0;
#endif

    try {
        if (type == NAME) {
            // Retrieve host by name
            hostPtr = gethostbyname(hostName.c_str());

            if (hostPtr == NULL) {
#ifdef WINDOWS_XP
                int errorCode;
                std::string errorMsg = "";
                detectErrorGethostbyname(&errorCode, errorMsg);
                ChExceptionSocket* gethostbynameException = new ChExceptionSocket(errorCode, errorMsg);
                throw gethostbynameException;
#endif

#ifdef UNIX
                ChExceptionSocket* gethostbynameException =
                    new ChExceptionSocket(0, "unix: error getting host by name");
                throw gethostbynameException;
#endif
            }
        } else if (type == ADDRESS) {
            // Retrieve host by address
            unsigned long netAddr = inet_addr(hostName.c_str());
            if (netAddr == -1) {
                ChExceptionSocket* inet_addrException = new ChExceptionSocket(0, "Error calling inet_addr()");
                throw inet_addrException;
            }

            hostPtr = gethostbyaddr((char*)&netAddr, sizeof(netAddr), AF_INET);
            if (hostPtr == NULL) {
#ifdef WINDOWS_XP
                int errorCode;
                std::string errorMsg = "";
                detectErrorGethostbyaddr(&errorCode, errorMsg);
                ChExceptionSocket* gethostbyaddrException = new ChExceptionSocket(errorCode, errorMsg);
                throw gethostbyaddrException;
#endif

#ifdef UNIX
                ChExceptionSocket* gethostbynameException =
                    new ChExceptionSocket(0, "unix: error getting host by name");
                throw gethostbynameException;
#endif
            }
        } else {
            ChExceptionSocket* unknownTypeException =
                new ChExceptionSocket(0, "unknown host type: host name/address has to be given ");
            throw unknownTypeException;
        }
    } catch (ChExceptionSocket* excp) {
        excp->response();
        exit(1);
    }
}

char* ChSocketHostInfo::getHostIPAddress() {
    struct in_addr* addr_ptr;
    // the first address in the list of host addresses
    addr_ptr = (struct in_addr*)*hostPtr->h_addr_list;
    // changed the address format to the Internet address in standard dot notation
    return inet_ntoa(*addr_ptr);
}

#ifdef WINDOWS_XP
void ChSocketHostInfo::detectErrorGethostbyname(int* errCode, std::string& errorMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errorMsg.append("need to call WSAStartup to initialize socket system on Window system.");
    else if (*errCode == WSAENETDOWN)
        errorMsg.append("The network subsystem has failed.");
    else if (*errCode == WSAHOST_NOT_FOUND)
        errorMsg.append("Authoritative Answer Host not found.");
    else if (*errCode == WSATRY_AGAIN)
        errorMsg.append("Non-Authoritative Host not found, or server failure.");
    else if (*errCode == WSANO_RECOVERY)
        errorMsg.append("Nonrecoverable error occurred.");
    else if (*errCode == WSANO_DATA)
        errorMsg.append("Valid name, no data record of requested type.");
    else if (*errCode == WSAEINPROGRESS)
        errorMsg.append(
            "A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a "
            "callback function.");
    else if (*errCode == WSAEFAULT)
        errorMsg.append("The name parameter is not a valid part of the user address space.");
    else if (*errCode == WSAEINTR)
        errorMsg.append("A blocking Windows Socket 1.1 call was canceled through WSACancelBlockingCall.");
}
#endif

#ifdef WINDOWS_XP
void ChSocketHostInfo::detectErrorGethostbyaddr(int* errCode, std::string& errorMsg) {
    *errCode = WSAGetLastError();

    if (*errCode == WSANOTINITIALISED)
        errorMsg.append("A successful WSAStartup must occur before using this function.");
    if (*errCode == WSAENETDOWN)
        errorMsg.append("The network subsystem has failed.");
    if (*errCode == WSAHOST_NOT_FOUND)
        errorMsg.append("Authoritative Answer Host not found.");
    if (*errCode == WSATRY_AGAIN)
        errorMsg.append("Non-Authoritative Host not found, or server failed.");
    if (*errCode == WSANO_RECOVERY)
        errorMsg.append("Nonrecoverable error occurred.");
    if (*errCode == WSANO_DATA)
        errorMsg.append("Valid name, no data record of requested type.");
    if (*errCode == WSAEINPROGRESS)
        errorMsg.append(
            "A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a "
            "callback function.");
    if (*errCode == WSAEAFNOSUPPORT)
        errorMsg.append("The type specified is not supported by the Windows Sockets implementation.");
    if (*errCode == WSAEFAULT)
        errorMsg.append(
            "The addr parameter is not a valid part of the user address space, or the len parameter is too small.");
    if (*errCode == WSAEINTR)
        errorMsg.append("A blocking Windows Socket 1.1 call was canceled through WSACancelBlockingCall.");
}
#endif

#ifdef UNIX
char ChSocketHostInfo::getNextHost() {
    // winLog<<"UNIX getNextHost() is called...\n";
    // Get the next host from the database
    if (searchHostDB == 1) {
        if ((hostPtr = gethostent()) == NULL)
            return 0;
        else
            return 1;
    }
    return 0;
}
#endif

}  // namespace utils
}  // end namespace chrono
