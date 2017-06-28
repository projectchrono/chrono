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

#include "chrono_cosimulation/ChHostInfo.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

using namespace std;

namespace chrono {
namespace cosimul {

ChHostInfo::ChHostInfo() {
#ifdef UNIX
    openHostDb();
// winLog<<"UNIX version ChHostInfo() is called...\n";
#endif

#ifdef WINDOWS_XP

    char sName[HOST_NAME_LENGTH + 1];
    memset(sName, 0, sizeof(sName));
    gethostname(sName, HOST_NAME_LENGTH);

    try {
        hostPtr = gethostbyname(sName);
        if (hostPtr == NULL) {
            int errorCode;
            string errorMsg = "";
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

ChHostInfo::ChHostInfo(const string& hostName, hostType type) {
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
                string errorMsg = "";
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
                string errorMsg = "";
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

char* ChHostInfo::getHostIPAddress() {
    struct in_addr* addr_ptr;
    // the first address in the list of host addresses
    addr_ptr = (struct in_addr*)*hostPtr->h_addr_list;
    // changed the address format to the Internet address in standard dot notation
    return inet_ntoa(*addr_ptr);
}

#ifdef WINDOWS_XP
void ChHostInfo::detectErrorGethostbyname(int* errCode, string& errorMsg) {
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
void ChHostInfo::detectErrorGethostbyaddr(int* errCode, string& errorMsg) {
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
char ChHostInfo::getNextHost() {
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

}  // end namespace cosimul
}  // end namespace chrono
