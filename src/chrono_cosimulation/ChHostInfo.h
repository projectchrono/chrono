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

#ifndef CHHOSTINFO_H
#define CHHOSTINFO_H

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

#include <string>

#include "chrono_cosimulation/ChApiCosimulation.h"

// This version is for both Windows and UNIX, the following statements
// are used to set the flags WINDOWS_XP or UNIX that in these few files of 'socket'
// code are used for conditional compilation
#if (defined _WIN32)
#define WINDOWS_XP
#endif
#if (defined(__linux__) || defined(__APPLE__))
#define UNIX
#endif

#ifdef UNIX
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#else
#include <winsock2.h>
#endif
#include <cstdio>

namespace chrono {
namespace cosimul {

enum hostType { NAME, ADDRESS };
const int HOST_NAME_LENGTH = 64;

/// Class for storing information about a TCP host
/// in socket communication, ex with an IP address

class ChApiCosimulation ChHostInfo {
  private:
#ifdef UNIX
    char searchHostDB;  // search the host database flag
#endif

    struct hostent* hostPtr;  // Entry within the host address database

  public:
    // Default constructor
    ChHostInfo();

    // Retrieves the host entry based on the host name or address
    ChHostInfo(const std::string& hostName, hostType type);

    // Destructor.  Closes the host entry database.
    ~ChHostInfo() {
#ifdef UNIX
        endhostent();
#endif
    }

#ifdef UNIX

    // Retrieves the next host entry in the database
    char getNextHost();

    // Opens the host entry database
    void openHostDb() {
        endhostent();
        searchHostDB = 1;
        sethostent(1);
    }

#endif

    // Retrieves the hosts IP address in dot x.y.z.w notation
    char* getHostIPAddress();

    // Retrieves the hosts name
    char* getHostName() { return hostPtr->h_name; }

  private:
#ifdef WINDOWS_XP
    void detectErrorGethostbyname(int*, std::string&);
    void detectErrorGethostbyaddr(int*, std::string&);
#endif
};

}  // end namespace cosimul
}  // end namespace chrono

#endif
