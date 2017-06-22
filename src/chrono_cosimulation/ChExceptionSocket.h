// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHEXCEPTIONSOCKET_H
#define CHEXCEPTIONSOCKET_H

#include <fstream>
#include <iostream>
#include <string>
#include <ctime>

#include "chrono/core/ChException.h"
#include "chrono/core/ChLog.h"
#include "chrono_cosimulation/ChApiCosimulation.h"

namespace chrono {
namespace cosimul {

/// Class for exceptions that are thrown by TCP socket connections,
/// used for example when connecting with other sw for cosimulation.

class ChExceptionSocket : public ChException {
  public:
    ChExceptionSocket(int code, const std::string& what) : ChException(what), errorCode(code){};

    // get socket error code in thrown exception
    int getErrCode() { return errorCode; }

    // std::string& getErrMsg() { return std::string(this->what()); }

    void response() {
        GetLog() << "TCP socket error: \n";
        GetLog() << "		==> error code: " << errorCode << "\n";
        GetLog() << "		==> error message: " << this->what() << "\n";
    }

  private:
    int errorCode;
};

}  // end namespace cosimul
}  // end namespace chrono

#endif