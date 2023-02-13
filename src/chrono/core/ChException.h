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

#ifndef CHEXCEPTION_H
#define CHEXCEPTION_H

#include <string>

namespace chrono {

/// Class for exceptions for throw() catch() mechanism.
/// Each class can contain a message in form of text.
class ChException : public std::exception {
  public:
    /// Constructor for a basic exception: sets the exception message to the specified string.
    ChException(const std::string& swhat) : m_swhat(swhat) {}

    /// Copy constructor.
    ChException(const ChException& right) : m_swhat(right.what()) {}

    /// Assignment operator.
    ChException& operator=(const ChException& right) {
        m_swhat = right.what();
        return *this;
    }

    virtual ~ChException() noexcept {}

    virtual const char* what() const throw() { return m_swhat.c_str(); }

  protected:
    std::string m_swhat;
};

}  // end namespace chrono

#endif
