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
#ifndef CHLOG_H
#define CHLOG_H

#include <cassert>

#include "chrono/core/ChStream.h"
#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Base class for output of errors, messages, warnings, etc.
/// (logging class). Must be specialized for specific output,
/// for example output to console, to windows, etc. by inheriting
/// custom classes. See below, an example for logging to std::cout

class ChApi ChLog : public ChStreamOutAscii {
  public:
    /// There are different levels of log messages. This is
    /// indifferent for the base class, but more sophisticated
    /// specializations of the ChLog class may handle message output
    /// in different ways (for example a ChLogForGUIapplication may
    /// print logs in STATUS level only to the bottom of the window, etc.)
    enum eChLogLevel { CHERROR = 0, CHWARNING, CHMESSAGE, CHSTATUS, CHQUIET };

  protected:
    eChLogLevel current_level;
    eChLogLevel default_level;

    /// Creates the ChLog, and sets the level at MESSAGE
    ChLog();

  public:
    /// Forces output of message buffers, if any.
    /// Also, should restore the default eChLogLevel
    /// This base class does pretty nothing, but inherited classes
    /// should override this method with more specific implementations.
    virtual void Flush() { RestoreDefaultLevel(); };

    /// Sets the default level, to be used from now on.
    void SetDefaultLevel(eChLogLevel mlev) { default_level = mlev; };

    /// Sets the current level, to be used until new flushing.
    void SetCurrentLevel(eChLogLevel mlev) { current_level = mlev; };

    /// Gets the current level
    eChLogLevel GetCurrentLevel() { return current_level; };

    /// Restore the default level.
    void RestoreDefaultLevel() { current_level = default_level; };

    /// Using the - operator is easy to set the status of the
    /// log, so in you code you can write, for example:
    ///   GetLog() - ChLog::CHERROR << "a big error in " << mynumber << " items \n" ;
    ChLog& operator-(eChLogLevel mnewlev);

  private:
};

////////////////////////////////////////////////////////
//  LOG TO CONSOLE
//
/// Specialized class for logging errors in std::cout.
/// Messages, warnings, etc. go always into the standard console.

class ChApi ChLogConsole : public ChLog, public ChStreamOstreamWrapper {
  public:
    /// Create default logger: this will use the std::cout
    ChLogConsole() : ChStreamOstreamWrapper(&std::cout) {}

    virtual ~ChLogConsole(){};

    /// Redirect output stream to file wrapper.
    virtual void Output(const char* data, size_t n) {
        if (current_level != CHQUIET)
            ChStreamOstreamWrapper::Write(data, n);
    }

  private:
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/// Global function to get the current ChLog object
ChApi ChLog& GetLog();

/// Global function to set another ChLog object as current 'global' logging system.
/// For example, a plugin developer may like to see logs on a GUI dialog: if so, he
/// can inherit a specialized ChLog class (overriding the Output() function) and
/// use the SetLog() function to have it available all times.
/// Doing so, in your code you can write, for example:
///
///   chrono::GetLog() << "message";
///

ChApi void SetLog(ChLog& new_logobject);

/// Global function to set the default ChLogConsole output to std::output.
ChApi void SetLogDefault();

}  // end namespace chrono

#endif
