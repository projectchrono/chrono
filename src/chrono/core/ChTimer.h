//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTIMER_H
#define CHTIMER_H

//////////////////////////////////////////////////
//
//   ChTimer.h
//
//   Class for high-resolution timer.
//   This class can be used to get the execution time of
//   fast algorithms, for example for profiling.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <ratio>
#include <chrono>

namespace chrono {


/// Class for high-resolution timing.
/// Use the start() ... stop() commands, then request the elapsed time between 
/// them using GetTimeMilliseconds() or GetTimeSeconds() etc.
/// Note: the elapsed time is not computed at GetTime... functions, but only at stop().
/// If you use multiple  start() ... stop()  pairs, each time the duration is accumulated,
/// until a reset() is done.

template <class seconds_type = double>
class ChTimer {

  private:
    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;
    std::chrono::duration<seconds_type> m_total;

  public:
    ChTimer() {}

    /// Start the timer
    void start() {
        m_start = std::chrono::high_resolution_clock::now();
    }

    /// Stops the timer
    void stop() { 
        m_end = std::chrono::high_resolution_clock::now();
        m_total += m_end-m_start;
    }

    /// Reset the total accumulated time (when repeating multiple start() stop() start() stop() )
    void reset() { m_total = std::chrono::duration<double>(0); }

    /// Returns the time in [ms]. 
    /// Use start()..stop() before calling this.
	unsigned long long GetTimeMilliseconds() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(m_total).count();
    }

    /// Returns the time in [ms] since start(). It does not require stop(). 
	unsigned long long GetTimeMillisecondsIntermediate() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(m_start - std::chrono::high_resolution_clock::now()).count();
    }

	/// Returns the time in [us]. 
    /// Use start()..stop() before calling this.
	unsigned long long GetTimeMicroseconds() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(m_total).count();
    }

    /// Returns the time in [us] since start(). It does not require stop(). 
	unsigned long long GetTimeMicrosecondsIntermediate() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(m_start - std::chrono::high_resolution_clock::now()).count();
    }

    /// Returns the time in [s], with real_type precision
    /// Use start()..stop() before calling this.
	seconds_type GetTimeSeconds() const {
        return m_total.count();
    }

    /// Returns the time in [s] since start(). It does not require stop(). 
	seconds_type GetTimeSecondsIntermediate() const {
        std::chrono::duration<seconds_type> int_time = m_start - std::chrono::high_resolution_clock::now();
        return int_time.count();
    }

    /// Get the last timer value, in seconds, with the () operator.
    seconds_type operator()() const {
        return GetTimeSeconds();
    }

};


}  // END_OF_NAMESPACE____

#endif  // END of ChTimer.h
