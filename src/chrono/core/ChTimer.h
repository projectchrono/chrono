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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_TIMER_H
#define CH_TIMER_H

#include <ratio>
#include <chrono>

namespace chrono {

/// Chrono wrappers for high-resolution timers.
class ChTimer {
  public:
    ChTimer() {}

    /// Start the timer.
    void start() { m_start = std::chrono::high_resolution_clock::now(); }

    /// Stop the timer.
    void stop() {
        m_end = std::chrono::high_resolution_clock::now();
        m_total += m_end - m_start;
    }

    /// Reset the total accumulated time (when repeating multiple start/stop).
    void reset() { m_total = std::chrono::duration<double>(0); }

    /// Return the time in milliseconds.
    /// Use start()..stop() before calling this function.
    unsigned long long GetTimeMilliseconds() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(m_total).count();
    }

    /// Return the time in microseconds.
    /// Use start()..stop() before calling this.
    unsigned long long GetTimeMicroseconds() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(m_total).count();
    }

    /// Return the time in m,illiseconds since start().
    /// This function does not require a call to stop().
    unsigned long long GetTimeMillisecondsIntermediate() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start).count();
    }

    /// Return the time in microseconds since start().
    /// This function does not require a call to stop().
    unsigned long long GetTimeMicrosecondsIntermediate() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now - m_start).count();
    }

    /// Return the time in seconds.
    /// Use start()..stop() before calling this.
    double GetTimeSeconds() const { return m_total.count(); }

    /// Return the time in seconds since start().
    /// This function does not require a call to stop().
    double GetTimeSecondsIntermediate() const {
        std::chrono::duration<double> int_time = std::chrono::high_resolution_clock::now() - m_start;
        return int_time.count();
    }

    /// Get the last timer value, in seconds.
    double operator()() const { return GetTimeSeconds(); }

  private:
    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;
    std::chrono::duration<double> m_total;
};

}  // end namespace chrono

#endif
