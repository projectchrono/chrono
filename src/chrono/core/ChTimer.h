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

#ifndef CHTIMER_H
#define CHTIMER_H

#if ((defined _WIN32) && !(defined(__MINGW32__) || defined(__CYGWIN__)))
    #include <ctime>
    #ifndef NOMINMAX
    #define NOMINMAX
    #endif
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    #include <ctime>
    #undef WIN32_LEAN_AND_MEAN
    #undef NOMINMAX
#else
    #include <ratio>
    #include <chrono>
#endif


namespace chrono {


/// Class for high-resolution timing.
/// Use the start() ... stop() commands, then request the elapsed time between 
/// them using GetTimeMilliseconds() or GetTimeSeconds() etc.
/// Note: the elapsed time is not computed at GetTime... functions, but only at stop().
/// If you use multiple  start() ... stop()  pairs, each time the duration is accumulated,
/// until a reset() is done.
/// Note: since the Windows std::chrono has a limited resolution (1ms-15ms depending on the
/// system) here we temporary provide a conditional compilation of another timer that does
/// not use std::chrono for windows, until Microsoft will provide a better std::chrono.

#if ((defined _WIN32) && !(defined(__MINGW32__) || defined(__CYGWIN__)))

template <typename seconds_type = double>
class ChTimer {


  private:
    LARGE_INTEGER m_start;
    LARGE_INTEGER m_end;
    LARGE_INTEGER m_freq;
    seconds_type total;

  public:
    ChTimer() : total(0) {
        QueryPerformanceFrequency(&m_freq);
        m_start.QuadPart = 0;
        m_end.QuadPart = 0;
    }

  public:
    /// Start the timer
    void start() {
        QueryPerformanceCounter(&m_start);
    }

    /// Stops the timer
    void stop() { 
        QueryPerformanceCounter(&m_end); 
        seconds_type end = static_cast<seconds_type>(m_end.QuadPart);
        seconds_type start = static_cast<seconds_type>(m_start.QuadPart);
        seconds_type freq = static_cast<seconds_type>(m_freq.QuadPart);
        total += (end - start) / freq;
    }

    /// Reset the total accumulated time (when repeating multiple start() stop() start() stop() )
    void reset() { total = 0; }

    /// Returns the time in [ms]. 
    /// Use start()..stop() before calling this.
	unsigned long long GetTimeMilliseconds() const {
        return (long long)(total*1000.);
    }

    /// Returns the time in [ms] since start(). It does not require stop(). 
	unsigned long long GetTimeMillisecondsIntermediate() const {
        return (long long)(GetTimeSecondsIntermediate()*1000.0);
    }

	/// Returns the time in [us]. 
    /// Use start()..stop() before calling this.
	unsigned long long GetTimeMicroseconds() const {
        return (long long)(total*1000000.0);
    }

    /// Returns the time in [us] since start(). It does not require stop(). 
	unsigned long long GetTimeMicrosecondsIntermediate() const {
        return (long long)(GetTimeSecondsIntermediate()*1000000.0);
    }

    /// Returns the time in [s], with seconds_type precision
    /// Use start()..stop() before calling this.
	seconds_type GetTimeSeconds() const {
        return total;
    }

    /// Returns the time in [s] since start(). It does not require stop(). 
	seconds_type GetTimeSecondsIntermediate() const {
        LARGE_INTEGER m_inter;
        QueryPerformanceCounter(&m_inter); 
        seconds_type end = static_cast<seconds_type>(m_inter.QuadPart);
        seconds_type start = static_cast<seconds_type>(m_start.QuadPart);
        seconds_type freq = static_cast<seconds_type>(m_freq.QuadPart);
        seconds_type intermediate= (end - start) / freq;
        return intermediate;
    }

    /// Get the timer value, with the () operator.
    seconds_type operator()() const {
        return total;
    }
};

#else 

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
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - m_start).count();
    }

	/// Returns the time in [us]. 
    /// Use start()..stop() before calling this.
	unsigned long long GetTimeMicroseconds() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(m_total).count();
    }

    /// Returns the time in [us] since start(). It does not require stop(). 
	unsigned long long GetTimeMicrosecondsIntermediate() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_start).count();
    }

    /// Returns the time in [s], with real_type precision
    /// Use start()..stop() before calling this.
	seconds_type GetTimeSeconds() const {
        return m_total.count();
    }

    /// Returns the time in [s] since start(). It does not require stop(). 
	seconds_type GetTimeSecondsIntermediate() const {
        std::chrono::duration<seconds_type> int_time = std::chrono::high_resolution_clock::now() - m_start;
        return int_time.count();
    }

    /// Get the last timer value, in seconds, with the () operator.
    seconds_type operator()() const {
        return GetTimeSeconds();
    }

};

#endif

}  // end namespace chrono

#endif
