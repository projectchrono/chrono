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

#if (((defined WIN32) || (defined WIN64)) && !(defined(__MINGW32__) || defined(__CYGWIN__)))
#include <time.h>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <time.h>
#undef WIN32_LEAN_AND_MEAN
#undef NOMINMAX
#else
#include <sys/time.h>
#endif

namespace chrono {

/// Class for high-resolution timer.
/// This class can be used to get the execution time of
/// fast algorithms, for example for profiling.
///
/// Based on http://www-106.ibm.com/developerworks/library/l-rt1/
/// and on the timer of the OpenTissue project ( (C) J.Sporring, K.Erleben)
///
/// How to use it:
///
///  ChTimer<double> timer;
///
///  timer.start();
///  ...
///  timer.stop();
///  GetLog() << "It took " << timer() << " seconds to perform" << std::endl;
///

template <typename real_type>
class ChTimer {
#if (((defined WIN32) || (defined WIN64)) && !(defined(__MINGW32__) || defined(__CYGWIN__)))

  private:
    LARGE_INTEGER m_start;
    LARGE_INTEGER m_end;
    LARGE_INTEGER m_freq;
    bool m_first;
    real_type total;

  public:
    ChTimer() : m_first(true), total(0) {}

  public:
    /// Start the timer
    void start() {
        if (m_first) {
            QueryPerformanceFrequency(&m_freq);
            m_first = false;
        }
        QueryPerformanceCounter(&m_start);
    }

    /// Stops the timer
    void stop() { 
        QueryPerformanceCounter(&m_end); 
        real_type end = static_cast<real_type>(m_end.QuadPart);
        real_type start = static_cast<real_type>(m_start.QuadPart);
        real_type freq = static_cast<real_type>(m_freq.QuadPart);
        total += (end - start) / freq;
    }

    /// Reset the total accumulated time (when repeating multiple start() stop() start() stop() )
    void reset() { total = 0; }

    /// Get the timer value, with the () operator.
    real_type operator()() const {
        return total;
    }

#else

  private:
    struct timeval m_start;
    struct timeval m_end;
    real_type total;

#ifdef SGI
    time_t m_tz;
#else
    struct timezone m_tz;
#endif

  public:
    ChTimer() : total(0) {}

    /// Start the timer
    void start() { gettimeofday(&m_start, &m_tz); }

    /// Stops the timer
    void stop() { 
        gettimeofday(&m_end, &m_tz); 
        real_type t1 = static_cast<real_type>(m_start.tv_sec) + static_cast<real_type>(m_start.tv_usec) / (1000 * 1000);
        real_type t2 = static_cast<real_type>(m_end.tv_sec) + static_cast<real_type>(m_end.tv_usec) / (1000 * 1000);
        total += t2 - t1;
    }

    /// Reset the total accumulated time (when repeating multiple start() stop() start() stop() )
    void reset() { total = 0; }

    /// Get the timer value, with the () operator.
    real_type operator()() const {
        return total;
    }

#endif
};

}  // END_OF_NAMESPACE____

#endif  // END of ChTimer.h
