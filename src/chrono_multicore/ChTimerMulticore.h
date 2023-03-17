// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Timer class that uses a map to query and add timers
//
// =============================================================================

#pragma once

#include <map>
#include <iostream>
#include <string>

#include "chrono/core/ChTimer.h"

#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

/// @addtogroup multicore_module
/// @{

/// Wrapper class for a timer object.
struct TimerData {
    TimerData() : runs(0) {}

    void Reset() {
        runs = 0;
        timer.reset();
    }

    double GetSec() { return timer(); }
    double GetMsec() { return timer() * 1000.0; }

    void start() {
        runs++;
        timer.start();
    }
    void stop() { timer.stop(); }

    ChTimer timer;
    int runs;
};

/// Utility class for managing a collection of timer objects.
class CH_MULTICORE_API ChTimerMulticore {
  public:
    ChTimerMulticore() : total_timers(0) {}
    ~ChTimerMulticore() {}

    void AddTimer(const std::string& name) {
        TimerData temp;
        timer_list[name] = temp;
        total_timers++;
    }

    void Reset() {
        for (auto& timer : timer_list) {
            timer.second.Reset();
        }
    }

    void start(const std::string& name) { timer_list.at(name).start(); }

    void stop(const std::string& name) { timer_list.at(name).stop(); }

    // Returns the time associated with a specific timer
    double GetTime(const std::string& name) const {
        if (timer_list.count(name) == 0) {
            return 0;
        }
        return timer_list.at(name).timer();
    }

    // Returns the number of times a specific timer was called
    int GetRuns(const std::string& name) const {
        if (timer_list.count(name) == 0) {
            return 0;
        }
        return timer_list.at(name).runs;
    }

    void PrintReport() const {
        std::cout << "Timer Report:" << std::endl;
        std::cout << "------------" << std::endl;
        for (auto& timer : timer_list) {
            std::cout << "Name:\t" << timer.first << "\t" << timer.second.timer() << "\n";
        }
        std::cout << "------------" << std::endl;
    }

    int total_timers;
    std::map<std::string, TimerData> timer_list;
    std::map<std::string, TimerData>::iterator it;
};

/// @} multicore_module

} // end namespace chrono
