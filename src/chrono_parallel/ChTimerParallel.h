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
// Description: Parallel timer class that uses a map to query and add timers
//
// =============================================================================

#pragma once

#include <map>
#include <iostream>
#include <string>

#include "chrono/core/ChTimer.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

/// @addtogroup parallel_module
/// @{

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

    ChTimer<double> timer;
    int runs;
};

class CH_PARALLEL_API ChTimerParallel {
  public:
    ChTimerParallel() : total_timers(0), total_time(0) {}
    ~ChTimerParallel() {}

    void AddTimer(std::string name) {
        TimerData temp;
        timer_list[name] = temp;
        total_timers++;
    }

    void Reset() {
        for (std::map<std::string, TimerData>::iterator it = timer_list.begin(); it != timer_list.end(); it++) {
            it->second.Reset();
        }
    }

    void start(std::string name) { timer_list[name].start(); }

    void stop(std::string name) { timer_list[name].stop(); }

    // Returns the time associated with a specific timer
    double GetTime(std::string name) {
        if (timer_list.count(name) == 0) {
            return 0;
        }
        return timer_list[name].timer();
    }

    // Returns the number of times a specific timer was called
    int GetRuns(std::string name) {
        if (timer_list.count(name) == 0) {
            return 0;
        }
        return timer_list[name].runs;
    }
    void PrintReport() {
        total_time = 0;
        std::cout << "Timer Report:" << std::endl;
        std::cout << "------------" << std::endl;
        for (std::map<std::string, TimerData>::iterator it = timer_list.begin(); it != timer_list.end(); it++) {
            std::cout << "Name:\t" << it->first << "\t" << it->second.timer();
            std::cout << std::endl;
            total_time += it->second.timer();
        }
        std::cout << "------------" << std::endl;
    }

    double total_time;
    int total_timers;
    std::map<std::string, TimerData> timer_list;
    std::map<std::string, TimerData>::iterator it;
};

/// @} parallel_module

} // end namespace chrono
