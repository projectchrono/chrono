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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Parallel timer class that uses a map to query and add timers
// =============================================================================

#ifndef CHTIMERPARALLEL_H
#define CHTIMERPARALLEL_H

#include "core/ChTimer.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

namespace chrono {

struct TimerData {
   double flop;
   double memory_ops;

   ChTimer<double> timer;
   double time;
   double flop_rate;
   double bandwidth;
   int runs;
   bool compute_stats;
   TimerData() {
      flop = memory_ops = 0;
      time = flop_rate = bandwidth = 0;
      runs = 0;
      compute_stats = false;
   }

   void Reset() {
      flop = memory_ops = 0;
      time = flop_rate = bandwidth = 0;
      runs = 0;
   }

   double GetSec() {
      return timer();
   }

   double GetMsec() {

      return timer() / 1000.0;
   }
   void Compute() {
      if (compute_stats) {
         flop_rate = flop * runs / time / 1.0e9;
         bandwidth = (memory_ops * runs / time) / 1024.0 / 1024.0 / 1024.0;
      }
   }

   void start() {
      runs++;
      timer.start();
   }

   void stop() {
      timer.stop();
      time += GetSec();
   }
};

class CH_PARALLEL_API ChTimerParallel {
 public:
   ChTimerParallel() {
      total_timers = 0;
      total_time = average_flops = average_bandwidth = 0;
   }
   ~ChTimerParallel() {
   }

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
   void SetFlop(std::string name,
                double f) {

      timer_list[name].flop = f;
      timer_list[name].compute_stats = true;
   }

   void SetMemory(std::string name,
                  double m) {

      timer_list[name].memory_ops = m;
      timer_list[name].compute_stats = true;
   }

   void start(std::string name) {
      timer_list[name].start();
   }

   void stop(std::string name) {
      timer_list[name].stop();

   }
   double GetTime(std::string name) {
      if (timer_list.count(name) == 0) {
         return 0;
      }
      return timer_list[name].time;
   }
   void PrintReport() {
      total_time = average_flops = average_bandwidth = 0;
      std::cout << "Timer Report:" << std::endl;
      std::cout << "------------" << std::endl;
      for (std::map<std::string, TimerData>::iterator it = timer_list.begin(); it != timer_list.end(); it++) {
         it->second.Compute();
         std::cout << "Name:\t" << it->first << "\t" << it->second.time;
         if (it->second.compute_stats) {
            std::cout << "\t" << it->second.flop_rate << "\t" << it->second.bandwidth;
            average_flops += it->second.flop_rate;
            average_bandwidth += it->second.bandwidth;
         }
         std::cout << std::endl;
         total_time += it->second.time;

      }
      std::cout << "------------" << std::endl;
      //cout << total_time << " " << average_flops / total_timers << " " << average_bandwidth / total_timers << endl;
   }

   double total_time;
   double average_flops;
   double average_bandwidth;
   int total_timers;
   std::map<std::string, TimerData> timer_list;
   std::map<std::string, TimerData>::iterator it;
};

}

#endif
