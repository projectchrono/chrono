// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author:  Conlain Kelly
// =============================================================================
//
// c timers in ms for basic timing
// =============================================================================
#include <omp.h>
// Use doubles and count milliseconds
class milliTimer {
  public:
    milliTimer() {
        begin = 0;
        end = 0;
    }
    void start() { begin = omp_get_wtime(); }
    void stop() { end = omp_get_wtime(); }
    double count() {
        return 1000.0 * (double)(end - begin);  // convert to ms
    }

  private:
    double begin, end;
};