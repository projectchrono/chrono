//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "../ChTestConfig.h"
#include "chrono/physics/ChGlobal.h"

#if defined(__APPLE__)
#include <stdatomic.h>
#endif
#include <iostream>

using namespace chrono;
using namespace std;

static volatile int first = 100000;

int main() {
    ChTimer<double> OSX, GNU, CHRONO;
#if defined(__APPLE__)
    OSX.start();
    for (int j = 0; j < 100; j++) {
        for (int i = 0; i < 1000000; i++) {
            static volatile _Atomic(int) id = first;
            //OSAtomicIncrement32Barrier(&id); // DEPRECATED in macOS Sierra
            atomic_fetch_add(&id, 1);
        }
    }
    OSX.stop();
    double result_OSX = OSX();

    // Deprecated in macOS Sierra
    // cout << "OSAtomicIncrement32Barrier: " << result_OSX << " " << result_OSX / 100000000.0 << endl;
    cout << "atomic_fetch_add: " << result_OSX << " " << result_OSX / 100000000.0 << endl;
#endif

#if defined(__GNUC__)
    GNU.start();
    for (int j = 0; j < 100; j++) {
        for (int i = 0; i < 1000000; i++) {
            static volatile int id = first;
            __sync_add_and_fetch(&id, 1);
        }
    }
    GNU.stop();
    double result_GNU = GNU();
    cout << "__sync_add_and_fetch: " << result_GNU << " " << result_GNU / 100000000.0 << endl;
#endif

    CHRONO.start();
    for (int j = 0; j < 100; j++) {
        for (int i = 0; i < 1000000; i++) {
            GetUniqueIntID();
        }
    }
    CHRONO.stop();
    double result_CHRONO = CHRONO();
    cout << "Chrono GetUniqueIntID: " << result_CHRONO << " " << result_CHRONO / 100000000.0 << endl;

    return 0;
}
