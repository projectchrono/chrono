#include "../ChTestConfig.h"
#include "physics/ChGlobal.h"
#if defined(__APPLE__)
#include <libkern/OSAtomic.h>
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
            static volatile int32_t id = first;
            OSAtomicIncrement32Barrier(&id);
        }
    }
    OSX.stop();
    double result_OSX = OSX();
    cout << "OSAtomicIncrement32Barrier: " << result_OSX << " " << result_OSX / 100000000.0 << endl;
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
