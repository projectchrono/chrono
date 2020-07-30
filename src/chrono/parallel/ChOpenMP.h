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

#ifndef CHOPENMP_H
#define CHOPENMP_H

#include "chrono/core/ChApiCE.h"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace chrono {

#ifdef _OPENMP

/// Class that wraps a 'omp_lock_t' for doing a mutex in OpenMP parallel sections.
class ChApi CHOMPmutex {
  public:
    CHOMPmutex() { omp_init_lock(&lock); }
    ~CHOMPmutex() { omp_destroy_lock(&lock); }
    void Lock() { omp_set_lock(&lock); }
    void Unlock() { omp_unset_lock(&lock); }

    CHOMPmutex(const CHOMPmutex&) { omp_init_lock(&lock); }
    CHOMPmutex& operator=(const CHOMPmutex&) { return *this; }

  private:
    omp_lock_t lock;
};

/// Class that wraps some useful functions in OpenMP
/// (in case no OpenMP is used, it defaults to no-op functions)
class ChApi CHOMPfunctions {
  public:
    /// Sets the number of threads in subsequent parallel regions, unless overridden by a 'num_threads' clause
    static void SetNumThreads(int mth) { omp_set_num_threads(mth); }

    /// Returns the number of threads in the parallel region.
    static int GetNumThreads() { return omp_get_num_threads(); }

    /// Returns the thread number of the thread executing within its thread team.
    static int GetThreadNum() { return omp_get_thread_num(); }

    /// Returns the number of available processors on this machine
    static int GetNumProcs() { return omp_get_num_procs(); }

    /// Returns the max.number of threads that would be used by default if num_threads not specified.
    /// This is the same number as GetNumProcs() on most OMP implementations.
    static int GetMaxThreads() { return omp_get_max_threads(); }
};

#else
/// Dummy no-op mutex in case that no parallel multithreading via OpenMP is available.
class ChApi CHOMPmutex {
  public:
    void Lock() {}
    void Unlock() {}
};

/// Dummy no-op functions in case that no parallel multithreading via OpenMP is available.
class ChApi CHOMPfunctions {
  public:
    static void SetNumThreads(int mth) {}
    static int GetNumThreads() { return 1; }
    static int GetThreadNum() { return 0; }
    static int GetNumProcs() { return 1; }
    static int GetMaxThreads() { return 1; }
};

#endif

/// Exception-safe wrapper to a mutex: it automatically locks the
/// mutex as soon as the wrapper is created, and releases the
/// mutex when the wrapper is deleted (you simply put the wrapper in
/// a code section delimited by {} parentheses, so it is deleted
/// by the compiler when exiting the scope of the section or in case
/// of premature exit because of an exception throw)
struct CHOMPscopedLock {
    explicit CHOMPscopedLock(CHOMPmutex& m) : mut(m), locked(true) { mut.Lock(); }

    ~CHOMPscopedLock() { Unlock(); }

    void Unlock() {
        if (!locked)
            return;
        locked = false;
        mut.Unlock();
    }

    void LockAgain() {
        if (locked)
            return;
        mut.Lock();
        locked = true;
    }

  private:
    CHOMPmutex& mut;
    bool locked;

  private:
    // trick to prevent copying the scoped lock
    void operator=(const CHOMPscopedLock&);
    CHOMPscopedLock(const CHOMPscopedLock&);
};

}  // end namespace chrono

#endif
