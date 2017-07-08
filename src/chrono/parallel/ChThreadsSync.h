/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt

Modified by: Alessandro Tasora

*/

#ifndef SPU_SYNC_H
#define SPU_SYNC_H

#include "chrono/core/ChApiCE.h"

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN

#ifdef _XBOX
#include <Xtl.h>
#else
#include <Windows.h>
#include <intrin.h>
#endif

/// Class that wraps a thread mutex, a locking mechanism to avoid,
/// for instance, multiple thread concurrent access to the same data.
class ChMutexSpinlock {
  public:
    ChMutexSpinlock() {
        // if (!InitializeCriticalSectionAndSpinCount(&_cs, 4000))
        //	throw SystemException("cannot create mutex");
        InitializeCriticalSection(&_cs);
    }
    ~ChMutexSpinlock() {
        DeleteCriticalSection(&_cs);  //???
    }

    void Lock() { EnterCriticalSection(&_cs); }

    void Unlock() { LeaveCriticalSection(&_cs); }

  private:
    CRITICAL_SECTION _cs;
};

#define EBUSY 16
#pragma intrinsic(_InterlockedExchange)
#pragma intrinsic(_ReadWriteBarrier)

/// Class that wraps a spinlock, a very fast locking mutex
/// that should be used only for short wait periods.
/// This uses MSVC intrinsics to mimic a fast spinlock as
/// in pthreads.h, but without the need of including
/// the pthreads library for windows.
/// See http://locklessinc.com/articles/pthreads_on_windows/
class ChApi ChSpinlock {
  public:
    ChSpinlock() { lock = 0; }
    ~ChSpinlock() {}
    void Lock() {
        while (_InterlockedExchange(&lock, EBUSY)) {
            /* Don't lock the bus whilst waiting */
            while (lock) {
                YieldProcessor();
                /* Compiler barrier.  Prevent caching of *l */
                _ReadWriteBarrier();
            }
        }
    }
    void Unlock() {
        _ReadWriteBarrier();
        lock = 0;
    }

  private:
    typedef long pseudo_pthread_spinlock_t;
    pseudo_pthread_spinlock_t lock;
};

#endif

//
// LINUX CODE
//

#if (defined(__linux__) || defined(__APPLE__) || defined(__FreeBSD__))

#include <pthread.h>
#include <semaphore.h>
// pthread_spin functions are not defined on APPLE
// this is a work around, no clue if it works. - Hammad
#if defined(__APPLE__)
#include <cerrno>
typedef int pthread_spinlock_t;

static inline int pthread_spin_init(pthread_spinlock_t* lock, int pshared) {
	__asm__ __volatile__("" ::: "memory");
    *lock = 0;
    return 0;
}

static inline int pthread_spin_destroy(pthread_spinlock_t* lock) {
    return 0;
}

static inline int pthread_spin_lock(pthread_spinlock_t* lock) {
    while (1) {
        int i;
        for (i = 0; i < 10000; i++) {
            if (__sync_bool_compare_and_swap(lock, 0, 1)) {
                return 0;
            }
        }
        sched_yield();
    }
}

static inline int pthread_spin_trylock(pthread_spinlock_t* lock) {
    if (__sync_bool_compare_and_swap(lock, 0, 1)) {
        return 0;
    }
    return EBUSY;
}

static inline int pthread_spin_unlock(pthread_spinlock_t* lock) {
	__asm__ __volatile__("" ::: "memory");
    *lock = 0;
    return 0;
}

#endif

/// Class that wraps a thread mutex, a locking mechanism to avoid,
/// for instance, multiple thread concurrent access to the same data.
class ChMutexSpinlock {
  public:
    ChMutexSpinlock() { pthread_mutex_init(&_cs_mutex, 0); }
    ~ChMutexSpinlock() { pthread_mutex_destroy(&_cs_mutex); }

    void Lock() { pthread_mutex_lock(&_cs_mutex); }

    void Unlock() { pthread_mutex_unlock(&_cs_mutex); }

  private:
    pthread_mutex_t _cs_mutex;
};

/// Class that wraps a spinlock, a very fast locking mutex
/// that should be used only for short wait periods.
/// ***TO BE TESTED***
class ChSpinlock {
  public:
    typedef pthread_spinlock_t SpinVariable;

    ChSpinlock() { pthread_spin_init(&spinVariable, 0); }
    ~ChSpinlock() { pthread_spin_destroy(&spinVariable); }

    void Lock() { pthread_spin_lock(&spinVariable); }

    void Unlock() { pthread_spin_unlock(&spinVariable); }

  private:
    SpinVariable spinVariable;
};

#endif

//
// CELL CODE
//

#if defined(__CELLOS_LV2__)

//#include <cell/atomic.h>
#include <cell/sync/mutex.h>

class ChSpinlock {
  public:
    typedef CellSyncMutex SpinVariable;

    ChSpinlock(SpinVariable* var) : spinVariable(var) {}
#ifndef __SPU__
    void Init() {
        //*spinVariable = 1;
        cellSyncMutexInitialize(spinVariable);
    }
#endif

#ifdef __SPU__
    void Lock() {
        // lock semaphore
        /*while (cellAtomicTestAndDecr32(atomic_buf, (uint64_t)spinVariable) == 0)
        {

        };*/
        cellSyncMutexLock((uint64_t)spinVariable);
    }

    void Unlock() {
        // cellAtomicIncr32(atomic_buf, (uint64_t)spinVariable);
        cellSyncMutexUnlock((uint64_t)spinVariable);
    }
#endif

  private:
    SpinVariable* spinVariable;
    ATTRIBUTE_ALIGNED128(uint32_t atomic_buf[32]);
};

#endif

#endif
