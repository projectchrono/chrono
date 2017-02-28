//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTHREADSPOSIX_H
#define CHTHREADSPOSIX_H

#include <string>
#include "chrono/core/ChApiCE.h"
#include "chrono/parallel/ChThreadsFunct.h"
#include "chrono/collision/bullet/LinearMath/btAlignedObjectArray.h"

// macOS platform-specific headers:
#if defined(__APPLE__)
#include <fcntl.h>
#include <sys/stat.h>
#include <cerrno>
#endif

// UNIX - LINUX platform specific:
#include <pthread.h>
#include <semaphore.h>

namespace chrono {

typedef unsigned int uint32_t;

struct ChThreadStatePOSIX {
    uint32_t m_taskId;

    uint32_t m_commandId;
    uint32_t m_status;

    ChThreadFunc m_userThreadFunc;  // user function
    void* m_userPtr;                // user data
    void* m_lsMemory;               // initialized using PosixLocalStoreMemorySetupFunc

    pthread_t thread;

    #if defined(__APPLE__)
    sem_t* startSemaphore;
    #else
    sem_t startSemaphore;
    #endif

    sem_t* mainSemaphore;

    unsigned long threadUsed;
};

class ChApi ChThreadsPOSIX {
    btAlignedObjectArray<ChThreadStatePOSIX> m_activeSpuStatus;
    btAlignedObjectArray<void*> m_completeHandles;

    std::string uniqueName;

    // this semaphore will signal, if and how many threads are finished with their work
    #if defined(__APPLE__)
    sem_t* mainSemaphore;
    #else
    sem_t mainSemaphore;
    #endif

  public:
    /// Constructor: create and initialize N threads.
    ChThreadsPOSIX(ChThreadConstructionInfo& threadConstructionInfo);

    /// Destructor: cleanup/shutdown
    virtual ~ChThreadsPOSIX();

    void makeThreads(ChThreadConstructionInfo& threadInfo);

    virtual void sendRequest(uint32_t uiCommand, void* uiUserPtr, unsigned int threadId);

    virtual void waitForResponse(unsigned int* puiArgument0, unsigned int* puiArgument1);

    virtual void startSPU();

    virtual void stopSPU();

    virtual void flush();

    virtual int getNumberOfThreads() { return m_activeSpuStatus.size(); }

    virtual std::string getUniqueName() { return uniqueName; }
};

typedef ChThreadsPOSIX ChThreadsPlatformImplementation;

}  // end namespace chrono

#endif
