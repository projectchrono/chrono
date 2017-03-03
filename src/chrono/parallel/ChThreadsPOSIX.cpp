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

#if (defined(__linux__) || defined(__APPLE__) || defined(__FreeBSD__))

#define DWORD unsigned int

#include <cstdio>
#include "chrono/parallel/ChThreadsPOSIX.h"

#define checkPThreadFunction(returnValue)                                                       \
    if (0 != returnValue) {                                                                     \
        printf("PThread problem at line %i in file %s: %i\n", __LINE__, __FILE__, returnValue); \
    }

namespace chrono {

ChThreadsPOSIX::ChThreadsPOSIX(ChThreadConstructionInfo& threadConstructionInfo) {
    makeThreads(threadConstructionInfo);
}

ChThreadsPOSIX::~ChThreadsPOSIX() {
    stopSPU();
}

static void* threadFunction(void* argument) {
    ChThreadStatePOSIX* status = (ChThreadStatePOSIX*)argument;

    while (1) {
        checkPThreadFunction(
        #if defined(__APPLE__)
            sem_wait(status->startSemaphore)
        #else
            sem_wait(&status->startSemaphore)
        #endif
        );

        void* userPtr = status->m_userPtr;

        if (userPtr) {
            status->m_userThreadFunc(userPtr, status->m_lsMemory);
            status->m_status = 2;
            checkPThreadFunction(
                sem_post(status->mainSemaphore)
            );

            // status->threadUsed++;
        } else {
            // exit Thread
            // status->m_status = 3; //??? needed ???
            // checkPThreadFunction(sem_post(&this->mainSemaphore)); //??? needed ???
            break;
        }
    }

    return 0;
}

/// send messages to SPUs
void ChThreadsPOSIX::sendRequest(uint32_t uiCommand,
                                 void* uiUserPtr,
                                 unsigned int threadId)  //(uint32_t uiCommand, uint32_t uiArgument0, uint32_t taskId)
{
    ChThreadStatePOSIX& spuStatus = m_activeSpuStatus[threadId];

    spuStatus.m_commandId = uiCommand;
    spuStatus.m_status = 1;
    spuStatus.m_userPtr = uiUserPtr;

    // fire event to start new task
    checkPThreadFunction(
    #if defined(__APPLE__)
        sem_post(spuStatus.startSemaphore)
    #else
        sem_post(&spuStatus.startSemaphore)
    #endif
    );
}

/// check for messages from SPUs
void ChThreadsPOSIX::waitForResponse(unsigned int* puiArgument0, unsigned int* puiArgument1) {
    /// We should wait for (one of) the first tasks to finish (or other SPU messages), and report its response

    /// A possible response can be 'yes, SPU handled it', or 'no, please do a PPU fallback'

    btAssert(m_activeSpuStatus.size());

    // wait for any of the threads to finish
    checkPThreadFunction(
    #if defined(__APPLE__)
        sem_wait(this->mainSemaphore)
    #else
        sem_wait(&this->mainSemaphore)
    #endif
    );

    // get at least one thread which has finished
    int last = -1;

    for (int t = 0; t < m_activeSpuStatus.size(); ++t) {
        if (2 == m_activeSpuStatus[t].m_status) {
            last = t;
            break;
        }
    }

    ChThreadStatePOSIX& spuStatus = m_activeSpuStatus[last];

    btAssert(spuStatus.m_status > 1);
    spuStatus.m_status = 0;

    // need to find an active spu
    btAssert(last >= 0);

    *puiArgument0 = spuStatus.m_taskId;
    *puiArgument1 = spuStatus.m_status;
}

void ChThreadsPOSIX::makeThreads(ChThreadConstructionInfo& threadConstructionInfo) {
    m_activeSpuStatus.resize(threadConstructionInfo.m_numThreads);

    uniqueName = threadConstructionInfo.m_uniqueName;
    
    #if defined(__APPLE__)
    std::string sem_prefix("/" + uniqueName);
    #endif

    #if defined(__APPLE__)
    this->mainSemaphore = sem_open(
        std::string(sem_prefix+"_main").c_str(), // Construct a unique name
        O_CREAT,           // Create the semaphore if it doesn't exist
        S_IRUSR | S_IWUSR, // Set as readable and writable by this process
        0                  // Set the initial value to 0
    );
    checkPThreadFunction(errno);
    #else
    checkPThreadFunction(sem_init(&this->mainSemaphore, 0, 0));
    #endif

    for (int i = 0; i < threadConstructionInfo.m_numThreads; i++) {
        ChThreadStatePOSIX& spuStatus = m_activeSpuStatus[i];

        #if defined(__APPLE__)
        spuStatus.mainSemaphore = this->mainSemaphore;
        #else
        spuStatus.mainSemaphore = &this->mainSemaphore;
        #endif

        #if defined(__APPLE__)
        spuStatus.startSemaphore = sem_open(
            std::string(sem_prefix+"_start"+std::to_string(i)).c_str(),
            O_CREAT,
            S_IRUSR | S_IWUSR,
            0
        );
        checkPThreadFunction(errno);
        #else
        checkPThreadFunction(sem_init(&spuStatus.startSemaphore, 0, 0));
        #endif

        checkPThreadFunction(pthread_create(&spuStatus.thread, NULL, &threadFunction, (void*)&spuStatus));

        spuStatus.m_userPtr = 0;

        spuStatus.m_taskId = i;
        spuStatus.m_commandId = 0;
        spuStatus.m_status = 0;
        spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
        spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;
        spuStatus.threadUsed = 0;
    }
}

void ChThreadsPOSIX::flush() {
    int nthreads = m_activeSpuStatus.size();

    while (1) {
        bool stillrunning = false;
        for (int i = 0; i < nthreads; i++) {
            if (m_activeSpuStatus[i].m_status != 0) {
                stillrunning = true;
                unsigned int mtaskid;
                unsigned int mres2;
                waitForResponse(&mtaskid, &mres2);
            }
        }
        if (!stillrunning)
            return;
    }
}

void ChThreadsPOSIX::startSPU() {
}

/// tell the task scheduler we are done with the SPU tasks
void ChThreadsPOSIX::stopSPU() {

    #if defined(__APPLE__)
    std::string sem_prefix("/" + uniqueName);
    #endif

    for (size_t t = 0; t < m_activeSpuStatus.size(); ++t) {
        ChThreadStatePOSIX& spuStatus = m_activeSpuStatus[t];

        #if defined(__APPLE__)
        checkPThreadFunction(sem_close(spuStatus.startSemaphore));
        checkPThreadFunction(sem_unlink(
            std::string(sem_prefix+"_start"+std::to_string(t)).c_str()
        ));
        #else
        checkPThreadFunction(sem_destroy(&spuStatus.startSemaphore));
        #endif

        checkPThreadFunction(pthread_cancel(spuStatus.thread));
    }

    #if defined(__APPLE__)
    checkPThreadFunction(sem_close(this->mainSemaphore));
    checkPThreadFunction(sem_unlink(
        std::string(sem_prefix+"_main").c_str()
    ));
    #else
    checkPThreadFunction(sem_destroy(&this->mainSemaphore));
    #endif

    m_activeSpuStatus.clear();
}

}  // end namespace chrono

#endif
