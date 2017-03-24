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

#if defined _WIN32

#include <cstdio>
#include <Windows.h>

#include "chrono/parallel/ChThreadsWIN32.h"
#include "chrono/core/ChLog.h"

namespace chrono {

ChThreadsWIN32::ChThreadsWIN32(ChThreadConstructionInfo& threadConstructionInfo) {
    makeThreads(threadConstructionInfo);
}

ChThreadsWIN32::~ChThreadsWIN32() {
    stopSPU();

    // STOP thread fx

    for (int i = 0; i < m_activeSpuStatus.size(); i++)
        this->sendRequest(1, 0, i);

    for (int i = 0; i < m_activeSpuStatus.size(); i++) {
        WaitForSingleObject(m_activeSpuStatus[i].m_threadHandle, 1000);
        CloseHandle(m_activeSpuStatus[i].m_threadHandle);
        CloseHandle(m_activeSpuStatus[i].m_eventCompletetHandle);
        CloseHandle(m_activeSpuStatus[i].m_eventStartHandle);
    }
}

DWORD WINAPI Thread_no_1(LPVOID lpParam) {
    ChThreadStateWIN32* status = (ChThreadStateWIN32*)lpParam;

    while (1) {
        WaitForSingleObject(status->m_eventStartHandle, INFINITE);
        btAssert(status->m_status);

        void* userPtr = status->m_userPtr;

        if (userPtr) {
            status->m_userThreadFunc(userPtr, status->m_lsMemory);
            status->m_status = 2;
            SetEvent(status->m_eventCompletetHandle);
        } else {
            // exit Thread
            break;
        }
    }

    return 0;
}

/// send messages to SPUs
void ChThreadsWIN32::sendRequest(uint32_t uiCommand, void* uiUserPtr, uint32_t threadId) {
    ChThreadStateWIN32& spuStatus = m_activeSpuStatus[threadId];
    btAssert(threadId >= 0);
    btAssert(threadId < (unsigned)m_activeSpuStatus.size());

    spuStatus.m_commandId = uiCommand;
    spuStatus.m_status = 1;
    spuStatus.m_userPtr = uiUserPtr;

    /// fire event to start new task
    SetEvent(spuStatus.m_eventStartHandle);
}

/// check for messages from SPUs
void ChThreadsWIN32::waitForResponse(unsigned int* puiArgument0, unsigned int* puiArgument1) {
    btAssert(m_activeSpuStatus.size());

    int last = -1;

    DWORD res = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], 0, INFINITE);
    btAssert(res != WAIT_FAILED);
    last = res - WAIT_OBJECT_0;

    ChThreadStateWIN32& spuStatus = m_activeSpuStatus[last];
    btAssert(spuStatus.m_threadHandle);
    btAssert(spuStatus.m_eventCompletetHandle);

    btAssert(spuStatus.m_status > 1);
    spuStatus.m_status = 0;

    /// need to find an active spu
    btAssert(last >= 0);

    *puiArgument0 = spuStatus.m_taskId;
    *puiArgument1 = spuStatus.m_status;
}

void ChThreadsWIN32::makeThreads(ChThreadConstructionInfo& threadConstructionInfo) {
    m_activeSpuStatus.resize(threadConstructionInfo.m_numThreads);
    m_completeHandles.resize(threadConstructionInfo.m_numThreads);

    uniqueName = threadConstructionInfo.m_uniqueName;

    for (int i = 0; i < threadConstructionInfo.m_numThreads; i++) {
        ChThreadStateWIN32& spuStatus = m_activeSpuStatus[i];

        LPSECURITY_ATTRIBUTES lpThreadAttributes = NULL;
        SIZE_T dwStackSize = threadConstructionInfo.m_threadStackSize;
        LPTHREAD_START_ROUTINE lpStartAddress = &Thread_no_1;
        LPVOID lpParameter = &spuStatus;
        DWORD dwCreationFlags = 0;
        LPDWORD lpThreadId = 0;

        spuStatus.m_userPtr = 0;

        sprintf(spuStatus.m_eventStartHandleName, "eventStart%s%d", threadConstructionInfo.m_uniqueName, i);
        spuStatus.m_eventStartHandle = CreateEvent(0, false, false, spuStatus.m_eventStartHandleName);

        sprintf(spuStatus.m_eventCompletetHandleName, "eventComplete%s%d", threadConstructionInfo.m_uniqueName, i);
        spuStatus.m_eventCompletetHandle = CreateEvent(0, false, false, spuStatus.m_eventCompletetHandleName);

        m_completeHandles[i] = spuStatus.m_eventCompletetHandle;
        HANDLE handle =
            CreateThread(lpThreadAttributes, dwStackSize, lpStartAddress, lpParameter, dwCreationFlags, lpThreadId);
        SetThreadPriority(handle, THREAD_PRIORITY_HIGHEST);

        spuStatus.m_taskId = i;
        spuStatus.m_commandId = 0;
        spuStatus.m_status = 0;
        spuStatus.m_threadHandle = handle;
        spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
        spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;
    }
}

void ChThreadsWIN32::flush() {
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

void ChThreadsWIN32::startSPU() {
}

void ChThreadsWIN32::stopSPU() {
}

}  // end namespace chrono

#endif
