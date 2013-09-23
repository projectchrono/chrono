//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTHREADSPOSIX_H
#define CHTHREADSPOSIX_H


//////////////////////////////////////////////////
//
//   ChThreadsPOSIX.h
//
//   Interface for multithreading (for multi-core 
//   processors) on the Window platform
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <string>
#include "core/ChApiCE.h"
#include "parallel/ChThreadsFunct.h"
#include "LinearMath/btAlignedObjectArray.h"

// UNIX - LINUX platform specific:
#include <pthread.h>
#include <semaphore.h>


namespace chrono
{

typedef unsigned int      uint32_t;	




struct	ChThreadStatePOSIX
{
	uint32_t	m_taskId;

	uint32_t	m_commandId;
	uint32_t	m_status;

	ChThreadFunc	m_userThreadFunc; //user function
	void*	m_userPtr;  //user data
	void*	m_lsMemory; //initialized using PosixLocalStoreMemorySetupFunc

    pthread_t thread;
    sem_t startSemaphore;

	sem_t* mainSemaphore;

    unsigned long threadUsed;
};




class ChApi ChThreadsPOSIX 
{

	btAlignedObjectArray<ChThreadStatePOSIX>	m_activeSpuStatus;
	btAlignedObjectArray<void*>				    m_completeHandles;

	std::string uniqueName;

	// this semaphore will signal, if and how many threads are finished with their work
	sem_t mainSemaphore;

public:

	

		/// Constructor: create and initialize N threads. 
	ChThreadsPOSIX(ChThreadConstructionInfo& threadConstructionInfo);

		/// Destructor: cleanup/shutdown 
	virtual	~ChThreadsPOSIX();


	void	makeThreads(ChThreadConstructionInfo&	threadInfo);


	virtual	void sendRequest(uint32_t uiCommand, void* uiUserPtr, unsigned int threadId);

	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);

	virtual	void startSPU();

	virtual	void stopSPU();

	virtual void flush();

	virtual int getNumberOfThreads() {return  m_activeSpuStatus.size();}

	virtual std::string getUniqueName() {return uniqueName;}
};




typedef ChThreadsPOSIX ChThreadsPlatformImplementation;


};  // END_OF_NAMESPACE____

#endif


