#ifndef CHTHREADSWIN32_H
#define CHTHREADSWIN32_H


//////////////////////////////////////////////////
//
//   ChThreadsWIN32.h
//
//   Interface for multithreading (for multi-core 
//   processors) on the Window platform
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <string>
#include "parallel/ChThreadsFunct.h"
#include "LinearMath/btAlignedObjectArray.h"


namespace chrono
{

typedef unsigned int      uint32_t;

//typedef void (*Win32ThreadFunc)(void* userPtr,void* lsMemory);
//typedef void* (*Win32lsMemorySetupFunc)();



struct	ChThreadStateWIN32
{
	uint32_t	m_taskId;

	uint32_t	m_commandId;
	uint32_t	m_status;

	ChThreadFunc	m_userThreadFunc; //user function
	void*	m_userPtr;				  //user data
	void*	m_lsMemory;				  //initialized using Win32LocalStoreMemorySetupFunc

	void*	m_threadHandle; 

	void*	m_eventStartHandle;
	char	m_eventStartHandleName[32];

	void*	m_eventCompletetHandle;
	char	m_eventCompletetHandleName[32];
};




class ChThreadsWIN32 // : public ChThreads
{

	btAlignedObjectArray<ChThreadStateWIN32>	m_activeSpuStatus;
	btAlignedObjectArray<void*>				    m_completeHandles;

	std::string uniqueName;

public:

	

		/// Constructor: create and initialize N threads. 
	ChThreadsWIN32(ChThreadConstructionInfo& threadConstructionInfo);

		/// Destructor: cleanup/shutdown 
	virtual	~ChThreadsWIN32();


	void	makeThreads(ChThreadConstructionInfo&	threadInfo);


	virtual	void sendRequest(uint32_t uiCommand, void* uiUserPtr, unsigned int threadId);

	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);

	virtual	void startSPU();

	virtual	void stopSPU();

	virtual void flush();

	virtual int getNumberOfThreads() {return  m_activeSpuStatus.size();}

	virtual std::string getUniqueName() {return uniqueName;}
};




typedef ChThreadsWIN32 ChThreadsPlatformImplementation;


};  // END_OF_NAMESPACE____

#endif


