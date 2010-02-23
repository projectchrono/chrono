#ifndef CHTHREADSFUNCT_H
#define CHTHREADSFUNCT_H


//////////////////////////////////////////////////
//
//   ChThreadsFunct.h
//
//   Stuff used by interface for multithreading (for multi-core 
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




namespace chrono
{


typedef void  (*ChThreadFunc)(void* userPtr,void* lsMemory);

typedef void* (*ChMemorySetupFunc)();


struct	ChThreadConstructionInfo
	{
		ChThreadConstructionInfo(char* uniqueName,
									ChThreadFunc userThreadFunc,
									ChMemorySetupFunc	lsMemoryFunc,
									int numThreads=1,
									int threadStackSize=65535
									)
									:m_uniqueName(uniqueName),
									m_userThreadFunc(userThreadFunc),
									m_lsMemoryFunc(lsMemoryFunc),
									m_numThreads(numThreads),
									m_threadStackSize(threadStackSize)
		{

		}

		char*					m_uniqueName;
		ChThreadFunc			m_userThreadFunc;
		ChMemorySetupFunc		m_lsMemoryFunc;
		int						m_numThreads;
		int						m_threadStackSize;

	};


};  // END_OF_NAMESPACE____

#endif


