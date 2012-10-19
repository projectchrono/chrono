#ifndef CHTHREADS_H
#define CHTHREADS_H


//////////////////////////////////////////////////
//
//   ChThreads.h
//
//   Interface for multithreading (for multi-core 
//   processors, SPUs etc.)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"


#if ((defined WIN32)|| (defined WIN64))
	#include "ChThreadsWIN32.h"
#endif

#if (defined(__linux__) || defined(APPLE))
    #include "ChThreadsPOSIX.h"
#endif


namespace chrono
{


class ChApi ChThreads : private ChThreadsPlatformImplementation
{
public:

	ChThreads (ChThreadConstructionInfo& threadConstructionInfo) : ChThreadsPlatformImplementation(threadConstructionInfo) {};

	virtual ~ChThreads() {};

		///send messages to threads-fibers, executing the user function
	virtual	void sendRequest(unsigned int uiCommand, void* uiUserPtr, unsigned int threadId) 
		{
			ChThreadsPlatformImplementation::sendRequest(uiCommand, uiUserPtr, threadId);
		};

		///check for messages from threads-fibers
	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1)
		{
			ChThreadsPlatformImplementation::waitForResponse(puiArgument0, puiArgument1);
		};

		///start the threads-fibers 
	virtual	void startSPU()
		{
			ChThreadsPlatformImplementation::startSPU();
		};

		///tell the task scheduler we are done with the threads-fibers
	virtual	void stopSPU()
		{
			ChThreadsPlatformImplementation::stopSPU();
		};

		/// Wait for all the threads/fibers to finish
	virtual void flush()
		{
			ChThreadsPlatformImplementation::flush();
		};

		/// Returns the number of threads handled by this pool
	virtual int getNumberOfThreads()
		{
			return ChThreadsPlatformImplementation::getNumberOfThreads();
		};

	virtual std::string getUniqueName() 
		{
			return ChThreadsPlatformImplementation::getUniqueName();
		}
};








};  // END_OF_NAMESPACE____

#endif
