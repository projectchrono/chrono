/*
Copyright (c) 2003-2014 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_THREADS_H
#define BT_THREADS_H

#include "cbtScalar.h"  // has definitions like SIMD_FORCE_INLINE

#if defined(_MSC_VER) && _MSC_VER >= 1600
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#endif

#ifndef BT_OVERRIDE
#define BT_OVERRIDE
#endif

// Don't set this to larger than 64, without modifying cbtThreadSupportPosix
// and cbtThreadSupportWin32. They use UINT64 bit-masks.
const unsigned int BT_MAX_THREAD_COUNT = 64;  // only if BT_THREADSAFE is 1

// for internal use only
bool cbtIsMainThread();
bool cbtThreadsAreRunning();
unsigned int cbtGetCurrentThreadIndex();
void cbtResetThreadIndexCounter();  // notify that all worker threads have been destroyed

///
/// cbtSpinMutex -- lightweight spin-mutex implemented with atomic ops, never puts
///               a thread to sleep because it is designed to be used with a task scheduler
///               which has one thread per core and the threads don't sleep until they
///               run out of tasks. Not good for general purpose use.
///
class cbtSpinMutex
{
	int mLock;

public:
	cbtSpinMutex()
	{
		mLock = 0;
	}
	void lock();
	void unlock();
	bool tryLock();
};

//
// NOTE: cbtMutex* is for internal Bullet use only
//
// If BT_THREADSAFE is undefined or 0, should optimize away to nothing.
// This is good because for the single-threaded build of Bullet, any calls
// to these functions will be optimized out.
//
// However, for users of the multi-threaded build of Bullet this is kind
// of bad because if you call any of these functions from external code
// (where BT_THREADSAFE is undefined) you will get unexpected race conditions.
//
SIMD_FORCE_INLINE void cbtMutexLock(cbtSpinMutex* mutex)
{
#if BT_THREADSAFE
	mutex->lock();
#else
	(void)mutex;
#endif  // #if BT_THREADSAFE
}

SIMD_FORCE_INLINE void cbtMutexUnlock(cbtSpinMutex* mutex)
{
#if BT_THREADSAFE
	mutex->unlock();
#else
	(void)mutex;
#endif  // #if BT_THREADSAFE
}

SIMD_FORCE_INLINE bool cbtMutexTryLock(cbtSpinMutex* mutex)
{
#if BT_THREADSAFE
	return mutex->tryLock();
#else
	(void)mutex;
	return true;
#endif  // #if BT_THREADSAFE
}

//
// cbtIParallelForBody -- subclass this to express work that can be done in parallel
//
class cbtIParallelForBody
{
public:
	virtual ~cbtIParallelForBody() {}
	virtual void forLoop(int iBegin, int iEnd) const = 0;
};

//
// cbtIParallelSumBody -- subclass this to express work that can be done in parallel
//                       and produces a sum over all loop elements
//
class cbtIParallelSumBody
{
public:
	virtual ~cbtIParallelSumBody() {}
	virtual cbtScalar sumLoop(int iBegin, int iEnd) const = 0;
};

//
// cbtITaskScheduler -- subclass this to implement a task scheduler that can dispatch work to
//                     worker threads
//
class cbtITaskScheduler
{
public:
	cbtITaskScheduler(const char* name);
	virtual ~cbtITaskScheduler() {}
	const char* getName() const { return m_name; }

	virtual int getMaxNumThreads() const = 0;
	virtual int getNumThreads() const = 0;
	virtual void setNumThreads(int numThreads) = 0;
	virtual void parallelFor(int iBegin, int iEnd, int grainSize, const cbtIParallelForBody& body) = 0;
	virtual cbtScalar parallelSum(int iBegin, int iEnd, int grainSize, const cbtIParallelSumBody& body) = 0;
	virtual void sleepWorkerThreadsHint() {}  // hint the task scheduler that we may not be using these threads for a little while

	// internal use only
	virtual void activate();
	virtual void deactivate();

protected:
	const char* m_name;
	unsigned int m_savedThreadCounter;
	bool m_isActive;
};

// set the task scheduler to use for all calls to cbtParallelFor()
// NOTE: you must set this prior to using any of the multi-threaded "Mt" classes
void cbtSetTaskScheduler(cbtITaskScheduler* ts);

// get the current task scheduler
cbtITaskScheduler* cbtGetTaskScheduler();

// get non-threaded task scheduler (always available)
cbtITaskScheduler* cbtGetSequentialTaskScheduler();

// create a default task scheduler (Win32 or pthreads based)
cbtITaskScheduler* cbtCreateDefaultTaskScheduler();

// get OpenMP task scheduler (if available, otherwise returns null)
cbtITaskScheduler* cbtGetOpenMPTaskScheduler();

// get Intel TBB task scheduler (if available, otherwise returns null)
cbtITaskScheduler* cbtGetTBBTaskScheduler();

// get PPL task scheduler (if available, otherwise returns null)
cbtITaskScheduler* cbtGetPPLTaskScheduler();

// cbtParallelFor -- call this to dispatch work like a for-loop
//                 (iterations may be done out of order, so no dependencies are allowed)
void cbtParallelFor(int iBegin, int iEnd, int grainSize, const cbtIParallelForBody& body);

// cbtParallelSum -- call this to dispatch work like a for-loop, returns the sum of all iterations
//                 (iterations may be done out of order, so no dependencies are allowed)
cbtScalar cbtParallelSum(int iBegin, int iEnd, int grainSize, const cbtIParallelSumBody& body);

#endif
