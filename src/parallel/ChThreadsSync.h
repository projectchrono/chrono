/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt

Modified by: Alessandro Tasora

*/

#ifndef SPU_SYNC_H
#define	SPU_SYNC_H


//#include "PlatformDefinitions.h"


#if defined(WIN32)||defined(WIN64)

#define WIN32_LEAN_AND_MEAN

#ifdef _XBOX
#include <Xtl.h>
#else
#include <Windows.h>
#endif

class ChMutexSpinlock
{
public:

	ChMutexSpinlock ()
	{
		//if (!InitializeCriticalSectionAndSpinCount(&_cs, 4000))
		//	throw SystemException("cannot create mutex");
		InitializeCriticalSection(&_cs);
	}
	~ChMutexSpinlock()
	{
		DeleteCriticalSection(&_cs); //???
	}

	void Lock ()
	{
		EnterCriticalSection(&_cs);
	}

	void Unlock ()
	{
		LeaveCriticalSection(&_cs);
	}

private:
	CRITICAL_SECTION _cs;
};


class ChSpinlock
{
public:
	typedef CRITICAL_SECTION SpinVariable;

	ChSpinlock (SpinVariable* var)
		: spinVariable (var)
	{}

	void Init ()
	{
		InitializeCriticalSection(spinVariable);
	}

	void Lock ()
	{
		EnterCriticalSection(spinVariable);
	}

	void Unlock ()
	{
		LeaveCriticalSection(spinVariable);
	}

private:
	SpinVariable* spinVariable;
};

#endif



//
// LINUX CODE
//


#if (defined(__linux__)||defined(APPLE))

#include <pthread.h>
#include <semaphore.h>

class ChMutexSpinlock
{
public:
	ChMutexSpinlock ()
	{
		pthread_mutex_init( &_cs_mutex,0);
	}
	~ChMutexSpinlock()
	{
		pthread_mutex_destroy( &_cs_mutex );
	}

	void Lock ()
	{
		pthread_mutex_lock( &_cs_mutex );
	}

	void Unlock ()
	{
		pthread_mutex_unlock( &_cs_mutex );
	}

private:
	pthread_mutex_t _cs_mutex;
};


class ChSpinlock
{
public:
	typedef pthread_mutex_t SpinVariable;

	ChSpinlock (SpinVariable* var)
		: spinVariable (var)
	{}

	void Init ()
	{
		pthread_mutex_init( spinVariable ,0);
	}

	void Lock ()
	{
		pthread_mutex_lock( spinVariable);
	}

	void Unlock ()
	{
		pthread_mutex_unlock( spinVariable);
	}

private:
	SpinVariable* spinVariable;
};

#endif


//
// CELL CODE
//


#if defined (__CELLOS_LV2__)

//#include <cell/atomic.h>
#include <cell/sync/mutex.h>

class ChSpinlock
{
public:
	typedef CellSyncMutex SpinVariable;

	ChSpinlock (SpinVariable* var)
		: spinVariable (var)
	{}
#ifndef __SPU__
	void Init ()
	{
		//*spinVariable = 1;
		cellSyncMutexInitialize(spinVariable);
	}
#endif

#ifdef __SPU__
	void Lock ()
	{
		// lock semaphore
		/*while (cellAtomicTestAndDecr32(atomic_buf, (uint64_t)spinVariable) == 0) 
		{

		};*/
		cellSyncMutexLock((uint64_t)spinVariable);
	}

	void Unlock ()
	{
		//cellAtomicIncr32(atomic_buf, (uint64_t)spinVariable);
		cellSyncMutexUnlock((uint64_t)spinVariable);
	}
#endif

private:
	SpinVariable*	spinVariable;
	ATTRIBUTE_ALIGNED128(uint32_t		atomic_buf[32]);
};

#endif




#endif

