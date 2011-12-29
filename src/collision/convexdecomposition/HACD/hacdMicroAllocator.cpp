#include <hacdMicroAllocator.h>

/*!
**
** Copyright (c) 2009 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
**
** If you find this code useful or you are feeling particularily generous I would
** ask that you please go to http://www.amillionpixels.us and make a donation
** to Troy DeMolay.
**
**
** If you wish to contact me you can use the following methods:
**
** Skype ID: jratcliff63367
** email: jratcliffscarab@gmail.com
**
**
** The MIT license:
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is furnished
** to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.

** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#include <new>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

#ifdef WIN32
#include <windows.h>
#endif

#if defined(__APPLE__) || defined(LINUX)
#include <pthread.h>
#endif


#pragma warning(disable:4100)

namespace HACD
{

//==================================================================================
class MemMutex
{
	public:
		MemMutex(void);
		~MemMutex(void);

	public:
		// Blocking Lock.
		void Lock(void);

		// Unlock.
		void Unlock(void);

private:
		#if defined(_WIN32) || defined(_XBOX)
		CRITICAL_SECTION m_Mutex;
		#elif defined(__APPLE__) || defined(LINUX)
		pthread_mutex_t  m_Mutex;
		#endif
};

//==================================================================================
MemMutex::MemMutex(void)
{
#if defined(_WIN32) || defined(_XBOX)
	InitializeCriticalSection(&m_Mutex);
#elif defined(__APPLE__) || defined(LINUX)
	pthread_mutexattr_t mta;
	pthread_mutexattr_init(&mta);
	pthread_mutexattr_settype(&mta, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&m_Mutex, &mta);
	pthread_mutexattr_destroy(&mta);
#endif
}

//==================================================================================
MemMutex::~MemMutex(void)
{
#if defined(_WIN32) || defined(_XBOX)
	DeleteCriticalSection(&m_Mutex);
#elif defined(__APPLE__) || defined(LINUX)
	pthread_mutex_destroy(&m_Mutex);
#endif
}

//==================================================================================
// Blocking Lock.
//==================================================================================
void MemMutex::Lock(void)
{
#if defined(_WIN32) || defined(_XBOX)
	EnterCriticalSection(&m_Mutex);
#elif defined(__APPLE__) || defined(LINUX)
	pthread_mutex_lock(&m_Mutex);
#endif
}

//==================================================================================
// Unlock.
//==================================================================================
void MemMutex::Unlock(void)
{
#if defined(_WIN32) || defined(_XBOX)
	LeaveCriticalSection(&m_Mutex);
#elif defined(__APPLE__) || defined(LINUX)
	pthread_mutex_unlock(&m_Mutex);
#endif
}



struct ChunkHeader
{
  ChunkHeader   *mNextChunk;
};

// interface to add and remove new chunks to the master list.
class MicroChunkUpdate
{
public:
  virtual void addMicroChunk(NxU8 *memStart,NxU8 *memEnd,MemoryChunk *chunk) = 0;
  virtual void removeMicroChunk(MemoryChunk *chunk) = 0;
};

class MemoryHeader
{
public:
  MemoryHeader *mNext;
};

// a single fixed size chunk for micro-allocations.
class MemoryChunk
{
public:
  MemoryChunk(void)
  {
    mData       = 0;
    mDataEnd    = 0;
    mUsedCount  = 0;
    mFreeList   = 0;
    mMyHeap     = false;
    mChunkSize  = 0;
  }

  NxU8 * init(NxU8 *chunkBase,NxU32 chunkSize,NxU32 maxChunks)
  {
    mChunkSize  = chunkSize;
    mData     =  chunkBase;
    mDataEnd  = mData+(chunkSize*maxChunks);
    mFreeList = (MemoryHeader *) mData;
    MemoryHeader *scan = mFreeList;
    NxU8 *data = mData;
    data+=chunkSize;
    for (NxU32 i=0; i<(maxChunks-1); i++)
    {
        MemoryHeader *next = (MemoryHeader *)data;
        scan->mNext = next;
        data+=chunkSize;
        scan = next;
    }
    scan->mNext = 0;
    return mDataEnd;
  }

  inline void * allocate(MicroHeap *heap,NxU32 chunkSize,NxU32 maxChunks,MicroChunkUpdate *update)
  {
    void *ret = 0;

    if ( mData == 0 )
    {
        mMyHeap = true;
        mData = (NxU8 *)heap->micro_malloc( chunkSize * maxChunks );
        init(mData,chunkSize,maxChunks);
        update->addMicroChunk(mData,mDataEnd,this);
    }
    if ( mFreeList )
    {
        mUsedCount++;
        ret = mFreeList;
        mFreeList = mFreeList->mNext;
    }
    return ret;
  }

  inline void deallocate(void *p,MicroHeap *heap,MicroChunkUpdate *update)
  {
#ifdef _DEBUG
    assert(mUsedCount);
    NxU8 *s = (NxU8 *)p;
    assert( s >= mData && s < mDataEnd );
#endif
    MemoryHeader *mh = mFreeList;
    mFreeList = (MemoryHeader *)p;
    mFreeList->mNext = mh;
    mUsedCount--;
    if ( mUsedCount == 0 && mMyHeap  ) // free the heap back to the application if we are done with this.
    {
        update->removeMicroChunk(this);
        heap->micro_free(mData);
        mMyHeap = false;
        mData   = 0;
        mDataEnd = 0;
        mFreeList = 0;
    }
  }

  NxU32 getChunkSize(void) const { return mChunkSize; };

  bool isInside(const NxU8 *p) const
  {
	  return p>=mData && p < mDataEnd; 
  }

private:
  bool          mMyHeap;
  NxU8         *mData;
  NxU8         *mDataEnd;
  NxU32         mUsedCount;
  MemoryHeader *mFreeList;
  NxU32         mChunkSize;
};

#define DEFAULT_CHUNKS 32

class MemoryChunkChunk
{
public:
  MemoryChunkChunk(void)
  {
    mNext = 0;
    mChunkSize = 0;
	mMaxChunks = 0;
  }

  ~MemoryChunkChunk(void)
  {
  }

  inline void * allocate(MemoryChunk *&current,MicroChunkUpdate *update)
  {
    void *ret = 0;

    MemoryChunkChunk *scan = this;
    while ( scan && ret == 0 )
    {
      for (NxU32 i=0; i<DEFAULT_CHUNKS; i++)
      {
          ret = scan->mChunks[i].allocate(mHeap,mChunkSize,mMaxChunks,update);
          if ( ret )
          {
            current = &scan->mChunks[i];
            scan = 0;
            break;
          }
      }
      if ( scan )
        scan = scan->mNext;
    }

    if ( !ret )
    {
        MemoryChunkChunk *mcc = (MemoryChunkChunk *)mHeap->micro_malloc( sizeof(MemoryChunkChunk) );
        new ( mcc ) MemoryChunkChunk;
        MemoryChunkChunk *onext = mNext;
        mNext = mcc;
        mcc->mNext = onext;
        ret = mcc->mChunks[0].allocate(mHeap,mChunkSize,mMaxChunks,update);
        current = &mcc->mChunks[0];
    }

    return ret;
  }

  NxU8 * init(NxU8 *chunkBase,NxU32 fixedSize,NxU32 chunkSize,MemoryChunk *&current,MicroHeap *heap)
  {
    mHeap = heap;
    mChunkSize = chunkSize;
	mMaxChunks = fixedSize/chunkSize;
    current = &mChunks[0];
    chunkBase = mChunks[0].init(chunkBase,chunkSize,mMaxChunks);
    return chunkBase;
  }

  MicroHeap        *mHeap;
  NxU32             mChunkSize;
  NxU32             mMaxChunks;
  MemoryChunkChunk *mNext;
  MemoryChunk       mChunks[DEFAULT_CHUNKS];
};

class FixedMemory
{
public:
  FixedMemory(void)
  {
    mCurrent = 0;
  }

  void * allocate(MicroChunkUpdate *update)
  {
    void *ret = mCurrent->allocate(mChunks.mHeap,mChunks.mChunkSize,mChunks.mMaxChunks,update);
    if ( ret == 0 )
    {
        ret = mChunks.allocate(mCurrent,update);
    }
	return ret;
  }

  NxU8 * init(NxU8 *chunkBase,NxU32 chunkSize,NxU32 fixedSize,MicroHeap *heap)
  {
	mMemBegin = chunkBase;
	mMemEnd   = chunkBase+fixedSize;
    mChunks.init(chunkBase,fixedSize,chunkSize,mCurrent,heap);
	return mMemEnd;
  }

  NxU8            *mMemBegin;
  NxU8            *mMemEnd;
  MemoryChunk     *mCurrent; // the current memory chunk we are operating in.
  MemoryChunkChunk mChunks;  // the collection of all memory chunks used.
};

class MicroChunk
{
public:

  void set(NxU8 *memStart,NxU8 *memEnd,MemoryChunk *mc)
  {
    mMemStart = memStart;
    mMemEnd   = memEnd;
    mChunk    = mc;
    mPad      = 0;
  }

  inline bool inside(const NxU8 *p) const
  {
    return p >= mMemStart && p < mMemEnd;
  }

  NxU8        *mMemStart;
  NxU8        *mMemEnd;
  MemoryChunk *mChunk;
  NxU8        *mPad; // padding to make it 16 byte aligned.
};

class MyMicroAllocator : public MicroAllocator, public MicroChunkUpdate, public MemMutex
{
public:
  MyMicroAllocator(MicroHeap *heap,void *baseMem,NxU32 initialSize,NxU32 chunkSize)
  {
    mLastMicroChunk     = 0;
    mMicroChunks        = 0;
    mMicroChunkCount    = 0;
    mMaxMicroChunks		= 0;
	mHeap               = heap;
	mChunkSize          = chunkSize;

    // 0 through 8 bytes
    for (NxU32 i=0; i<=8; i++)
    {
        mFixedAllocators[i] = &mAlloc[0];
    }

    // 9 through 16 bytes
    for (NxU32 i=9; i<=16; i++)
    {
        mFixedAllocators[i] = &mAlloc[1];
    }

    // 17 through 32 bytes
    for (NxU32 i=17; i<=32; i++)
    {
        mFixedAllocators[i] = &mAlloc[2];
    }

    // 33 through 64
    for (NxU32 i=33; i<=64; i++)
    {
        mFixedAllocators[i] = &mAlloc[3];
    }

    // 65 through 128
    for (NxU32 i=65; i<=128; i++)
    {
        mFixedAllocators[i] = &mAlloc[4];
    }

    // 129 through 255
    for (NxU32 i=129; i<257; i++)
    {
        mFixedAllocators[i] = &mAlloc[5];
    }

    mBaseMem = (NxU8 *)baseMem;
    mBaseMemEnd = mBaseMem+initialSize;

    NxU8 *chunkBase = (NxU8 *)baseMem+sizeof(MyMicroAllocator);


    chunkBase+=32;
	NxU64 ptr = (NxU64)chunkBase;
	ptr = ptr>>4;
	ptr = ptr<<4; // make sure it is 16 byte aligned.
	chunkBase = (NxU8 *)ptr;

    mChunkStart = chunkBase;
    chunkBase = mAlloc[0].init(chunkBase,8,chunkSize,heap);
    chunkBase = mAlloc[1].init(chunkBase,16,chunkSize,heap);
    chunkBase = mAlloc[2].init(chunkBase,32,chunkSize,heap);
    chunkBase = mAlloc[3].init(chunkBase,64,chunkSize,heap);
    chunkBase = mAlloc[4].init(chunkBase,128,chunkSize,heap);
    chunkBase = mAlloc[5].init(chunkBase,256,chunkSize,heap);
	mChunkEnd = chunkBase;

    assert(chunkBase <= mBaseMemEnd );

  }

  ~MyMicroAllocator(void)
  {
    if ( mMicroChunks )
    {
        mHeap->micro_free(mMicroChunks);
    }
  }

  virtual NxU32           getChunkSize(MemoryChunk *chunk) 
  {
	  return chunk ? chunk->getChunkSize() : 0;
  }

  // we have to steal one byte out of every allocation to record the size, so we can efficiently de-allocate it later.
  virtual void * malloc(size_t size)
  {
    void *ret = 0;
	Lock();
   assert( size <= 256 );
   if ( size <= 256 )
   {
     ret = mFixedAllocators[size]->allocate(this);
   }
   Unlock();
    return ret;
  }

  virtual void   free(void *p,MemoryChunk *chunk)
  {
    Lock();
    chunk->deallocate(p,mHeap,this);
	Unlock();
  }

  // perform a binary search on the sorted list of chunks.
  MemoryChunk * binarySearchMicroChunks(const NxU8 *p)
  {
	  MemoryChunk *ret = 0;

	  NxU32 low = 0;
	  NxU32 high = mMicroChunkCount;

	  while ( low != high )
	  {
		  NxU32 mid = (high-low)/2+low;
		  MicroChunk &chunk = mMicroChunks[mid];
		  if ( chunk.inside(p))
		  {
			  mLastMicroChunk = &chunk;
			  ret = chunk.mChunk;
			  break;
		  }
		  else
		  {
			  if ( p > chunk.mMemEnd )
			  {
				  low = mid+1;
			  }
			  else
			  {
				  high = mid;
			  }
		  }
	  }

	  return ret;
  }

  virtual MemoryChunk *   isMicroAlloc(const void *p)  // returns true if this pointer is handled by the micro-allocator.
  {
    MemoryChunk *ret = 0;

	Lock();

    const NxU8 *s = (const NxU8 *)p;

    if ( s >= mChunkStart && s < mChunkEnd )
    {
        NxU32 index = (NxU32)(s-mChunkStart)/mChunkSize;
        assert(index>=0 && index < 6 );
        ret = &mAlloc[index].mChunks.mChunks[0];
		assert( ret->isInside(s) );
    }
	else if ( mMicroChunkCount )
	{
        if ( mLastMicroChunk && mLastMicroChunk->inside(s) )
        {
            ret = mLastMicroChunk->mChunk;
        }
        else
        {
			if ( mMicroChunkCount >= 4 )
			{
				ret = binarySearchMicroChunks(s);
#ifdef _DEBUG
				if (ret )
				{
					assert( ret->isInside(s) );
				}
				else 
				{
					for (NxU32 i=0; i<mMicroChunkCount; i++)
					{
						assert( !mMicroChunks[i].inside(s) );
					}
				}
#endif
			}
			else
			{
				for (NxU32 i=0; i<mMicroChunkCount; i++)
				{
					if ( mMicroChunks[i].inside(s) )
					{
						ret = mMicroChunks[i].mChunk;
						assert( ret->isInside(s) );
						mLastMicroChunk = &mMicroChunks[i];
						break;
					}
				}
			}
        }
	}
#ifdef _DEBUG
	if ( ret )
		assert( ret->isInside(s) );
#endif
	Unlock();
    return ret;
  }

  MicroHeap * getMicroHeap(void) const { return mHeap; };

  void allocateMicroChunks(void)
  {
    if ( mMaxMicroChunks == 0 )
    {
        mMaxMicroChunks = 64; // initial reserve.
        mMicroChunks = (MicroChunk *)mHeap->micro_malloc( sizeof(MicroChunk)*mMaxMicroChunks );
    }
    else
    {
        mMaxMicroChunks*=2;
        mMicroChunks = (MicroChunk *)mHeap->micro_realloc( mMicroChunks, sizeof(MicroChunk)*mMaxMicroChunks);
    }
  }

  // perform an insertion sort of the new chunk.
  virtual void addMicroChunk(NxU8 *memStart,NxU8 *memEnd,MemoryChunk *chunk)
  {
    if ( mMicroChunkCount >= mMaxMicroChunks )
    {
        allocateMicroChunks();
    }

    bool inserted = false;
    for (NxU32 i=0; i<mMicroChunkCount; i++)
    {
        if ( memEnd < mMicroChunks[i].mMemStart )
        {
            for (NxU32 j=mMicroChunkCount; j>i; j--)
            {
                mMicroChunks[j] = mMicroChunks[j-1];
            }
            mMicroChunks[i].set( memStart, memEnd, chunk );
			mLastMicroChunk = &mMicroChunks[i];
            mMicroChunkCount++;
            inserted = true;
            break;
        }
    }
    if ( !inserted )
    {
        mMicroChunks[mMicroChunkCount].set(memStart,memEnd,chunk);
		mLastMicroChunk = &mMicroChunks[mMicroChunkCount];
        mMicroChunkCount++;
    }
  }

  virtual void removeMicroChunk(MemoryChunk *chunk)
  {
    mLastMicroChunk = 0;
    #ifdef _DEBUG
    bool removed = false;
    #endif
    for (NxU32 i=0; i<mMicroChunkCount; i++)
    {
        if ( mMicroChunks[i].mChunk == chunk )
        {
            mMicroChunkCount--;
            for (NxU32 j=i; j<mMicroChunkCount; j++)
            {
                mMicroChunks[j] = mMicroChunks[j+1];
            }
            #ifdef _DEBUG
            removed = true;
            #endif
            break;
        }
    }
#ifdef _DEBUG
    assert(removed);
#endif
  }

  inline void * inline_malloc(size_t size)
  {
     Lock();
     void *ret = mFixedAllocators[size]->allocate(this);
	 Unlock();
	 return ret;
  }

  inline void            inline_free(void *p,MemoryChunk *chunk) // free relative to previously located MemoryChunk
  {
	Lock();
    chunk->deallocate(p,mHeap,this);
	Unlock();
  }

  inline MemoryChunk *   inline_isMicroAlloc(const void *p) // returns pointer to the chunk this memory belongs to, or null if not a micro-allocated block.
  {
    MemoryChunk *ret = 0;

	Lock();

    const NxU8 *s = (const NxU8 *)p;

    if ( s >= mChunkStart && s < mChunkEnd )
    {
        NxU32 index = (NxU32)(s-mChunkStart)/mChunkSize;
        assert(index>=0 && index < 6 );
        ret = &mAlloc[index].mChunks.mChunks[0];
    }
	else if ( mMicroChunkCount )
	{
        if ( mLastMicroChunk && mLastMicroChunk->inside(s) )
        {
            ret = mLastMicroChunk->mChunk;
        }
        else
        {
			if ( mMicroChunkCount >= 4 )
			{
				ret = binarySearchMicroChunks(s);
			}
			else
			{
				for (NxU32 i=0; i<mMicroChunkCount; i++)
				{
					if ( mMicroChunks[i].inside(s) )
					{
						ret = mMicroChunks[i].mChunk;
						mLastMicroChunk = &mMicroChunks[i];
						break;
					}
				}
			}
        }
	}

	Unlock();

    return ret;
  }


private:
  MicroHeap   *mHeap;
  NxU8        *mBaseMem;
  NxU8        *mBaseMemEnd;
  FixedMemory *mFixedAllocators[257];
  NxU32        mChunkSize;
  NxU8        *mChunkStart;
  NxU8        *mChunkEnd;

  NxU32        mMaxMicroChunks;
  NxU32        mMicroChunkCount;
  MicroChunk  *mLastMicroChunk;
  MicroChunk  *mMicroChunks;

  FixedMemory  mAlloc[6];
};


MicroAllocator *createMicroAllocator(MicroHeap *heap,NxU32 chunkSize)
{
    NxU32 initialSize = chunkSize*6+sizeof(MyMicroAllocator)+32;
    void *baseMem = heap->micro_malloc(initialSize);
    MyMicroAllocator *mc = (MyMicroAllocator *)baseMem;
    new ( mc ) MyMicroAllocator(heap,baseMem,initialSize,chunkSize);
    return static_cast< MicroAllocator *>(mc);
}

void releaseMicroAllocator(MicroAllocator *m)
{
    MyMicroAllocator *mc = static_cast< MyMicroAllocator *>(m);
	MicroHeap *mh = mc->getMicroHeap();
    mc->~MyMicroAllocator();
	mh->micro_free(mc);
}

class MyHeapManager : public MicroHeap, public HeapManager
{
public:
  MyHeapManager(NxU32 defaultChunkSize)
  {
    mMicro = createMicroAllocator(this,defaultChunkSize);
  }

  ~MyHeapManager(void)
  {
    releaseMicroAllocator(mMicro);
  }

  // heap allocations used by the micro allocator.
  virtual void * micro_malloc(size_t size)
  {
    return ::malloc(size);
  }

  virtual void   micro_free(void *p)
  {
    return ::free(p);
  }

  virtual void * micro_realloc(void *oldMem,size_t newSize)
  {
    return ::realloc(oldMem,newSize);
  }

  virtual void * heap_malloc(size_t size)
  {
    void *ret;

    if ( size <= 256 ) // micro allocator only handles allocations between 0 and 256 bytes in length.
    {
        ret = mMicro->malloc(size);
    }
    else
    {
        ret = ::malloc(size);
    }
    return ret;
  }

  virtual void   heap_free(void *p)
  {
    MemoryChunk *chunk = mMicro->isMicroAlloc(p);
    if ( chunk )
    {
        mMicro->free(p,chunk);
    }
    else
    {
        ::free(p);
    }
  }

  virtual void * heap_realloc(void *oldMem,size_t newSize)
  {
    void *ret = 0;

    MemoryChunk *chunk = mMicro->isMicroAlloc(oldMem);
    if ( chunk )
    {
        ret = heap_malloc(newSize);
        NxU32 oldSize = chunk->getChunkSize();
        if ( oldSize < newSize )
        {
            memcpy(ret,oldMem,oldSize);
        }
        else
        {
            memcpy(ret,oldMem,newSize);
        }
        mMicro->free(oldMem,chunk);
    }
    else
    {
        ret = ::realloc(oldMem,newSize);
    }

    return ret;
  }

  inline void * inline_heap_malloc(size_t size)
  {
    return size<=256 ? ((MyMicroAllocator *)mMicro)->inline_malloc(size) : ::malloc(size);
  }

  inline void   inline_heap_free(void *p)
  {
	  MemoryChunk *chunk = ((MyMicroAllocator *)mMicro)->inline_isMicroAlloc(p);
	  if ( chunk )
	  {
		  ((MyMicroAllocator *)mMicro)->inline_free(p,chunk);
	  }
	  else
	  {
		  ::free(p);
	  }

  }

private:
  MicroAllocator *mMicro;
};


HeapManager * createHeapManager(NxU32 defaultChunkSize)
{
    MyHeapManager *m = (MyHeapManager *)::malloc(sizeof(MyHeapManager));
    new ( m ) MyHeapManager(defaultChunkSize);
    return static_cast< HeapManager *>(m);
}

void          releaseHeapManager(HeapManager *heap)
{
    MyHeapManager *m = static_cast< MyHeapManager *>(heap);
    m->~MyHeapManager();
	free(m);
}


#define TEST_SIZE 63
#define TEST_ALLOC_COUNT 8192
#define TEST_RUN 40000000
#define TEST_INLINE 1

#ifdef WIN32
#include <windows.h>
#pragma comment(lib,"winmm.lib")
#else
static NxU32 timeGetTime(void)
{
	return 0;
}
#endif
#include <stdio.h>

void performUnitTests(void)
{
    void *allocs[TEST_ALLOC_COUNT];
    for (NxU32 i=0; i<TEST_ALLOC_COUNT; i++)
    {
        allocs[i] = 0;
    }


    HeapManager *hm = createHeapManager(65536*32);


    {

      NxU32 stime = timeGetTime();
      srand(0);


      for (NxU32 i=0; i<TEST_RUN; i++)
      {
          NxU32 index = rand()&(TEST_ALLOC_COUNT-1);
          if ( allocs[index] )
          {
#if TEST_INLINE
              heap_free(hm, allocs[index] );
#else
              hm->heap_free( allocs[index] );
#endif
              allocs[index] = 0;
          }
          else
          {
              NxU32 asize = (rand()&TEST_SIZE);
			  if ( (rand()&127)==0) asize+=256; // one out of every 15 allocs is larger than 256 bytes.
#if TEST_INLINE
              allocs[index] = heap_malloc(hm,asize);
#else
              allocs[index] = hm->heap_malloc(asize);
#endif
          }
      }

      for (NxU32 i=0; i<TEST_ALLOC_COUNT; i++)
      {
          if ( allocs[i] )
          {
#if TEST_INLINE
              heap_free(hm,allocs[i] );
#else
              hm->heap_free(allocs[i] );
#endif
			  allocs[i] = 0;
          }
      }

      NxU32 etime = timeGetTime();
      printf("Micro allocation test took %d milliseconds.\r\n", etime - stime );

    }

    {

      NxU32 stime = timeGetTime();
      srand(0);


      for (NxU32 i=0; i<TEST_RUN; i++)
      {
          NxU32 index = rand()&(TEST_ALLOC_COUNT-1);
          if ( allocs[index] )
          {
              ::free( allocs[index] );
              allocs[index] = 0;
          }
          else
          {
              NxU32 asize = (rand()&TEST_SIZE);
			  if ( (rand()&127)==0) asize+=256; // one out of every 15 allocs is larger than 256 bytes.
              allocs[index] = ::malloc(asize);
          }
      }

      for (NxU32 i=0; i<TEST_ALLOC_COUNT; i++)
      {
          if ( allocs[i] )
          {
              ::free(allocs[i] );
			  allocs[i] = 0;
          }
      }

      NxU32 etime = timeGetTime();
      printf("Standard malloc/free test took %d milliseconds.\r\n", etime - stime );

    }

    releaseHeapManager(hm);
}

void * heap_malloc(HeapManager *hm,size_t size)
{
    return ((MyHeapManager *)hm)->inline_heap_malloc(size);
}

void   heap_free(HeapManager *hm,void *p)
{
    ((MyHeapManager *)hm)->inline_heap_free(p);
}

void * heap_realloc(HeapManager *hm,void *oldMem,size_t newSize)
{
    return hm->heap_realloc(oldMem,newSize);
}

}; // end of namespace
