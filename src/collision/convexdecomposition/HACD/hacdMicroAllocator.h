#ifndef MICRO_ALLOCATOR_H

#define MICRO_ALLOCATOR_H

/*!
**
** Copyright (c) 2009 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
**
** If you find this code useful or you are feeling particularily generous I would
** ask that you please go to http://www.amillionpixels.us and make a donation
** to Troy DeMolay.
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


// This code snippet provides a high speed micro-allocator.
//
// The concept is that you reserve an initial bank of memory for small allocations.  Ideally a megabyte or two.
// The amount of memory reserved is equal to chunkSize*6
//
// All micro-allocations are split into 6 seperate pools.
//    They are: 0-8 bytes
//              9-32 bytes
//              33-64 bytes
//              65-128 bytes
//              129-256 bytes
//
// On creation of the micro-allocation system you preserve a certiain amount of memory for each of these banks.
//
// The user provides a heap interface to callback for additional memory as needed.
//
// In most cases allocations are order-N and frees are order-N as well.
//
// The larger a buffer you provide, the closer to 'order-N' the allocator behaves.
//
// This kind of a micro-allocator is ideal for use with STL as it does many tiny allocations.
// All allocations are 16 byte aligned (with the exception of the 8 byte allocations, which are 8 byte aligned every other one).
//
#include <stdio.h>

#ifdef WIN32
	typedef __int64				NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned __int64	NxU64;
	typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;
	typedef unsigned char		NxU8;

	typedef float				NxF32;
	typedef double				NxF64;

#elif __gnu_linux__
	typedef long long			NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned long long	NxU64;
    typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;
	typedef unsigned char		NxU8;

	typedef float				NxF32;
	typedef double				NxF64;

#elif __APPLE__
	typedef long long			NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned long long	NxU64;
	typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;
	typedef unsigned char		NxU8;

	typedef float				NxF32;
	typedef double				NxF64;

#elif __CELLOS_LV2__
	typedef long long			NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned long long	NxU64;
	typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;
	typedef unsigned char		NxU8;

	typedef float				NxF32;
	typedef double				NxF64;

#elif _XBOX
	typedef __int64				NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned __int64	NxU64;
	typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;
	typedef unsigned char		NxU8;

	typedef float				NxF32;
	typedef double				NxF64;

#elif defined(__PPCGEKKO__)
	typedef long long			NxI64;
	typedef signed int			NxI32;
	typedef signed short		NxI16;
	typedef signed char			NxI8;

	typedef unsigned long long	NxU64;
	typedef unsigned int		NxU32;
	typedef unsigned short		NxU16;

	typedef float				NxF32;
	typedef double				NxF64;

#else
	#error Unknown platform!
#endif


namespace HACD
{

// user provided heap allocator
class MicroHeap
{
public:
  virtual void * micro_malloc(size_t size) = 0;
  virtual void   micro_free(void *p) = 0;
  virtual void * micro_realloc(void *oldMen,size_t newSize) = 0;
};

class MemoryChunk;

class MicroAllocator
{
public:
  virtual void *          malloc(size_t size) = 0;
  virtual void            free(void *p,MemoryChunk *chunk) = 0; // free relative to previously located MemoryChunk
  virtual MemoryChunk *   isMicroAlloc(const void *p) = 0; // returns pointer to the chunk this memory belongs to, or null if not a micro-allocated block.
  virtual NxU32           getChunkSize(MemoryChunk *chunk) = 0;
};

MicroAllocator *createMicroAllocator(MicroHeap *heap,NxU32 chunkSize=32768); // initial chunk size 32k per block.
void            releaseMicroAllocator(MicroAllocator *m);

class HeapManager
{
public:
  virtual void * heap_malloc(size_t size) = 0;
  virtual void   heap_free(void *p) = 0;
  virtual void * heap_realloc(void *oldMem,size_t newSize) = 0;
};


// creates a heap manager that uses micro-allocations for all allocations < 256 bytes and standard malloc/free for anything larger.
HeapManager * createHeapManager(NxU32 defaultChunkSize=32768);
void          releaseHeapManager(HeapManager *heap);

// about 10% faster than using the virtual interface, inlines the functions as much as possible.
void * heap_malloc(HeapManager *hm,size_t size);
void   heap_free(HeapManager *hm,void *p);
void * heap_realloc(HeapManager *hm,void *oldMem,size_t newSize);

void performUnitTests(void);

//

}; // end of namespace

#endif
