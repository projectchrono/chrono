/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_ALIGNED_ALLOCATOR
#define BT_ALIGNED_ALLOCATOR

///we probably replace this with our own aligned memory allocator
///so we replace _aligned_malloc and _aligned_free with our own
///that is better portable and more predictable

#include "cbtScalar.h"

///BT_DEBUG_MEMORY_ALLOCATIONS preprocessor can be set in build system
///for regression tests to detect memory leaks
///#define BT_DEBUG_MEMORY_ALLOCATIONS 1
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS

int cbtDumpMemoryLeaks();

#define cbtAlignedAlloc(a, b) \
	cbtAlignedAllocInternal(a, b, __LINE__, __FILE__)

#define cbtAlignedFree(ptr) \
	cbtAlignedFreeInternal(ptr, __LINE__, __FILE__)

void* cbtAlignedAllocInternal(size_t size, int alignment, int line, char* filename);

void cbtAlignedFreeInternal(void* ptr, int line, char* filename);

#else
void* cbtAlignedAllocInternal(size_t size, int alignment);
void cbtAlignedFreeInternal(void* ptr);

#define cbtAlignedAlloc(size, alignment) cbtAlignedAllocInternal(size, alignment)
#define cbtAlignedFree(ptr) cbtAlignedFreeInternal(ptr)

#endif
typedef int size_type;

typedef void*(cbtAlignedAllocFunc)(size_t size, int alignment);
typedef void(cbtAlignedFreeFunc)(void* memblock);
typedef void*(cbtAllocFunc)(size_t size);
typedef void(cbtFreeFunc)(void* memblock);

///The developer can let all Bullet memory allocations go through a custom memory allocator, using cbtAlignedAllocSetCustom
void cbtAlignedAllocSetCustom(cbtAllocFunc* allocFunc, cbtFreeFunc* freeFunc);
///If the developer has already an custom aligned allocator, then cbtAlignedAllocSetCustomAligned can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, and instruments it.
void cbtAlignedAllocSetCustomAligned(cbtAlignedAllocFunc* allocFunc, cbtAlignedFreeFunc* freeFunc);

///The cbtAlignedAllocator is a portable class for aligned memory allocations.
///Default implementations for unaligned and aligned allocations can be overridden by a custom allocator using cbtAlignedAllocSetCustom and cbtAlignedAllocSetCustomAligned.
template <typename T, unsigned Alignment>
class cbtAlignedAllocator
{
	typedef cbtAlignedAllocator<T, Alignment> self_type;

public:
	//just going down a list:
	cbtAlignedAllocator() {}
	/*
	cbtAlignedAllocator( const self_type & ) {}
	*/

	template <typename Other>
	cbtAlignedAllocator(const cbtAlignedAllocator<Other, Alignment>&)
	{
	}

	typedef const T* const_pointer;
	typedef const T& const_reference;
	typedef T* pointer;
	typedef T& reference;
	typedef T value_type;

	pointer address(reference ref) const { return &ref; }
	const_pointer address(const_reference ref) const { return &ref; }
	pointer allocate(size_type n, const_pointer* hint = 0)
	{
		(void)hint;
		return reinterpret_cast<pointer>(cbtAlignedAlloc(sizeof(value_type) * n, Alignment));
	}
	void construct(pointer ptr, const value_type& value) { new (ptr) value_type(value); }
	void deallocate(pointer ptr)
	{
		cbtAlignedFree(reinterpret_cast<void*>(ptr));
	}
	void destroy(pointer ptr) { ptr->~value_type(); }

	template <typename O>
	struct rebind
	{
		typedef cbtAlignedAllocator<O, Alignment> other;
	};
	template <typename O>
	self_type& operator=(const cbtAlignedAllocator<O, Alignment>&)
	{
		return *this;
	}

	friend bool operator==(const self_type&, const self_type&) { return true; }
};

#endif  //BT_ALIGNED_ALLOCATOR
