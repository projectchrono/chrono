#ifndef SPARSE_ARRAY_H

#define SPARSE_ARRAY_H

#include "PlatformConfigHACD.h"

using namespace hacd;

/*!
**
** Copyright (c) 20011 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
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

// This class implements a sparse array.
// You must know the maximum number of actual elements you will ever have in your array at creation time.
// Meaning it does not support 'growing' the number of active elements.
// The size of the array can be any array indexes up to 32 bits.
//


template < class Value,	size_t hashTableSize = 512 >		// *MUST* be a power of 2!
class SparseArray : public UANS::UserAllocated
{
public:
	SparseArray(HaU32 maxEntries)
	{
		mFreeList = NULL;
		mHashTableCount = 0;
		for (size_t i = 0; i < hashTableSize; i++)
		{
			mHashTable[i] = NULL;
		}
		mMaxEntries = maxEntries;
		mEntries = HACD_NEW(HashEntry)[maxEntries];
	}
	~SparseArray(void)
	{
		delete []mEntries;
	}

	Value& operator[] (HaU32 key)
	{
		Value *v = find(key);
		if ( v == NULL )
		{
			Value dummy;
			insert(key,dummy);
			v = find(key);
			HACD_ASSERT(v);
		}
		return *v;
	}

	const Value& operator[](HaU32 key) const
	{
		Value *v = find(key);
		if ( v == NULL )
		{
			Value dummy;
			insert(key,dummy);
			v = find(key);
			HACD_ASSERT(v);
		}
		return *v;
	}


	Value* find(HaU32 key)  const
	{
		Value* ret = NULL;
		HaU32 hash = getHash(key);
		HashEntry* h = mHashTable[hash];
		while (h)
		{
			if (h->mKey == key)
			{
				ret = &h->mValue;
				break;
			}
			h = h->mNext;
		}
		return ret;
	}

	void erase(HaU32 key)
	{
		HaU32 hash = getHash(key);
		HashEntry* h = mHashTable[hash];
		HashEntry *prev = NULL;
		while (h)
		{
			if (h->mKey == key)
			{
				if ( prev )
				{
					prev->mNext = h->mNext; // if there was a previous, then it's next is our old next
				}
				else
				{
					mHashTable[hash] = h->mNext; // if there was no previous than the new head of the list is our next.
				}
				// add this hash entry to the free list.
				HashEntry *oldFreeList = mFreeList;
				mFreeList = h;
				h->mNext = oldFreeList;
				break;
			}
			prev = h;
			h = h->mNext;
		}
	}

	void insert(HaU32 key, const Value& value)
	{
		if (mHashTableCount < mMaxEntries )
		{
			HashEntry* h;
			if ( mFreeList ) // if there are previously freed hash entry items
			{
				h = mFreeList;
				mFreeList = h->mNext;
				h->mNext = NULL;
			}
			else
			{
				h = &mEntries[mHashTableCount];
				mHashTableCount++;
				HACD_ASSERT( mHashTableCount < mMaxEntries );
			}

			h->mKey = key;
			h->mValue = value;
			HaU32 hash = getHash(key);
			if (mHashTable[hash])
			{
				HashEntry* next = mHashTable[hash];
				mHashTable[hash] = h;
				h->mNext = next;
			}
			else
			{
				mHashTable[hash] = h;
			}
		}
	}
private:

	// Thomas Wang's 32 bit mix
	// http://www.cris.com/~Ttwang/tech/inthash.htm
	HACD_INLINE HaU32 hash(const HaU32 key) const
	{
		HaU32 k = key;
		k += ~(k << 15);
		k ^= (k >> 10);
		k += (k << 3);
		k ^= (k >> 6);
		k += ~(k << 11);
		k ^= (k >> 16);
		return (HaU32)k;
	}

	HACD_INLINE HaU32 getHash(HaU32 key) const
	{
		HaU32 ret = hash(key);
		return ret & (hashTableSize - 1);
	}

	class HashEntry : public UANS::UserAllocated
	{
	public:
		HashEntry(void)
		{
			mNext = NULL;
		}
		HashEntry*	mNext;
		HaU32		mKey;
		Value		mValue;
	};

	HashEntry*		mFreeList;
	HashEntry*		mHashTable[hashTableSize];
	unsigned int	mHashTableCount;
	HaU32			mMaxEntries;
	HashEntry		*mEntries;

};

#endif
