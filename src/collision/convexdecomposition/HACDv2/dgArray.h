/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#ifndef __dgArray__
#define __dgArray__

#include "dgTypes.h"

template<class T>
class dgArray
{
	public:
	dgArray (hacd::HaI32 granulatitySize,hacd::HaI32 aligmentInBytes = DG_MEMORY_GRANULARITY);
	~dgArray ();


	T& operator[] (hacd::HaI32 i);
	const T& operator[] (hacd::HaI32 i) const;
	void Resize (hacd::HaI32 size) const;
	hacd::HaI32 GetBytesCapacity () const;
	hacd::HaI32 GetElementsCapacity () const; 

	bool ExpandCapacityIfNeessesary (hacd::HaI32 index, hacd::HaI32 stride) const;

	private:
	hacd::HaI32 m_granulatity;
	hacd::HaI32 m_aligmentInByte;
	mutable hacd::HaI32 m_maxSize;
	mutable T *m_array;
};


template<class T>
dgArray<T>::dgArray (hacd::HaI32 granulatitySize,hacd::HaI32 aligmentInBytes)
 :m_granulatity(granulatitySize), m_aligmentInByte(aligmentInBytes), m_maxSize(0), m_array(NULL)
{
	if (m_aligmentInByte <= 0) {
		m_aligmentInByte = 8;
	}
	m_aligmentInByte = 1 << exp_2(m_aligmentInByte);
}

template<class T>
dgArray<T>::~dgArray ()
{
	if (m_array) {
		HACD_FREE(m_array);
	}
}


template<class T>
const T& dgArray<T>::operator[] (hacd::HaI32 i) const
{ 
	HACD_ASSERT (i >= 0);
	while (i >= m_maxSize) {
		Resize (i);
	}
	return m_array[i];
}


template<class T>
T& dgArray<T>::operator[] (hacd::HaI32 i)
{
	HACD_ASSERT (i >= 0);
	while (i >= m_maxSize) {
		Resize (i);
	}
	return m_array[i];
}

template<class T>
hacd::HaI32 dgArray<T>::GetElementsCapacity () const
{
	return m_maxSize;
}


template<class T>
hacd::HaI32 dgArray<T>::GetBytesCapacity () const
{
	return  m_maxSize * hacd::HaI32 (sizeof (T));
}


template<class T>
void dgArray<T>::Resize (hacd::HaI32 size) const
{
	if (size >= m_maxSize) {
		size = size + m_granulatity - (size + m_granulatity) % m_granulatity;
		T* const newArray = (T*) HACD_ALLOC_ALIGNED(hacd::HaI32 (sizeof (T) * size), m_aligmentInByte);
		if (m_array) {
			for (hacd::HaI32 i = 0; i < m_maxSize; i ++) {
				newArray[i]	= m_array[i];
			}
			HACD_FREE(m_array);
		}
		m_array = newArray;
		m_maxSize = size;
	} else if (size < m_maxSize) {
		size = size + m_granulatity - (size + m_granulatity) % m_granulatity;
		T* const newArray = (T*) HACD_ALLOC_ALIGNED(hacd::HaI32 (sizeof (T) * size), m_aligmentInByte);
		if (m_array) {
			for (hacd::HaI32 i = 0; i < size; i ++) {
				newArray[i]	= m_array[i];
			}
			HACD_FREE(m_array);
		}
		m_array = newArray;
		m_maxSize = size;
	}
}

template<class T>
bool dgArray<T>::ExpandCapacityIfNeessesary (hacd::HaI32 index, hacd::HaI32 stride) const
{
	bool ret = false;
	hacd::HaI32 size = (index + 1) * stride;
	while (size >= m_maxSize) {
		ret = true;
		Resize (m_maxSize * 2);
	}
	return ret;
}

#endif




