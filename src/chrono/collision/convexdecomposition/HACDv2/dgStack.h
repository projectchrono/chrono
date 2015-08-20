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

#ifndef __dgStack__
#define __dgStack__

#include "dgTypes.h"

class dgStackBase
{
	protected:
	dgStackBase (hacd::HaI32 size);
	~dgStackBase ();

	const void *m_ptr;
};

inline dgStackBase::dgStackBase (hacd::HaI32 size)
{
	m_ptr = HACD_ALLOC(size_t (size));
}

inline dgStackBase::~dgStackBase ()
{
	HACD_FREE((void*)m_ptr);
}




template<class T>
class dgStack: public dgStackBase
{
	public:
	dgStack (hacd::HaI32 size);
	~dgStack ();
	hacd::HaI32 GetSizeInBytes() const;
	hacd::HaI32 GetElementsCount() const;
	
	T& operator[] (hacd::HaI32 entry);
	const T& operator[] (hacd::HaI32 entry) const;

	private:
	hacd::HaI32 m_size;
};

template<class T>
dgStack<T>::dgStack (hacd::HaI32 size)
	:dgStackBase (hacd::HaI32 (size * sizeof(T)))
{
	m_size = size;
}

template<class T>
dgStack<T>::~dgStack ()
{
}

template<class T>
hacd::HaI32 dgStack<T>::GetElementsCount() const
{
	return m_size;
}

template<class T>
hacd::HaI32 dgStack<T>::GetSizeInBytes() const
{
	return hacd::HaI32 (m_size * sizeof(T));
}
 

template<class T>
T& dgStack<T>::operator[] (hacd::HaI32 entry) 
{
	T *mem;

	HACD_ASSERT (entry >= 0);
	HACD_ASSERT ((entry < m_size) || ((m_size == 0) && (entry == 0)));

	mem = (T*) m_ptr;
	return mem[entry];
}

template<class T>
const T& dgStack<T>::operator[] (hacd::HaI32 entry) const
{
	T *mem;

	HACD_ASSERT (0);
	HACD_ASSERT (entry >= 0);
	HACD_ASSERT ((entry < m_size) || ((m_size == 0) && (entry == 0)));

	mem = (T*) m_ptr;
	return mem[entry];
}


#endif

