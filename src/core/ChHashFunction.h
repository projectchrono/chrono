//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHHASHFUNCTION__H
#define CHHASHFUNCTION__H

//////////////////////////////////////////////////
//
//   ChHashFunction.h
//
//   Basic hash function to be used with the 
//   ChHashTable class. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <string>
#include "core/ChLog.h"


namespace chrono
{

// Note.
// Hash functions to be used with the ChHashTable do NOT need to
// return the result wrapped in the 0..maxValue range (where maxValue
// is the number of bins in table) because this will be automatically done 
// by the ChHashTable.



/// A very simple hash function for integer keys and such..
///

template <class K>
struct HashFunction_Generic
{
	unsigned operator()(const K key) const 
	{ 
		return key * 0xf4243;  // no need to wrap with ... % maxValue;
	}
};


/// Specialized hash function for strings
///

struct HashFunction_String1
{
	unsigned operator()(const std::string& key) const 
	{ 
		unsigned h = 0;
		for (unsigned int i = 0; i < key.length(); ++i)
		{
			h = h * 0xf4243 ^ key[i];
		}
		return h;
	}
};



/// Specialized hash function for strings (other version)
///

struct HashFunction_String2
{
	unsigned operator()(const std::string& key) const 
	{ 
		unsigned int n = key.length();
		const char* d = key.c_str();
		unsigned h = 0; 
	      
		for (unsigned int i = 0; i < n; ++i, ++d)
			  h = (h << 2) + *d;

		return ((h >= 0) ? (h ) : (-h ));
	}
};



/// Specialized hash function for strings (other version)
///

struct HashFunction_String3
{
	unsigned operator()(const std::string& key) const 
	{ 
		int n = key.length();
		const char* d = key.c_str();
		long h = n; 
      
		for (int i = 0; i < n; ++i, ++d)
			h = 613*h + *d;

		return ((h >= 0) ? (h ) : (-h )); 
	}
};






} // END_OF_NAMESPACE____




#endif  // END of ChHashFunction.h
