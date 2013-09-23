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

#ifndef CHWRAPHASHMAP_H
#define CHWRAPHASHMAP_H

//////////////////////////////////////////////////
// 
//   ChWrapHashmap.h
//
//   This header can be used to include the STL 
//   hash_map implementation, in a compiler-independent
//   way.
//   In fact, an annoying thing is that some STL 
//   distributions (ex. in GNU C++) must include
//   the #include<ext/hash_map>, other must do
//   #include<hash_map>, maybe with different namespaces.
//   To avoid this problem you can just use
//   #include<ChWrapHashmap.h>.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#ifdef COMPILER_GCC
 #include <ext/hash_map>
 namespace chronohash = ::__gnu_cxx;
#else
 #include <hash_map>
 namespace chronohash = ::stdext; // NOTE: in Visual C++ Toolkit 2003 is ::std;
#endif



#endif

