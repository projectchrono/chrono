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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

