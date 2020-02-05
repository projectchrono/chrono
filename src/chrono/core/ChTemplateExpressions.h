// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHTEMPLATEEXPRESSION_H
#define CHTEMPLATEEXPRESSION_H


#include "chrono/core/ChApiCE.h"


namespace chrono {


/// Macro to exploit SFINAE in templates. This builds a compile-time function detector.
/// The detector can be used to conditionally use specialized templates, depending on the
/// fact that the T class has amember function or not. Use ::value to check the result, 0 or 1.
/// Works also if T is not a class, ex. double, int, float; if so, obviously evaluates to 0.
/// When using CH_CREATE_MEMBER_DETECTOR(foo) a detector ChDetect_foo is generated.
/// Example:
///
///       CH_CREATE_MEMBER_DETECTOR(DoSomething)
///
///       // this is used if T has a 'DoSomething()' member function...
///       template<class T, typename enable_if< ChDetect_DoSomething<T>::value, T>::type* = nullptr >
///       void myfunct     (T val) {
///          val.DoSomething();
///       }
///       // this is used otherwise:
///       template<class T, typename enable_if< !ChDetect_DoSomething<T>::value, T>::type* = nullptr >
///       void myfunct     (T val) {
///          ...
///       }
///
/// then call as myfunct(myvalue);


#define CH_CREATE_MEMBER_DETECTOR(X)                                                \
template <typename T, typename Enable = void>                                       \
class ChDetect_##X;                                                                 \
template<typename T>                                                                \
class ChDetect_##X <T, typename std::enable_if<std::is_class<T>::value>::type> {    \
    struct Fallback { int X; };                                                     \
    struct Derived : T, Fallback { };                                               \
                                                                                    \
    template<typename U, U> struct Check;                                           \
                                                                                    \
    typedef char ArrayOfOne[1];                                                     \
    typedef char ArrayOfTwo[2];                                                     \
                                                                                    \
    template<typename U> static ArrayOfOne & func(Check<int Fallback::*, &U::X> *); \
    template<typename U> static ArrayOfTwo & func(...);                             \
  public:                                                                           \
    typedef ChDetect_##X type;                                                      \
    enum { value = sizeof(func<Derived>(0)) == 2 };                                 \
};                                                                                  \
template<typename T>                                                                \
class ChDetect_##X <T, typename std::enable_if<!std::is_class<T>::value>::type> {   \
  public:                                                                           \
    typedef ChDetect_##X type;                                                      \
    enum { value = 0 };                                                             \
}; 


template<bool B, class T = void>
struct enable_if {};
 
template<class T>
struct enable_if<true, T> { typedef T type; };

}  // end namespace chrono

#endif