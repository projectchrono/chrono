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
// Authors: Radu Serban
// =============================================================================

#ifndef CHTYPES_H
#define CHTYPES_H

#include <memory>
#include <type_traits>

/// Namespace for custom make_shared implementation.
namespace chrono_types {

// Adapted from MRtrix3 (https://github.com/MRtrix3)

/// Check if a class has a custom new operator.
/// Classes that have fixed-sized Eigen matrices as members, use an Eigen-provided override of operator new.
template <class T>
class class_has_custom_new_operator {
    template <typename C>
    static inline char test(decltype(C::operator new(sizeof(C))));
    template <typename C>
    static inline long test(...);

  public:
    enum { value = sizeof(test<T>(nullptr)) == sizeof(char) };
};

// -----------------------------------------------------------------------------

/// Replacement for make_shared guaranteed to use `operator new` rather than `placement new` in order to avoid memory
/// alignment issues (classes with members that are fixed-sized Eigen matrices have overriden operator new).
///
/// Dynamic objects of classes with fixed-size vectorizable Eigen object members\n
/// - Many of the Chrono classes now have members that are fixed-size vectorizable Eigen types. These classes overload
///   their `operator new` to generate 16-byte-aligned pointers (using an Eigen - provided macro).
/// - This requirement for aligned memory allocation has implications on creation of shared pointers.  Indeed,
///   `std::make_shared` uses `placement new` instead of `operator new`.  To address this issue and preserve
///   encapsulation (as much as possible), Chrono provides this for `make_shared`. This function will automatically
///   infer if it can safely fallback on `std::make_shared` or else create a shared pointer with a mechanism that
///   ensures use of aligned memory. As such, user code should **always** use `chrono_types::make_shared` as in
///   <pre>
///   auto my_body = chrono_types::make_shared<ChBody>();
///   </pre>

////C++11 version
////template <typename T,
////          typename... Args,
////          typename std::enable_if<class_has_custom_new_operator<T>::value, int>::type = 0>
////inline std::shared_ptr<T> make_shared(Args&&... args) {
////    return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
////}

// C++14 version
template <typename T, typename... Args, std::enable_if_t<class_has_custom_new_operator<T>::value, int> = 0>
inline std::shared_ptr<T> make_shared(Args&&... args) {
    return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
}

/// For classes without members that are fixed-sized Eigen matrices (and hence do not have an overriden operator new),
/// it is safe to default to using `std::make_shared` (no alignment issues).

////C++11 version
////template <typename T,
////          typename... Args,
////          typename std::enable_if<!class_has_custom_new_operator<T>::value, int>::type = 0>
////inline std::shared_ptr<T> make_shared(Args&&... args) {
////    return std::make_shared<T>(std::forward<Args>(args)...);
////}

// C++14 version
template <typename T, typename... Args, std::enable_if_t<!class_has_custom_new_operator<T>::value, int> = 0>
inline std::shared_ptr<T> make_shared(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

// -----------------------------------------------------------------------------

/// Replacement for make_unique guaranteed to use `operator new` rather than `placement new` in order to avoid memory
/// alignment issues (classes with members that are fixed-sized Eigen matrices have overriden operator new).

//// Since std::make_unique was only introduced in C++14, the C++11 implementation of chrono_types::make_unique
//// does not distinguish between classes with and without fixed-size Eigen members.
////template <typename T, typename... Args>
////inline std::unique_ptr<T> make_unique(Args&&... args) {
////    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
////}

// C++14 version - make_unique for classes with overriden operator new.
template <typename T, typename... Args, std::enable_if_t<class_has_custom_new_operator<T>::value, int> = 0>
inline std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// C++14 version - make_unique for classes with no overriden operator new.
template <typename T, typename... Args, std::enable_if_t<!class_has_custom_new_operator<T>::value, int> = 0>
inline std::unique_ptr<T> make_unique(Args&&... args) {
    return std::make_unique<T>(std::forward<Args>(args)...);
}

}  // end namespace chrono_types

#endif
