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

/// Replacement for make_shared guaranteed to use 'operator new' rather than 'placement new' in order to avoid memory
/// alignment issues (classes with members that are fixed-sized Eigen matrices have overriden operator new).
template <typename T, typename... Args, typename std::enable_if<class_has_custom_new_operator<T>::value, int>::type = 0>
inline std::shared_ptr<T> make_shared(Args&&... args) {
    return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
}

// C++14 version
////template <typename T, typename... Args, std::enable_if_t<class_has_custom_new_operator<T>::value, int> = 0>
////inline std::shared_ptr<T> make_shared(Args&&... args) {
////    return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
////}

/// For classes without members that are fixed-sized Eigen matrices (and hence do not have an overriden operator new),
/// it is safe to default to using std::make_shared (no alignment issues).
template <typename T,
          typename... Args,
          typename std::enable_if<!class_has_custom_new_operator<T>::value, int>::type = 0>
inline std::shared_ptr<T> make_shared(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

// C++14 version
////template <typename T, typename... Args, std::enable_if_t<!class_has_custom_new_operator<T>::value, int> = 0>
////inline std::shared_ptr<T> make_shared(Args&&... args) {
////    return std::make_shared<T>(std::forward<Args>(args)...);
////}

}  // end namespace chrono_types

#endif
