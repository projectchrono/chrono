// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHBITMASKENUMS_H
#define CHBITMASKENUMS_H

// This header contains a macro that allows using C++11 enum classes (aka strongly typed
// enums) as type-safe bitmasks.
// - It is a more modern way to deal with bit flags, respect to old C-style
//   of using many #define FLAG_A 0x1   #define FLAG_B 0x2  etc., 
// - It is a safer way, i.e. type safe, respect to use plain C++ enums as in:
//     enum myflags {flag_a=0x1, flag_b=0x2};
//
// Use the macro CH_ENABLE_BITMASK_OPERATORS(myflags) as in this example:
//
// enum class Permissions   {
//    Readable   = 0x4,
//    Writeable  = 0x2
// };
// CH_ENABLE_BITMASK_OPERATORS(Permissions)
// Permissions p = Permissions::Readable | Permissions::Writable;  
//
// This code snipped is inspired by the work in 
// https://www.justsoftwaresolutions.co.uk/cplusplus/using-enum-classes-as-bitfields.html
// http://blog.bitwigglers.org/using-enum-classes-as-type-safe-bitmasks/
//

#include<type_traits>


/// Use the macro CH_ENABLE_BITMASK_OPERATORS(myflags) as in this example:
///
/// enum class Permissions  
/// {
///    Readable   = 0x4,
///    Writeable  = 0x2,
///    Executable = 0x1
/// };
/// CH_ENABLE_BITMASK_OPERATORS(Permissions)
///
/// Permissions p = Permissions::Readable | Permissions::Writable;  
/// p |= Permissions::Executable;  
/// p &= ~Permissions::Writable;

#define CH_ENABLE_BITMASK_OPERATORS(menum)      \
 template<>                                     \
 struct Ch_enable_bitmask_operators<menum>  {   \
     static const bool enable = true;           \
 };



namespace chrono { 


/// Helper for adding bitmask operators | ^ & etc. to strongly typed
/// enum classes. 
/// Use it as in this example:
///
/// enum class Permissions   {
///    Readable   = 0x4,
///    Writeable  = 0x2
/// };
/// template<>
/// struct Ch_enable_bitmask_operators<Permissions>{
///     static constexpr bool enable=true;
/// };
/// Permissions p = Permissions::Readable | Permissions::Writable;
///
/// Alternative: better use CH_ENABLE_BITMASK_OPERATORS macro, it is less typing.

template<typename E>
struct Ch_enable_bitmask_operators{
    static const bool enable=false;
};


//
// Templated operators for automatic handling of enum classes (aka strongly typed
// enums) as type-safe bitmasks
//

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E>::type
operator|(E lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
        static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E>::type
operator&(E lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
        static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E>::type
operator^(E lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
        static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs));
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E>::type
operator~(E lhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
        ~static_cast<underlying>(lhs));
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E&>::type
operator|=(E& lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs=static_cast<E>(
        static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
    return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E&>::type
operator&=(E& lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs=static_cast<E>(
        static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
    return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable,E&>::type
operator^=(E& lhs,E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs=static_cast<E>(
        static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs));
    return lhs;
}

}  // end namespace chrono

#endif
