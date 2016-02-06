//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSMARTPOINTERS_H
#define CHSMARTPOINTERS_H

//////////////////////////////////////////////////
//
//   ChSmartpointers.h
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdexcept>

namespace chrono {


/////////////////////////////////////////////////////////////////////

/// [See nice article on shared pointers at:
/// http://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one.
/// See also the chrono demo demo_sharedptr.cpp]
/// The class ChSharedPtr wraps a 'bare' C++ pointer in order
/// to manage the lifetime of the object being pointed to by the
/// bare pointer. With 'bare' C++ pointers, the programmer has to
/// explicitly destroy the object when it is no longer useful.
/// A ChSharedPtr object by comparison defines a policy
/// as to when the object pointed to by the bare pointer is
/// to be destroyed. You still have to create the object, but
/// you no longer have to worry about destroying it.
/// It should be initialized in one of the following two ways:
///    ChSharedPtr<MyClass> foo_pointer(new MyClass);
///       or
///    ChSharedPtr<MyClass> foo_pointer(another_shared_pointer);
/// In doing so, you will _never_ need to call 'delete', because
/// the reference count mechanism will automatically delete the
/// 'bare' pointer when no other smart pointer reference it.
/// NOTE1: When copied, a ChSharedPtr object does not perform copy
/// of the data pointed to by the 'bare' pointer.


template <class T>
class ChSharedPtr {
    // Make all ChSharedPtr as friend classes, so that they can access each other ptr quickly, in casting
    template <typename TT>
    friend class ChSharedPtr;

  public:
    typedef T element_type;

    /// Constructor for initializing with dynamically allocated data, with new()
    /// Mandatory way of using it:
    ///   ChSharedPtr<MyClass> pointerA(new MyClass);
    /// Thank to automatic reference counting, you never have to call delete()!
    explicit ChSharedPtr(T* p = 0) { ptr = p; }

    /// Copy constructor for the case
    ///   ChSharedPtr<MyClassA> pointerA(pointerB);
    ChSharedPtr(const ChSharedPtr& r) throw() { acquire(r); }

    /// Copy constructor and converter for the case
    ///  ChSharedPtr<MyClassA> pointerA(pointerB);
    /// when pointerB type is a class MyClassB which is inherited from MyClassA.
    template <class T_other>
    ChSharedPtr(const ChSharedPtr<T_other>& r) throw() {
        acquire(r);
    }

    /// Destructor decrements the reference count and automatically delete only
    /// when the last reference is destroyed
    ~ChSharedPtr() { release(); }

    /// Bool type casting, true if the pointer is still bound to an object, or
    /// false if unbound and invalidated (ex. after unsuccesfull casting). Example:
    ///    if(mysharedptr) {...}
    // Trick to avoid problems as in  http://www.artima.com/cppsource/safebool2.html
    // In future C++0x will be simply:
    //  explicit operator bool() const { return ptr!=0; }
    typedef void (ChSharedPtr::*bool_type)() const;
    void this_type_does_not_support_comparisons() const {}
    operator bool_type() const { return ptr != 0 ? &ChSharedPtr::this_type_does_not_support_comparisons : 0; }

    /// Assignment form for an already-constructed smart-pointer.
    ChSharedPtr& operator=(const ChSharedPtr& r) {
        if (this != &r) {
            release();
            acquire(r);
        }
        return *this;
    }

    /// Dereference the smart pointer to get the object, as in the form *p1
    T& operator*() throw() { return *ptr; }
    const T& operator*() const throw() { return *ptr; }

    /// Used for member access to the contained object,
    /// e.g. pointer->Print() calls T::Print()
    T* operator->() throw() { return ptr; }
    const T* operator->(void) const throw() { return ptr; }

    /// Tells if this is shared by no one else.
    bool IsUnique() const throw() { return (ptr ? ptr->ReferenceCount() == 1 : true); }

    /// Returns the raw pointer to pointed instance.
    /// Note: If a correct programming style is used, you should
    /// never need to use this.
    T* get_ptr() const throw() { return ptr; }
    T* get() const throw() { return ptr; }

    /// Occasionally, the shared pointer can be invalidated (unbound from
    /// object), for instance if you create it with null default
    /// ChSharedPtr<MyClass> pointerA;   instead of typical
    /// ChSharedPtr<MyClass> pointerA(new MyClass);
    bool IsNull() const throw() { return ptr == 0; };

    /// Unbind the shared pointer from referenced shared object,
    /// and automatically delete in case of last reference. It should
    /// be used sparingly, because this unbinding already happens automatically
    /// when the shared pointer dies. Use this only to 'force' premature unbinding.
    void SetNull() {
        this->release();
        ptr = 0;
    }

    /// Tells if the referenced object is inherited from a specific class
    /// and can be cast with copy constructor, for example
    ///   if (ptrA.IsType<classB>() ) { ChSharedPtr<classB> ptrB (ptrA); }
    /// NOTE: this requires polymorphism: classA & classB MUST have 'virtual' destructors
    template <class T_other>
    bool IsType() {
        if (dynamic_cast<T_other*>(ptr))
            return true;
        else
            return false;
    }

    /// This works like dynamic_cast, but for shared pointers.
    /// If it does not succeed, returns an empty pointer.
    /// For example
    /// ChSharedPtr<classA> ma = mb.DynamicCastTo<classA>();
    /// NOTE: this requires polymorphism: classA & classB MUST have 'virtual' destructors
    template <class T_other>
    ChSharedPtr<T_other> DynamicCastTo() const {
        ChSharedPtr<T_other> result;
        if ((result.ptr = dynamic_cast<T_other*>(this->ptr))) {
            result.ptr->AddRef();
        }
        return result;
    }

    /// This works like static_cast, but for shared pointers.
    /// For example
    /// ChSharedPtr<classA> ma = mb.StaticCastTo<classA>();
    /// NOTE: no runtime check on correctness of casting (use DynamicCastTo if required)
    template <class T_other>
    ChSharedPtr<T_other> StaticCastTo() const {
        ChSharedPtr<T_other> result;
        result.ptr = static_cast<T_other*>(this->ptr);
        result.ptr->AddRef();
        return result;
    }

    /// Tells how many references to the pointed object. (Return 0 if empty pointer).
    int ReferenceCounter() { return ptr ? ptr->ReferenceCount() : 0; }

  private:
    T* ptr;

    // increment the count
    template <class T_other>
    void acquire(const ChSharedPtr<T_other>& r) throw() {
        ptr = r.ptr;
        if (ptr) {
            ptr->AddRef();
        }
    }

    // decrement the count, delete if it is 0
    void release() {
        // this should automatically delete the object when its ref.counter goes to 0.
        if (ptr) {
            ptr->RemoveRef();
        }
    }
};

/// Equivalent of dynamic_cast<>() for the ChSharedPtr, for example:
///  ChSharedPtr<classB> pB = dynamic_cast_chshared<classB>(pA);
/// NOTE: this requires polymorphism: classA & classB MUST have 'virtual' destructors

template <typename Tout, typename Tin>
inline ChSharedPtr<Tout> dynamic_cast_chshared(const ChSharedPtr<Tin>& __r) {
    return __r.template DynamicCastTo<Tout>();
}

/// Equivalent of static_cast<>() for the ChSharedPtr, for example:
///  ChSharedPtr<classB> pB = static_cast_chshared<classB>(pA);

template <typename Tout, typename Tin>
inline ChSharedPtr<Tout> static_cast_chshared(const ChSharedPtr<Tin>& __r) {
    return __r.template StaticCastTo<Tout>();
}

// Comparisons operators are required for using the shared pointer
// class in an STL container

template <typename T,typename R>
bool operator==(const ChSharedPtr<T>& left, const ChSharedPtr<R>& right) {
    if (left.get_ptr() == right.get_ptr())
        return true;
    return false;
}
template <typename T,typename R>
bool operator<(const ChSharedPtr<T>& left, const ChSharedPtr<R>& right) {
    if (left.get_ptr() < right.get_ptr())
        return true;
    return false;
}

// Trick to avoid problems as in  http://www.artima.com/cppsource/safebool2.html
// Not needed in future C++0x when we'll use simply:
//  explicit operator bool() const { return ptr!=0; }
template <typename T, typename R>
bool operator!=(const ChSharedPtr<T>& lhs, const R& rhs) {
    lhs.this_type_does_not_support_comparisons();
    return false;
}
template <typename T, typename R>
bool operator==(const ChSharedPtr<T>& lhs, const R& rhs) {
    lhs.this_type_does_not_support_comparisons();
    return false;
}

}  // END_OF_NAMESPACE____

#endif
