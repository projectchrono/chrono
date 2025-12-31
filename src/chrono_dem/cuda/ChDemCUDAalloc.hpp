
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018, Colin Vanden Heuvel
// All rights reserved.
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Colin Vanden Heuvel
// =============================================================================

#ifndef CUDALLOC_HPP
#define CUDALLOC_HPP

#include <cuda_runtime_api.h>
#include <climits>
#include <iostream>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

////#if (__cplusplus >= 201703L)  // C++17 or newer
////template <class T>
////struct cudallocator {
////  public:
////    std::true_type is_always_equal;
////#else  // C++14 or older
template <class T>
class cudallocator {
  public:
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;

    template <class U>
    struct rebind {
        typedef typename ::cudallocator<U> other;
    };

#if (__cplusplus >= 201402L)  // C++14
    std::false_type propagate_on_container_copy_assignment;
    std::false_type propagate_on_container_move_assignment;
    std::false_type propagate_on_container_swap;
#endif
////#endif

    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

////#if (__cplusplus > 201703L)  // newer than (but not including) C++17
////    constexpr cudallocator() noexcept {};
////    constexpr cudallocator(const cudallocator& other) noexcept {}
////
////    template <class U>
////    constexpr cudallocator(const cudallocator<U>& other) noexcept {}
////#else  // C++17 or older
    cudallocator() noexcept {}
    cudallocator(const cudallocator& other) noexcept {}

    template <class U>
    cudallocator(const cudallocator<U>& other) noexcept {}
////#endif

////#if (__cplusplus < 201703L)  // before C++17
    pointer address(reference x) const noexcept { return &x; }

    size_type max_size() const noexcept { return ULLONG_MAX / sizeof(T); }

    template <class... Args>
    void construct(T* p, Args&&... args) {
        ::new ((void*)p) T(std::forward<Args>(args)...);
    }
    void destroy(T* p) { p->~T(); }
////#endif

    pointer allocate(size_type n, std::allocator<void>::const_pointer hint = 0) {
        void* vptr;
        cudaError_t err = cudaMallocManaged(&vptr, n * sizeof(T), cudaMemAttachGlobal);
        if (err == cudaErrorMemoryAllocation || err == cudaErrorNotSupported) {
            throw std::bad_alloc();
        }
        return (T*)vptr;
    }

    void deallocate(pointer p, size_type n) { cudaFree(p); }

    bool operator==(const cudallocator& other) const { return true; }
    bool operator!=(const cudallocator& other) const { return false; }
};

#endif
