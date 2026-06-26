
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

#ifndef CH_DEM_GPU_APPLOC_H
#define CH_DEM_GPU_APPLOC_H

#include <climits>
#include <iostream>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

#include "chrono_dem/ChDemDefines.h"

////#if (__cplusplus >= 201703L)  // C++17 or newer
////template <class T>
////struct gpuallocator {
////  public:
////    std::true_type is_always_equal;
////#else  // C++14 or older
template <class T>
class gpuallocator {
  public:
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;

    template <class U>
    struct rebind {
        typedef typename ::gpuallocator<U> other;
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
    ////    constexpr gpuallocator() noexcept {};
    ////    constexpr gpuallocator(const gpuallocator& other) noexcept {}
    ////
    ////    template <class U>
    ////    constexpr gpuallocator(const gpuallocator<U>& other) noexcept {}
    ////#else  // C++17 or older
    gpuallocator() noexcept {}
    gpuallocator(const gpuallocator& other) noexcept {}

    template <class U>
    gpuallocator(const gpuallocator<U>& other) noexcept {}
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
        gpuError err = gpuMallocManaged(&vptr, n * sizeof(T), gpuMemAttachGlobal);
        if (err == gpuErrorMemoryAllocation || err == gpuErrorNotSupported) {
            throw std::bad_alloc();
        }
        return (T*)vptr;
    }

    void deallocate(pointer p, size_type n) {
        if (p) {
            demErrchk(gpuFree(p));
        }
    }

    bool operator==(const gpuallocator& other) const { return true; }
    bool operator!=(const gpuallocator& other) const { return false; }
};

#endif
