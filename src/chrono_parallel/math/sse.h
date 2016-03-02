// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#pragma once

#include "chrono_parallel/ChConfigParallel.h"

// Check if SSE was found in CMake
#ifdef CHRONO_HAS_SSE
// Depending on the SSE variable in CMake include the proper header file for that
// version of sse
#ifdef CHRONO_SSE_1_0
#include <xmmintrin.h>
#elif defined CHRONO_SSE_2_0
#include <emmintrin.h>
#elif defined CHRONO_SSE_3_0
#include <pmmintrin.h>
#elif defined CHRONO_SSE_4_1
#include <smmintrin.h>
#elif defined CHRONO_SSE_4_2
#include <nmmintrin.h>
#endif
#endif

#ifdef CHRONO_HAS_AVX
#include <immintrin.h>
#endif

#if defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_AVX) && defined(CHRONO_PARALLEL_USE_DOUBLE)
#define USE_AVX
#undef USE_SSE
#elif defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_SSE) && !defined(CHRONO_PARALLEL_USE_DOUBLE)
#undef USE_AVX
#define USE_SSE
#else
#undef USE_AVX
#undef USE_SSE
#endif

#include <cstdint>
#include <cstddef>
#include <stdexcept>

// https://gist.github.com/donny-dont/1471329
template <typename T, std::size_t Alignment>
class aligned_allocator {
  public:
    // The following will be the same for virtually all allocators.
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;
    typedef std::size_t size_type;
    typedef ptrdiff_t difference_type;

    T* address(T& r) const { return &r; }

    const T* address(const T& s) const { return &s; }

    std::size_t max_size() const {
        // The following has been carefully written to be independent of
        // the definition of size_t and to avoid signed/unsigned warnings.
        return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T);
    }

    // The following must be the same for all allocators.
    template <typename U>
    struct rebind {
        typedef aligned_allocator<U, Alignment> other;
    };

    bool operator!=(const aligned_allocator& other) const { return !(*this == other); }

    void construct(T* const p, const T& t) const {
        void* const pv = static_cast<void*>(p);

        new (pv) T(t);
    }

    void destroy(T* const p) const { p->~T(); }

    // Returns true if and only if storage allocated from *this
    // can be deallocated from other, and vice versa.
    // Always returns true for stateless allocators.
    bool operator==(const aligned_allocator& other) const { return true; }

    // Default constructor, copy constructor, rebinding constructor, and destructor.
    // Empty for stateless allocators.
    aligned_allocator() {}

    aligned_allocator(const aligned_allocator&) {}

    template <typename U>
    aligned_allocator(const aligned_allocator<U, Alignment>&) {}

    ~aligned_allocator() {}

    // The following will be different for each allocator.
    T* allocate(const std::size_t n) const {
        // The return value of allocate(0) is unspecified.
        // Mallocator returns NULL in order to avoid depending
        // on malloc(0)'s implementation-defined behavior
        // (the implementation can define malloc(0) to return NULL,
        // in which case the bad_alloc check below would fire).
        // All allocators can return NULL in this case.
        if (n == 0) {
            return NULL;
        }

        // All allocators should contain an integer overflow check.
        // The Standardization Committee recommends that std::length_error
        // be thrown in the case of integer overflow.
        if (n > max_size()) {
            throw std::length_error("aligned_allocator<T>::allocate() - Integer overflow.");
        }

// Mallocator wraps malloc().
#if defined(CHRONO_USE_SIMD)
        void* const pv = _mm_malloc(n * sizeof(T), Alignment);
#else
        void* const pv = malloc(n * sizeof(T));
#endif
        // Allocators should throw std::bad_alloc in the case of memory allocation failure.
        if (pv == NULL) {
            throw std::bad_alloc();
        }

        return static_cast<T*>(pv);
    }

    void deallocate(T* const p, const std::size_t n) const { _mm_free(p); }

    // The following will be the same for all allocators that ignore hints.
    template <typename U>
    T* allocate(const std::size_t n, const U* /* const hint */) const {
        return allocate(n);
    }

    // Allocators are not required to be assignable, so
    // all allocators should have a private unimplemented
    // assignment operator. Note that this will trigger the
    // off-by-default (enabled under /Wall) warning C4626
    // "assignment operator could not be generated because a
    // base class assignment operator is inaccessible" within
    // the STL headers, but that warning is useless.
  private:
    aligned_allocator& operator=(const aligned_allocator&);
};
