// Copyright (c) 2018, Colin Vanden Heuvel
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional permissions:
// The University of Wisconsin - Madison may relicense this code
// in any way so long as it maintains the above copyright notice.



#include <cuda_runtime_api.h>
#include <climits>
#include <iostream>
#include <memory>
#include <new>
#include <utility>

template <class T>
class cudallocator {
  public:
    typedef T value_type;
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    template <class U>
    struct rebind {
        typedef typename ::cudallocator<U> other;
    };
    bool propagate_on_container_move_assignment = true;

    pointer address(reference x) const noexcept { return &x; }

    pointer allocate(size_type n, std::allocator<void>::const_pointer hint = 0) {
        void* vptr;
        cudaError_t err = cudaMallocManaged(&vptr, n * sizeof(T), cudaMemAttachGlobal);
        if (err == cudaErrorMemoryAllocation || err == cudaErrorNotSupported) {
            throw std::bad_alloc();
        }
        return (T*)vptr;
    }

    void deallocate(pointer p, size_type n) { cudaFree(p); }

    size_type max_size() const noexcept { return ULLONG_MAX / sizeof(T); }

    template <class... Args>
    void construct(T* p, Args&&... args) {
        ::new ((void*)p) T(std::forward<Args>(args)...);
    }
    void destroy(T* p) { p->~T(); }

    bool operator==(const cudallocator& other) const { return true; }
};
