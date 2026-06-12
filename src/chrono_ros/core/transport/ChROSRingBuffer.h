// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================
//
// Lock-free single-producer/single-consumer byte ring over shared memory.
//
// Head/tail are std::atomic<uint64_t> living in the shared-memory control
// block (lock-free for u64 on all supported platforms - statically asserted);
// monotonically increasing, masked into the power-of-2 data region. A write
// of multiple chunks publishes a single head update, so the consumer observes
// a frame entirely or not at all.
//
// =============================================================================

#ifndef CH_ROS_CORE_RING_BUFFER_H
#define CH_ROS_CORE_RING_BUFFER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace chrono {
namespace ros {
namespace core {

class RingBuffer {
  public:
    /// @param head producer cursor in shared memory
    /// @param tail consumer cursor in shared memory
    /// @param data data region in shared memory
    /// @param capacity size of the data region; must be a power of 2
    RingBuffer(std::atomic<uint64_t>* head, std::atomic<uint64_t>* tail, uint8_t* data, size_t capacity);

    /// Producer: append one or two chunks as a single atomic publication.
    /// Returns false (writing nothing) if both chunks do not fit.
    bool Write(const void* data1, size_t size1, const void* data2 = nullptr, size_t size2 = 0);

    /// Consumer: copy out 'size' bytes without consuming them.
    /// Returns false if fewer than 'size' bytes are available.
    bool Peek(void* out, size_t size) const;

    /// Consumer: copy out and consume 'size' bytes.
    bool Read(void* out, size_t size);

    /// Consumer: discard 'size' bytes (must be <= Available()).
    void Consume(size_t size);

    size_t Available() const;  ///< bytes ready to read (consumer view)
    size_t Space() const;      ///< bytes writable (producer view)
    size_t Capacity() const { return m_capacity; }

    static bool IsPowerOf2(size_t v) { return v != 0 && (v & (v - 1)) == 0; }

  private:
    void CopyIn(uint64_t at, const void* src, size_t size);
    void CopyOut(uint64_t from, void* dst, size_t size) const;

    std::atomic<uint64_t>* m_head;
    std::atomic<uint64_t>* m_tail;
    uint8_t* m_data;
    size_t m_capacity;
    size_t m_mask;
};

static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "chrono_ros IPC requires lock-free 64-bit atomics (available on all supported platforms)");

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
