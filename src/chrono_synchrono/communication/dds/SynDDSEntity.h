// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// File description
//
// =============================================================================

#ifndef SYN_DDS_ENTITY_H
#define SYN_DDS_ENTITY_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#include <stdint.h>
#include <queue>
#include <vector>

namespace chrono {
namespace synchrono {

/// TODO: Class description here
class SynDDSEntity {
  public:
    ///@brief Construct a new SynDDSEntity object
    ///
    SynDDSEntity() : m_ok(true) {}

    ///@brief Is data available to be processed?
    ///
    ///@return true data is available
    ///@return false data is not available
    virtual bool HasData() { return !m_data.empty(); }

    ///@brief Push new data onto the queue
    ///
    ///@param data data to be pushed
    virtual void PushData(std::vector<uint8_t> data) { m_data.push(data); }

    ///@brief Get the data that is stored in this participant
    ///
    ///@return std::vector<uint8_t> the data currently stored
    virtual std::vector<uint8_t>& GetData() { return m_data.front(); }

    ///@brief  Pop the front of the queue
    virtual void PopData() { m_data.pop(); }

    ///@brief Get the underlying SynDDSListener object
    ///
    ///@return SynDDSListener&
    virtual SynDDSListener& GetListener() = 0;

    ///@brief Checks and waits for the passed number of matched FastDDS Entity at the specified rate
    ///
    ///@param num_matches number of entity matches to wait for. Defaults to 1.
    ///@param rate how often to check for a match in milliseconds. Defaults to 1000ms (1Hz).
    virtual void WaitForMatches(unsigned int num_matches = 1, unsigned int rate = 1000) {
        GetListener().WaitForMatches(num_matches, rate);
    }

    ///@brief Get whether this entity is still alive
    bool IsOk() { return m_ok; }

  protected:
    bool m_ok;  ///< Is this entity still alive

    std::queue<std::vector<uint8_t>> m_data;  ///< Data buffer to store incoming information
};

}  // namespace synchrono
}  // namespace chrono

#endif
