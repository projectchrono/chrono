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

#ifndef SYN_DDS_LISTENER
#define SYN_DDS_LISTENER

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication_dds
/// @{

/// TODO: Class description here
class SYN_API SynDDSListener {
  public:
    SynDDSListener();

    ///@brief Checks and waits for the passed number of matched FastDDS Entity at the specified rate
    ///
    ///@param num_matches number of entity matches to wait for. Defaults to 1.
    ///@param rate how often to check for a match in milliseconds. Defaults to 1000ms (1Hz).
    void WaitForMatches(unsigned int num_matches = 1, unsigned int rate = 1000);

    ///@brief Is the listener matched with atleast one FastDDS Entity
    ///
    ///@return true the listener is matched
    ///@return false the listener is not matched
    bool IsMatched() { return m_matched > 0; }

    ///@brief Get the number of matched FastDDS Entities
    ///
    ///@return int number of matched entities
    unsigned int GetNumMatched() { return m_matched; }

  protected:
    unsigned int m_matched;  ///< Number of matched FastDDS Entities
};

/// @} synchrono_communication_dds

}  // namespace synchrono
}  // namespace chrono

#endif