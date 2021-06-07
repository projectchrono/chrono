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
// Authors: Nic Olsen
// =============================================================================

#pragma once

namespace chrono {
namespace distributed {

/// @addtogroup distributed_module
/// @{

/// Location and status of a given body with respect to this rank
typedef enum COMM_STATUS {
    EMPTY = 0,         /// None, empty, disabled
    OWNED = 1,         /// exclusive to this rank
    GHOST_UP = 2,      /// a proxy for a body on high neighbor rank
    GHOST_DOWN = 3,    /// a proxy for a body on low neighbor rank
    SHARED_UP = 4,     /// has a proxy body on high neighbor rank
    SHARED_DOWN = 5,   /// has a proxy body on low neighbor rank
    UNOWNED_UP = 6,    /// unrelated to this rank
    UNOWNED_DOWN = 7,  /// unrelated to this rank
    GLOBAL = 8,        /// Present on all ranks
    UNDEFINED = 9
} COMM_STATUS;
/// @} distributed_module

/// @addtogroup distributed_module
/// @{

/// Types of internal message that can be sent
typedef enum MESSAGE_TYPE {
    EXCHANGE,              /// Introduction of new body to a rank
    UPDATE,                /// Update for an existing body on a rank from the owning rank
    FINAL_UPDATE_GIVE,     /// Update which ends in the other rank taking exclusive ownership
    FINAL_UPDATE_TAKE,     /// Update which ends in this rank taking exclusive ownership
    UPDATE_TRANSFER_SHARE  /// Update which updates the primary rank for the body
} MESSAGE_TYPE;
/// @} distributed_module

}  // End namespace distributed
}  // End namespace chrono
