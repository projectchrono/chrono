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
typedef enum COMM_STATUS {
    EMPTY = 0,
    OWNED = 1,
    GHOST_UP = 2,
    GHOST_DOWN = 3,
    SHARED_UP = 4,
    SHARED_DOWN = 5,
    UNOWNED_UP = 6,
    UNOWNED_DOWN = 7,
    GLOBAL = 8,
    UNDEFINED = 9
} COMM_STATUS;

typedef enum MESSAGE_TYPE {
    EXCHANGE,
    UPDATE,
    FINAL_UPDATE_GIVE,
    FINAL_UPDATE_TAKE,
    UPDATE_TRANSFER_SHARE
} MESSAGE_TYPE;

}  // End namespace distributed
}  // End namespace chrono
