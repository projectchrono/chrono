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

namespace chrono
{
namespace distributed
{
typedef enum COMM_STATUS {
	EMPTY = 0,
	OWNED,
	GHOST_UP,
	GHOST_DOWN,
	SHARED_UP,
	SHARED_DOWN,
	UNOWNED_UP,
	UNOWNED_DOWN,
	GLOBAL,
	UNDEFINED
} COMM_STATUS;

typedef enum MESSAGE_TYPE {
	EXCHANGE,
	UPDATE,
	FINAL_UPDATE_GIVE,
	FINAL_UPDATE_TAKE
} MESSAGE_TYPE;

} // End namespace distributed
} // End namespace chrono
