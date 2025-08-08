// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_FLOW_API_H
#define CH_FLOW_API_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_FLOW
// (so that the symbols with 'ChFlowApi' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_FLOW_API_COMPILE)
#define ChFlowApi ChApiEXPORT
#else
#define ChFlowApi ChApiIMPORT
#endif



namespace chrono {

/// @addtogroup FLOW_module
/// @{

/// Namespace with classes for the FLOW module.
namespace flow {}

/// @}

}

#endif  // END of header
