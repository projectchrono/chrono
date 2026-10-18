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
//
// Precompiled header for the Chrono_core library.
//
// Chrono_core has ~500 translation units, almost all of which include the core
// linear-algebra headers (and therefore Eigen) together with the STL stream and
// container headers. Build-Insights profiling shows these dominate front-end
// parse and template-instantiation cost, so parsing them once here and reusing
// the result across every TU removes a large amount of repeated work.
//
// ChMatrix.h (rather than a bare Eigen include) is the anchor on purpose: it
// defines EIGEN_MATRIXBASE_PLUGIN / EIGEN_SPARSEMATRIX_PLUGIN before including
// Eigen/Dense and Eigen/Sparse, so pulling it in reproduces Chrono's exact Eigen
// configuration and also transitively precompiles ChArchive.h / ChArchiveASCII.h.
//
// The __cplusplus guard keeps this header empty when the precompiled header is
// generated for the handful of C sources in the target.
// =============================================================================

#ifndef CHRONO_CORE_PCH_H
#define CHRONO_CORE_PCH_H

#ifdef __cplusplus

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "chrono/core/ChMatrix.h"

#endif  // __cplusplus

#endif  // CHRONO_CORE_PCH_H
