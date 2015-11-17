// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Description: lots of useful definitions for thrust, includes and enums
// =============================================================================

#pragma once

#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <iostream>

#ifndef _MSC_VER
#include <fenv.h>
#endif
#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChConfigParallel.h"

//empty define
#define thrust_parallel

typedef int shape_type;

#ifdef __CDT_PARSER__
#define BLAZE_SERIAL_SECTION
#else
#endif

#if defined _MSC_VER || defined __clang__
class NullBuffer : public std::streambuf {
  public:
    int overflow(int c) { return c; }
};
static NullBuffer null_buffer;
static std::ostream null_stream(&null_buffer);
#define LOG(X) null_stream
#else
// Enable thread safe logging
#define ELPP_THREAD_SAFE
#include "thirdparty/easylogging/easylogging.h"
#define LOGGINGENABLED
#endif

#define DBG(x) printf(x);


enum SOLVERTYPE {
    STEEPEST_DESCENT,
    GRADIENT_DESCENT,
    CONJUGATE_GRADIENT,
    CONJUGATE_GRADIENT_SQUARED,
    BICONJUGATE_GRADIENT,
    BICONJUGATE_GRADIENT_STAB,
    MINIMUM_RESIDUAL,
    QUASAI_MINIMUM_RESIDUAL,
    APGD,
    APGDREF,
    JACOBI,
    GAUSS_SEIDEL,
    PDIP
};

enum SOLVERMODE { NORMAL, SLIDING, SPINNING, BILATERAL };

enum COLLISIONSYSTEMTYPE { COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL };

enum NARROWPHASETYPE {
    NARROWPHASE_MPR,
    NARROWPHASE_GJK,
    NARROWPHASE_R,
    NARROWPHASE_HYBRID_MPR,
    NARROWPHASE_HYBRID_GJK
};

// This is set so that parts of the code that have been "flattened" can know what
// type of system is used.
enum SYSTEMTYPE { SYSTEM_DVI, SYSTEM_DEM };

enum BILATERALTYPE { BODY_BODY, SHAFT_SHAFT, SHAFT_SHAFT_SHAFT, SHAFT_BODY, SHAFT_SHAFT_BODY, UNKNOWN };

// Supported Logging Levels
enum LOGGINGLEVEL { LOG_NONE, LOG_INFO, LOG_TRACE, LOG_WARNING, LOG_ERROR };
