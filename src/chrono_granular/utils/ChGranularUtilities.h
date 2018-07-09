// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================
#include <stdio.h>
#include <stdlib.h>

#pragma once

#define NOT_IMPLEMENTED_YET                                    \
    {                                                          \
        printf(__func__);                                      \
        printf(": FUNCTION NOT IMPLEMENTED YET. EXITTING.\n"); \
        exit(1);                                               \
    }

#define GRANULAR_ERROR(msg)                     \
    {                                           \
        printf(msg);                            \
        printf(__func__);                       \
        printf("\n: EXITTING GRANULAR SIM.\n"); \
        exit(1);                                \
    }
