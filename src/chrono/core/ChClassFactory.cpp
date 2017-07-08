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

#include "chrono/core/ChClassFactory.h"

namespace chrono {

/// Access the unique class factory here. It is unique even
/// between dll boundaries. It is allocated the 1st time it is called, if null.

ChClassFactory* ChClassFactory::GetGlobalClassFactory() {
    static ChClassFactory* mfactory = 0;
    if (!mfactory)
        mfactory = new ChClassFactory;
    return mfactory;
}

/// Delete the global class factory
void ChClassFactory::DisposeGlobalClassFactory() {
    delete ChClassFactory::GetGlobalClassFactory();
}

}  // end namespace chrono
