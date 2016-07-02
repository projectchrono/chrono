//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChClassRegister.cpp
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChClassRegister.h"

namespace chrono {

/// The root of the list of ChClassRegister<t> objects, each
/// will contain the name ID of the class,and other useful things
/// such as the method which can create a 't' object in runtime.

ChClassRegisterCommon** ChClassRegisterCommon::GetStaticHeadAddr() {
    static ChClassRegisterCommon* mlocalHead = 0;  // A STATIC DATA
    return &mlocalHead;                            //&m_pHead;
}



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

}  // END_OF_NAMESPACE____
