//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSYSTEMOPENMP_H
#define CHSYSTEMOPENMP_H

//////////////////////////////////////////////////
//
//   ChSystemOpenMP.h
//
//   The physical system definition.
//   A phisical system encloses bodies, links, 
//   probes, etc.
//   This is the oldest source file of Chrono::Engine
//   therefore it is poorly written and under major
//   revisiting... Stay tuned..
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChSystem.h"

namespace chrono {

//////////////////////////////////////
//  MULTIBODY SYSTEM CLASS
//
/// This class is used to represent a multibody physical system,
/// so it acts also as a database for most items involved in
/// simulations, most noticeably objects of ChBody and ChLink
/// classes, which are used to represent mechanisms.
///
/// Moreover, it also owns some global settings and features,
/// like the gravity acceleration, the global time and so on.
///
/// This object will be responsible of performing the entire
/// physical simulation (dynamics, kinematics, statics, etc.),
/// so you need at least one ChSystem object in your program, in
/// order to perform simulations (you'll insert rigid bodies and
/// links into it..)
///

class ChApi ChSystemOpenMP: public ChSystem {

    CH_RTTI(ChSystemOpenMP,ChSystem)
        ;

    public:

        //
        // BUILDERS
        //

        /// Create a physical system.
        /// Note, in case you will use collision detection, the values of
        /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
        /// collision algorithm in an optimal way. Scene size should be approximately
        /// the radius of the expected area where colliding objects will move.
        /// If init_sys is false it does not initialize the collision system or solver
        /// assumes that the user will do so.
        ChSystemOpenMP(unsigned int max_objects = 16000, double scene_size = 500, bool init_sys = true);

        /// Destructor
        virtual ~ChSystemOpenMP();
        void Update();

        void LCPprepare(bool load_jacobians, bool load_v, double F_factor, double K_factor, double R_factor, double Ct_factor, double C_factor, double recovery_clamp, bool do_clamp, ChLcpSystemDescriptor& mdescriptor);

        /// Copy from another ChSystem.
        /// Note! All settings are copied, but the hierarchy of children
        /// As Integrate_Y(), but uses the differential inclusion approach as in Anitescu,
        /// Use Anitescu stepper, with position stabilization in speed stage.
        virtual int Integrate_Y_impulse_Anitescu();

};

} // END_OF_NAMESPACE____

#endif // ond of ChSystem.h
