//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTTUPLE_H
#define CHCONTACTTUPLE_H



#include "core/ChFrame.h"
#include "core/ChVectorDynamic.h"
#include "lcp/ChLcpConstraintTwoTuplesContactN.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"
#include "collision/ChCCollisionInfo.h"
#include "physics/ChMaterialCouple.h"

namespace chrono {

  class ChContactContainerBase;

/// Class for DVI contact between two generic ChContactable objects.
/// T1 and T2 are of ChContactable sub classes.

template <class Ta, class Tb>  
class ChContactTuple {

  public: 
    typedef typename Ta::type_variable_tuple_carrier typecarr_a;
    typedef typename Tb::type_variable_tuple_carrier typecarr_b;

  protected:
    //
    // DATA
    //
    ChContactContainerBase* container;

    Ta* objA;   // ChContactable
    Tb* objB;   // ChContactable

    ChVector<> p1;            ///< max penetration point on geo1, after refining, in abs space
    ChVector<> p2;            ///< max penetration point on geo2, after refining, in abs space
    ChVector<double> normal;  ///< normal, on surface of master reference (geo1)

    ///< the plane of contact (X is normal direction)
    ChMatrix33<double> contact_plane;

    double norm_dist;  ///< penetration distance (negative if going inside) after refining

  public:
    //
    // CONSTRUCTORS
    //

    ChContactTuple() {
    }

    ChContactTuple(
              ChContactContainerBase* mcontainer,
              Ta* objA,  ///< ChContactable object A
              Tb* objB,  ///< ChContactable object B
              const collision::ChCollisionInfo& cinfo
              ) {

        assert(container);
        assert(objA);
        assert(objB);

        container = mcontainer;

        Reset(objA, 
              objB,
              cinfo);
        }
    
    virtual ~ChContactTuple() {}

    //
    // FUNCTIONS
    //

    /// Initialize again this constraint.
    virtual void Reset(
            Ta* objA,  ///< ChContactable object A
            Tb* objB,  ///< ChContactable object B
            const collision::ChCollisionInfo& cinfo
        ) {

        assert(objA);
        assert(objB);

        this->objA = objA;
        this->objB = objB;

        this->p1 = cinfo.vpA;
        this->p2 = cinfo.vpB;
        this->normal = cinfo.vN;
        this->norm_dist = cinfo.distance;

        // Contact plane
        ChVector<> Vx, Vy, Vz;
        ChVector<double> singul(VECT_Y);
        XdirToDxDyDz(&normal, &singul, &Vx, &Vy, &Vz);
        contact_plane.Set_A_axis(Vx, Vy, Vz);
    }

        /// Get the colliding object A, with point P1
    virtual Ta* GetObjA() { return this->objA; }

        /// Get the colliding object B, with point P2
    virtual Tb* GetObjB() { return this->objB; }


    /// Get the contact coordinate system, expressed in absolute frame.
    /// This represents the 'main' reference of the link: reaction forces
    /// are expressed in this coordinate system. Its origin is point P2.
    /// (It is the coordinate system of the contact plane and normal)
    virtual ChCoordsys<> GetContactCoords() {
        ChCoordsys<> mcsys;
        ChQuaternion<float> mrot = this->contact_plane.Get_A_quaternion();
        mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
        mcsys.pos = this->p2;
        return mcsys;
    }

    /// Returns the pointer to a contained 3x3 matrix representing the UV and normal
    /// directions of the contact. In detail, the X versor (the 1s column of the
    /// matrix) represents the direction of the contact normal.
    ChMatrix33<double>* GetContactPlane() { return &contact_plane; };

    /// Get the contact point 1, in absolute coordinates
    virtual ChVector<> GetContactP1() { return p1; };

    /// Get the contact point 2, in absolute coordinates
    virtual ChVector<> GetContactP2() { return p2; };

    /// Get the contact normal, in absolute coordinates
    virtual ChVector<double> GetContactNormal() { return normal; };

    /// Get the contact distance
    virtual double GetContactDistance() { return norm_dist; };

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector<> GetContactForce() =0;



    //
    // UPDATING FUNCTIONS
    //

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) =0;

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) =0;

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                 ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                 const ChVectorDynamic<>& L,  ///< the L vector
                                 const double c               ///< a scaling factor
                                 ) = 0;

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                 ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                 const double c,            ///< a scaling factor
                                 bool do_clamp,             ///< apply clamping to c*C?
                                 double recovery_clamp      ///< value for min/max clamping of c*C
                                 ) = 0;

    
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) =0;

    virtual void ContIntToLCP(const unsigned int off_L,  ///< offset in L, Qc
                      const ChVectorDynamic<>& L,
                      const ChVectorDynamic<>& Qc) = 0;

    virtual void ContIntFromLCP(const unsigned int off_L,  ///< offset in L
                        ChVectorDynamic<>& L)  = 0;

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor)  = 0;

    virtual void ConstraintsBiReset() = 0;

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) = 0;

    virtual void ConstraintsFetch_react(double factor) = 0;

    virtual void ConstraintsLiLoadSuggestedSpeedSolution()  = 0;
    virtual void ConstraintsLiLoadSuggestedPositionSolution() = 0;

    virtual void ConstraintsLiFetchSuggestedSpeedSolution() = 0;
    virtual void ConstraintsLiFetchSuggestedPositionSolution() = 0;

};



}  // END_OF_NAMESPACE____

#endif
