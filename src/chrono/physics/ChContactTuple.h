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

#ifndef CHCONTACTTUPLE_H
#define CHCONTACTTUPLE_H

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/physics/ChMaterialCouple.h"

namespace chrono {

class ChContactContainerBase;

/// Base class for contact between two generic ChContactable objects.
/// T1 and T2 are of ChContactable sub classes.

template <class Ta, class Tb>
class ChContactTuple {
  public:
    typedef typename Ta::type_variable_tuple_carrier typecarr_a;
    typedef typename Tb::type_variable_tuple_carrier typecarr_b;

  protected:
    ChContactContainerBase* container;  ///< associated contact container

    Ta* objA;  ///< first ChContactable object in the pair
    Tb* objB;  ///< second ChContactable object in the pair

    ChVector<> p1;      ///< max penetration point on geo1, after refining, in abs space
    ChVector<> p2;      ///< max penetration point on geo2, after refining, in abs space
    ChVector<> normal;  ///< normal, on surface of master reference (geo1)

    ChMatrix33<> contact_plane;  ///< the plane of contact (X is normal direction)

    double norm_dist;  ///< penetration distance (negative if going inside) after refining

  public:
    //
    // CONSTRUCTORS
    //

    ChContactTuple() {}

    ChContactTuple(ChContactContainerBase* mcontainer,      ///< contact container
                   Ta* mobjA,                               ///< ChContactable object A
                   Tb* mobjB,                               ///< ChContactable object B
                   const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                   ) {
        assert(mcontainer);
        assert(mobjA);
        assert(mobjB);

        container = mcontainer;

        Reset(mobjA, mobjB, cinfo);
    }

    virtual ~ChContactTuple() {}

    //
    // FUNCTIONS
    //

    /// Initialize again this constraint.
    virtual void Reset(Ta* mobjA,                               ///< ChContactable object A
                       Tb* mobjB,                               ///< ChContactable object B
                       const collision::ChCollisionInfo& cinfo  ///< data for the contact pair
                       ) {
        assert(mobjA);
        assert(mobjB);

        this->objA = mobjA;
        this->objB = mobjB;

        this->p1 = cinfo.vpA;
        this->p2 = cinfo.vpB;
        this->normal = cinfo.vN;
        this->norm_dist = cinfo.distance;

        // Contact plane
        ChVector<> Vx, Vy, Vz;
        XdirToDxDyDz(normal, VECT_Y, Vx, Vy, Vz);
        contact_plane.Set_A_axis(Vx, Vy, Vz);
    }

    /// Get the colliding object A, with point P1
    Ta* GetObjA() { return this->objA; }

    /// Get the colliding object B, with point P2
    Tb* GetObjB() { return this->objB; }

    /// Get the contact coordinate system, expressed in absolute frame.
    /// This represents the 'main' reference of the link: reaction forces
    /// are expressed in this coordinate system. Its origin is point P2.
    /// (It is the coordinate system of the contact plane and normal)
    ChCoordsys<> GetContactCoords() const {
        ChCoordsys<> mcsys;
        ChQuaternion<> mrot = this->contact_plane.Get_A_quaternion();
        mcsys.rot.Set(mrot.e0(), mrot.e1(), mrot.e2(), mrot.e3());
        mcsys.pos = this->p2;
        return mcsys;
    }

    /// Returns the pointer to a contained 3x3 matrix representing the UV and normal
    /// directions of the contact. In detail, the X versor (the 1s column of the
    /// matrix) represents the direction of the contact normal.
    const ChMatrix33<>& GetContactPlane() const { return contact_plane; }

    /// Get the contact point 1, in absolute coordinates
    const ChVector<>& GetContactP1() const { return p1; }

    /// Get the contact point 2, in absolute coordinates
    const ChVector<>& GetContactP2() const { return p2; }

    /// Get the contact normal, in absolute coordinates
    const ChVector<>& GetContactNormal() const { return normal; }

    /// Get the contact distance
    double GetContactDistance() const { return norm_dist; }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector<> GetContactForce() const { return ChVector<>(0); }

    //
    // UPDATING FUNCTIONS
    //

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {}

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {}

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
                                         ) {}

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
                                         ) {}

    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {}

    virtual void ContInjectKRMmatrices(ChSystemDescriptor& mdescriptor) {}

    virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) {}

    virtual void ContIntToDescriptor(const unsigned int off_L,    ///< offset in L, Qc
                                     const ChVectorDynamic<>& L,  ///<
                                     const ChVectorDynamic<>& Qc  ///<
                                     ) {}

    virtual void ContIntFromDescriptor(const unsigned int off_L,  ///< offset in L
                                       ChVectorDynamic<>& L       ///<
                                       ) {}

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) {}

    virtual void ConstraintsBiReset() {}

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) {}

    virtual void ConstraintsFetch_react(double factor) {}
};

}  // end namespace chrono

#endif
