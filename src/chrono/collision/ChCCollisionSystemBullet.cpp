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

#include "chrono/collision/ChCCollisionSystemBullet.h"
#include "chrono/collision/ChCModelBullet.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/collision/bullet/LinearMath/btPoolAllocator.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/bt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"

extern btScalar gContactBreakingThreshold;

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionSystemBullet)


////////////////////////////////////
////////////////////////////////////

// Utility class that we use to override the default cylinder-sphere collision
// case, because the default behavior in Bullet was using the GJK algorithm, that
// gives not 100% precise results if the cylinder is much larger than the sphere:
class btSphereCylinderCollisionAlgorithm : public btActivatingCollisionAlgorithm {
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;

  public:
    btSphereCylinderCollisionAlgorithm(btPersistentManifold* mf,
                                       const btCollisionAlgorithmConstructionInfo& ci,
                                       btCollisionObject* col0,
                                       btCollisionObject* col1,
                                       bool isSwapped)
        : btActivatingCollisionAlgorithm(ci, col0, col1),
          m_ownManifold(false),
          m_manifoldPtr(mf),
          m_isSwapped(isSwapped) {
        btCollisionObject* sphereObj = m_isSwapped ? col1 : col0;
        btCollisionObject* cylObj = m_isSwapped ? col0 : col1;

        if (!m_manifoldPtr) {
            m_manifoldPtr = m_dispatcher->getNewManifold(sphereObj, cylObj);
            m_ownManifold = true;
        }
    }

    btSphereCylinderCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
        : btActivatingCollisionAlgorithm(ci) {}

    virtual void processCollision(btCollisionObject* body0,
                                  btCollisionObject* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) {
        (void)dispatchInfo;

        if (!m_manifoldPtr)
            return;

        btCollisionObject* sphereObj = m_isSwapped ? body1 : body0;
        btCollisionObject* cylObj = m_isSwapped ? body0 : body1;

        resultOut->setPersistentManifold(m_manifoldPtr);

        btSphereShape* sphere0 = (btSphereShape*)sphereObj->getCollisionShape();
        btCylinderShape* cylinder = (btCylinderShape*)cylObj->getCollisionShape();

        const btTransform& m44T = cylObj->getWorldTransform();
        btVector3 diff = m44T.invXform(
            sphereObj->getWorldTransform()
                .getOrigin());  // col0->getWorldTransform().getOrigin()-  col1->getWorldTransform().getOrigin();
        btScalar radius0 = sphere0->getRadius();
        btScalar radius1 = cylinder->getHalfExtentsWithMargin().getX();  // cylinder->getRadius();
        btScalar H1 = cylinder->getHalfExtentsWithMargin().getY();

        btVector3 r1 = diff;
        r1.setY(0);

        btScalar y1 = diff.y();

        btScalar r1_len = r1.length();

        btVector3 pos1;
        btVector3 normalOnSurfaceB(1, 0, 0);
        btScalar dist;

        // Case A
        if ((y1 <= H1) && (y1 >= -H1)) {
            /// iff distance positive, don't generate a new contact
            if (r1_len > (radius0 + radius1)) {
                resultOut->refreshContactPoints();
                return;
            }
            /// distance (negative means penetration)
            dist = r1_len - (radius0 + radius1);

            btVector3 localnormalOnSurfaceB;
            if (r1_len > SIMD_EPSILON) {
                localnormalOnSurfaceB = r1 / r1_len;
                normalOnSurfaceB = m44T.getBasis() * localnormalOnSurfaceB;
            }
            /// point on B (worldspace)
            pos1 = m44T(btVector3(0, y1, 0)) + radius1 * normalOnSurfaceB;
        } else {
            btScalar side = 1;
            if (y1 < -H1)
                side = -1;

            if (r1_len > radius1) {
                // case B
                btVector3 pos_loc = r1.normalized() * radius1 + btVector3(0, H1 * side, 0);
                pos1 = m44T(pos_loc);
                btVector3 d = sphereObj->getWorldTransform().getOrigin() - pos1;
                normalOnSurfaceB = d.normalized();
                dist = d.length() - radius0;
            } else {
                // case C
                normalOnSurfaceB = m44T.getBasis() * btVector3(0, 1 * side, 0);
                btVector3 pos_loc = r1 + btVector3(0, H1 * side, 0);
                pos1 = m44T(pos_loc);
                dist = side * (y1 - H1) - radius0;
            }
        }
        /// report a contact. internally this will be kept persistent, and contact reduction is done
        resultOut->addContactPoint(normalOnSurfaceB, pos1, dist);

        resultOut->refreshContactPoints();
    }

    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) {
        // not yet
        return btScalar(1.);
    }

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) {
        if (m_manifoldPtr && m_ownManifold) {
            manifoldArray.push_back(m_manifoldPtr);
        }
    }

    virtual ~btSphereCylinderCollisionAlgorithm() {
        if (m_ownManifold) {
            if (m_manifoldPtr)
                m_dispatcher->releaseManifold(m_manifoldPtr);
        }
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               btCollisionObject* body0,
                                                               btCollisionObject* body1) {
            void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btSphereCylinderCollisionAlgorithm));
            if (!m_swapped) {
                return new (mem) btSphereCylinderCollisionAlgorithm(0, ci, body0, body1, false);
            } else {
                return new (mem) btSphereCylinderCollisionAlgorithm(0, ci, body0, body1, true);
            }
        }
    };
};

////////////////////////////////////
////////////////////////////////////


// Utility class that we use to override the default 2Dsegment-2Darc collision
// case (it works only of the two are coplanar)
class btArcSegmentCollisionAlgorithm : public btActivatingCollisionAlgorithm {
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;

  public:
    btArcSegmentCollisionAlgorithm(btPersistentManifold* mf,
                                       const btCollisionAlgorithmConstructionInfo& ci,
                                       btCollisionObject* col0,
                                       btCollisionObject* col1,
                                       bool isSwapped)
        : btActivatingCollisionAlgorithm(ci, col0, col1),
          m_ownManifold(false),
          m_manifoldPtr(mf),
          m_isSwapped(isSwapped) {
        btCollisionObject* arcObj = m_isSwapped ? col1 : col0;
        btCollisionObject* segmentObj = m_isSwapped ? col0 : col1;

        if (!m_manifoldPtr) {
            m_manifoldPtr = m_dispatcher->getNewManifold(arcObj, segmentObj);
            m_ownManifold = true;
        }
    }

    btArcSegmentCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
        : btActivatingCollisionAlgorithm(ci) {}

    virtual void processCollision(btCollisionObject* body0,
                                  btCollisionObject* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) {
        (void)dispatchInfo;

        if (!m_manifoldPtr)
            return;

        btCollisionObject* arcObj = m_isSwapped ? body1 : body0;
        btCollisionObject* segmentObj = m_isSwapped ? body0 : body1;

        resultOut->setPersistentManifold(m_manifoldPtr);

        // only 1 contact per pair, avoid persistence
        resultOut->getPersistentManifold()->clearManifold();

        bt2DarcShape* arc = (bt2DarcShape*)arcObj->getCollisionShape();
        bt2DsegmentShape* segment = (bt2DsegmentShape*)segmentObj->getCollisionShape();

        // A concave arc (i.e.with outward volume, counterclockwise abscissa) will never collide with segments
        if (arc->get_counterclock()) 
            return;

        const btTransform& m44Tarc= arcObj->getWorldTransform();
        const btTransform& m44Tsegment = segmentObj->getWorldTransform();  

        // Shapes on two planes that are not so parallel? no collisions!
        btVector3 Zarc     = m44Tarc.getBasis().getColumn(2);
        btVector3 Zsegment = m44Tsegment.getBasis().getColumn(2);
        if (fabs(Zarc.dot(Zsegment)) < 0.99) //***TODO*** threshold as setting
            return;

        // Shapes on two planes that are too far? no collisions!
        btVector3 diff = m44Tsegment.invXform(m44Tarc.getOrigin()); 
        if (fabs(diff.getZ())> (arc->get_zthickness()+segment->get_zthickness())) 
            return;

        // vectors of body 1 in body 2 csys:
        btVector3 local_arc_center = m44Tsegment.invXform( m44Tarc * btVector3(arc->get_X(),arc->get_Y(),0) );
        btVector3 local_arc_X     = m44Tsegment.getBasis().transpose() * ( m44Tarc.getBasis() * btVector3(1,0,0) );
        double  local_arc_rot    = atan2(local_arc_X.getY(), local_arc_X.getX());
        double  arc1_angle1 = local_arc_rot + arc->get_angle1();
        double  arc1_angle2 = local_arc_rot + arc->get_angle2();

        btVector3 local_CS1 = local_arc_center - segment->get_P1();
        btVector3 local_seg_S2S1 =  (segment->get_P2() - segment->get_P1());
        btScalar  seg_length = local_seg_S2S1.length();
        if (seg_length < 1e-30) 
            return;
        btVector3 local_seg_D =  local_seg_S2S1 / seg_length;
        btScalar param = local_CS1.dot(local_seg_D);
        
        // contact out of segment extrema?
        if (param < 0) 
            return;
        if (param > seg_length)
            return;

        btVector3 local_P2   = segment->get_P1() + local_seg_D * param;
        btVector3 local_CP2 = local_arc_center - local_P2;
        btVector3 local_R = local_CP2.normalized() * arc->get_radius();
        btVector3 local_P1;
        btVector3 local_N2;
        if (local_seg_S2S1.cross(local_CP2).getZ() > 0 ) {
            local_P1 = local_arc_center - local_R;
            local_N2 = local_R.normalized(); 
        }
        else {
            local_P1 = local_arc_center + local_R;
            local_N2 = -local_R.normalized(); 
        }
        btVector3 local_P1P2 = local_P1 - local_P2;

        double  alpha   = atan2(-local_N2.getY(), -local_N2.getX());

        // Discard points out of min-max angles

        // to always positive angles:
        arc1_angle1 = fmod(arc1_angle1 + 1e-30, CH_C_2PI);
        if (arc1_angle1 < 0)
            arc1_angle1 += CH_C_2PI;
        arc1_angle2 = fmod(arc1_angle2 + 1e-30, CH_C_2PI);
        if (arc1_angle2 < 0)
            arc1_angle2 += CH_C_2PI;
        alpha = fmod(alpha, CH_C_2PI);
        if (alpha < 0)
            alpha += CH_C_2PI;
     
        arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
        arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);

        alpha = fmod(alpha, CH_C_2PI);

        bool inangle1 = false;

        if (arc1_angle1 < arc1_angle2) {
            if (alpha >= arc1_angle2 || 
                alpha <= arc1_angle1) 
                    inangle1 = true;
        } else {
            if (alpha >= arc1_angle2 && 
                alpha <= arc1_angle1) 
                    inangle1 = true;
        }
        
        if (!inangle1)
           return;

        // transform in absolute coords:
        //btVector3 pos1 = m44Tsegment * local_P1; // not needed
        btVector3 pos2 = m44Tsegment * local_P2; 
        btVector3 normal_on_2 = m44Tsegment.getBasis() * local_N2;
        btScalar dist = local_N2.dot(local_P1 - local_P2);

        // too far or too interpenetrate? discard.
        if (fabs(dist) > (arc->getMargin()+segment->getMargin()))
            return;

        /// report a contact. internally this will be kept persistent, and contact reduction is done
        resultOut->addContactPoint(normal_on_2, pos2, dist);

        resultOut->refreshContactPoints();
    }

    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) {
        // not yet
        return btScalar(1.);
    }

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) {
        if (m_manifoldPtr && m_ownManifold) {
            manifoldArray.push_back(m_manifoldPtr);
        }
    }

    virtual ~btArcSegmentCollisionAlgorithm() {
        if (m_ownManifold) {
            if (m_manifoldPtr)
                m_dispatcher->releaseManifold(m_manifoldPtr);
        }
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               btCollisionObject* body0,
                                                               btCollisionObject* body1) {
            void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btArcSegmentCollisionAlgorithm));
            if (!m_swapped) {
                return new (mem) btArcSegmentCollisionAlgorithm(0, ci, body0, body1, false);
            } else {
                return new (mem) btArcSegmentCollisionAlgorithm(0, ci, body0, body1, true);
            }
        }
    };
};

////////////////////////////////////
////////////////////////////////////



// Utility class that we use to override the default 2Darc-2Darc collision
// case (it works only of the two are coplanar)
class btArcArcCollisionAlgorithm : public btActivatingCollisionAlgorithm {
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;

  public:
    btArcArcCollisionAlgorithm(btPersistentManifold* mf,
                                       const btCollisionAlgorithmConstructionInfo& ci,
                                       btCollisionObject* col0,
                                       btCollisionObject* col1,
                                       bool isSwapped)
        : btActivatingCollisionAlgorithm(ci, col0, col1),
          m_ownManifold(false),
          m_manifoldPtr(mf),
          m_isSwapped(isSwapped) {
        btCollisionObject* arcObj1 = m_isSwapped ? col1 : col0;
        btCollisionObject* arcObj2 = m_isSwapped ? col0 : col1;

        if (!m_manifoldPtr) {
            m_manifoldPtr = m_dispatcher->getNewManifold(arcObj1, arcObj2);
            m_ownManifold = true;
        }
    }

    btArcArcCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
        : btActivatingCollisionAlgorithm(ci) {}

    virtual void processCollision(btCollisionObject* body0,
                                  btCollisionObject* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) {
        (void)dispatchInfo;

        if (!m_manifoldPtr)
            return;

        btCollisionObject* arcObj1 = m_isSwapped ? body1 : body0;
        btCollisionObject* arcObj2 = m_isSwapped ? body0 : body1;

        resultOut->setPersistentManifold(m_manifoldPtr);

        // only 1 contact per pair, avoid persistence
        resultOut->getPersistentManifold()->clearManifold();

        bt2DarcShape* arc1 = (bt2DarcShape*)arcObj1->getCollisionShape();
        bt2DarcShape* arc2 = (bt2DarcShape*)arcObj2->getCollisionShape();

        const btTransform& m44Tarc1 = arcObj1->getWorldTransform();
        const btTransform& m44Tarc2 = arcObj2->getWorldTransform();  

        // Shapes on two planes that are not so parallel? no collisions!
        btVector3 Zarc1 = m44Tarc1.getBasis().getColumn(2);
        btVector3 Zarc2 = m44Tarc2.getBasis().getColumn(2);
        if (fabs(Zarc1.dot(Zarc2)) < 0.99) //***TODO*** threshold as setting
            return;

        // Shapes on two planes that are too far? no collisions!
        btVector3 diff = m44Tarc2.invXform(m44Tarc1.getOrigin()); 
        if (fabs(diff.getZ())> (arc1->get_zthickness()+arc2->get_zthickness())) 
            return;

        // vectors and angles of arc 1 in arc 2 csys:
        btVector3 local_arc1_center = m44Tarc2.invXform( m44Tarc1 * btVector3(arc1->get_X(),arc1->get_Y(),0) );
        btVector3 local_arc1_X      = m44Tarc2.getBasis().transpose() * ( m44Tarc1.getBasis() * btVector3(1,0,0) );
        double  local_arc1_rot    = atan2(local_arc1_X.getY(), local_arc1_X.getX());
        double  arc1_angle1 = local_arc1_rot + arc1->get_angle1();
        double  arc1_angle2 = local_arc1_rot + arc1->get_angle2();

        btVector3 local_arc2_center = btVector3(arc2->get_X(),arc2->get_Y(),0);
        double  arc2_angle1 = arc2->get_angle1();
        double  arc2_angle2 = arc2->get_angle2();

        btVector3 local_C1C2 = local_arc1_center - local_arc2_center;
        btVector3 local_D12  = local_C1C2.normalized();

        btVector3 local_P1;
        btVector3 local_P2;
        btVector3 local_N2;
        double dist = 0;
        bool paired = false;
        double  alpha   = atan2(local_C1C2.getY(), local_C1C2.getX());
        double  alpha1, alpha2;

        // convex-convex
        if (arc1->get_counterclock() == false && arc2->get_counterclock() == false) {
            local_P1 = local_arc1_center - local_D12* arc1->get_radius();
            local_P2 = local_arc2_center + local_D12* arc2->get_radius();
            local_N2  = local_D12;
            dist = local_C1C2.length() - arc1->get_radius() - arc2->get_radius();
            alpha1 = alpha + CH_C_PI;
            alpha2 = alpha;
            paired = true;
        }
        // convex-concave
        if (arc1->get_counterclock() == false && arc2->get_counterclock() == true) 
            if (arc1->get_radius() <= arc2->get_radius()) { 
                local_P1 = local_arc1_center + local_D12* arc1->get_radius();
                local_P2 = local_arc2_center + local_D12* arc2->get_radius();
                local_N2  = - local_D12;
                dist = - local_C1C2.length() - arc1->get_radius() + arc2->get_radius();
                alpha1 = alpha;
                alpha2 = alpha;
                paired = true;
            }
        // concave-convex
        if (arc1->get_counterclock() == true && arc2->get_counterclock() == false) 
            if (arc1->get_radius() >= arc2->get_radius()) { 
                local_P1 = local_arc1_center - local_D12* arc1->get_radius();
                local_P2 = local_arc2_center - local_D12* arc2->get_radius();
                local_N2  = - local_D12;
                dist = - local_C1C2.length() + arc1->get_radius() - arc2->get_radius();
                alpha1 = alpha + CH_C_PI;
                alpha2 = alpha + CH_C_PI;
                paired = true;
            }

        if (!paired) 
            return;

        // Discard points out of min-max angles

        // to always positive angles:
        arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
        if (arc1_angle1 < 0)
            arc1_angle1 += CH_C_2PI;
        arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);
        if (arc1_angle2 < 0)
            arc1_angle2 += CH_C_2PI;
        arc2_angle1 = fmod(arc2_angle1, CH_C_2PI);
        if (arc2_angle1 < 0)
            arc2_angle1 += CH_C_2PI;
        arc2_angle2 = fmod(arc2_angle2, CH_C_2PI);
        if (arc2_angle2 < 0)
            arc2_angle2 += CH_C_2PI;
        alpha1 = fmod(alpha1, CH_C_2PI);
        if (alpha1 < 0)
            alpha1 += CH_C_2PI;
        alpha2 = fmod(alpha2, CH_C_2PI);
        if (alpha2 < 0)
            alpha2 += CH_C_2PI;

        arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
        arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);
        arc2_angle1 = fmod(arc2_angle1, CH_C_2PI);
        arc2_angle2 = fmod(arc2_angle2, CH_C_2PI);
        alpha1 = fmod(alpha1, CH_C_2PI);
        alpha2 = fmod(alpha2, CH_C_2PI);

        bool inangle1 = false;
        bool inangle2 = false;

        if (arc1->get_counterclock() == true) {
            if (arc1_angle1 < arc1_angle2) {
                if (alpha1 >= arc1_angle1 && 
                    alpha1 <= arc1_angle2) 
                        inangle1 = true;
            } else {
                if (alpha1 >= arc1_angle1 || 
                    alpha1 <= arc1_angle2) 
                        inangle1 = true;
            }
        } else {
            if (arc1_angle1 < arc1_angle2) {
                if (alpha1 >= arc1_angle2 || 
                    alpha1 <= arc1_angle1) 
                        inangle1 = true;
            } else {
                if (alpha1 >= arc1_angle2 && 
                    alpha1 <= arc1_angle1) 
                        inangle1 = true;
            }
        }

        if (arc2->get_counterclock() == true) {
            if (arc2_angle1 < arc2_angle2) {
                if (alpha2 >= arc2_angle1 && 
                    alpha2 <= arc2_angle2) 
                        inangle2 = true;
            } else {
                if (alpha2 >= arc2_angle1 || 
                    alpha2 <= arc2_angle2) 
                        inangle2 = true;
            }
        } else {
            if (arc2_angle1 < arc2_angle2) {
                if (alpha2 >= arc2_angle2 || 
                    alpha2 <= arc2_angle1) 
                        inangle2 = true;
            } else {
                if (alpha2 >= arc2_angle2 && 
                    alpha2 <= arc2_angle1) 
                        inangle2 = true;
            }
        }
      
        if (!(inangle1 && inangle2))
            return;

        // transform in absolute coords:
        btVector3 pos2        = m44Tarc2 * local_P2; 
        btVector3 normal_on_2 = m44Tarc2.getBasis() * local_N2;

        // too far or too interpenetrate? discard.
        if (fabs(dist) > (arc1->getMargin()+arc2->getMargin()))
            return;

        /// report a contact.
        resultOut->addContactPoint(normal_on_2, pos2, (btScalar)dist);

        resultOut->refreshContactPoints();
    }

    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) {
        // not yet
        return btScalar(1.);
    }

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) {
        if (m_manifoldPtr && m_ownManifold) {
            manifoldArray.push_back(m_manifoldPtr);
        }
    }

    virtual ~btArcArcCollisionAlgorithm() {
        if (m_ownManifold) {
            if (m_manifoldPtr)
                m_dispatcher->releaseManifold(m_manifoldPtr);
        }
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               btCollisionObject* body0,
                                                               btCollisionObject* body1) {
            void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btArcArcCollisionAlgorithm));
            if (!m_swapped) {
                return new (mem) btArcArcCollisionAlgorithm(0, ci, body0, body1, false);
            } else {
                return new (mem) btArcArcCollisionAlgorithm(0, ci, body0, body1, true);
            }
        }
    };
};




// Utility class that we use to override the btCEtriangleShape vs btCEtriangleShape

class btCEtriangleShapeCollisionAlgorithm : public btActivatingCollisionAlgorithm {
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;

  public:
    btCEtriangleShapeCollisionAlgorithm(btPersistentManifold* mf,
                                       const btCollisionAlgorithmConstructionInfo& ci,
                                       btCollisionObject* col0,
                                       btCollisionObject* col1,
                                       bool isSwapped)
        : btActivatingCollisionAlgorithm(ci, col0, col1),
          m_ownManifold(false),
          m_manifoldPtr(mf),
          m_isSwapped(isSwapped) {
        btCollisionObject* triObj1 = m_isSwapped ? col1 : col0;
        btCollisionObject* triObj2 = m_isSwapped ? col0 : col1;

        if (!m_manifoldPtr) {
            m_manifoldPtr = m_dispatcher->getNewManifold(triObj1, triObj2);
            m_ownManifold = true;
        }
    }

    btCEtriangleShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
        : btActivatingCollisionAlgorithm(ci) {}

    virtual void processCollision(btCollisionObject* body0,
                                  btCollisionObject* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) {
        (void)dispatchInfo;

        if (!m_manifoldPtr)
            return;
        
        btCollisionObject* triObj1 = m_isSwapped ? body1 : body0;
        btCollisionObject* triObj2 = m_isSwapped ? body0 : body1;

        resultOut->setPersistentManifold(m_manifoldPtr);

        // avoid persistence of contacts in manifold:
        resultOut->getPersistentManifold()->clearManifold();

        btCEtriangleShape* triA = (btCEtriangleShape*)triObj1->getCollisionShape();
        btCEtriangleShape* triB = (btCEtriangleShape*)triObj2->getCollisionShape();
        ChModelBullet* triModelA = (ChModelBullet*)triA->getUserPointer();
        ChModelBullet* triModelB = (ChModelBullet*)triB->getUserPointer();

        // brute force discard of connected triangles
        // ***TODO*** faster approach based on collision families that can bypass the
        // check at the broadphase level?
        if (triA->get_p1() == triB->get_p1() ||
            triA->get_p1() == triB->get_p2() ||
            triA->get_p1() == triB->get_p3())
            return;
        if (triA->get_p2() == triB->get_p1() ||
            triA->get_p2() == triB->get_p2() ||
            triA->get_p2() == triB->get_p3())
            return;
        if (triA->get_p3() == triB->get_p1() ||
            triA->get_p3() == triB->get_p2() ||
            triA->get_p3() == triB->get_p3())
            return;

        double max_allowed_dist = triModelA->GetEnvelope()+triModelB->GetEnvelope() +triA->sphereswept_r()+triB->sphereswept_r();
        double min_allowed_dist = - (triModelA->GetSafeMargin()+triModelB->GetSafeMargin());
        
        double offset_A = triA->sphereswept_r();
        double offset_B = triB->sphereswept_r();
        
        // Trick!! offset also by outward 'envelope' because during ReportContacts()
        // contact points are offset inward by envelope, to cope with GJK method.
        offset_A += triModelA->GetEnvelope();
        offset_B += triModelB->GetEnvelope();

        const btTransform& m44Ta = triObj1->getWorldTransform();
        const btTransform& m44Tb = triObj2->getWorldTransform();  
        const btMatrix3x3& mbtRa = m44Ta.getBasis();
        const btMatrix3x3& mbtRb = m44Tb.getBasis();
        ChMatrix33<> mRa; 
        mRa(0,0) = mbtRa[0][0]; mRa(0,1) = mbtRa[0][1]; mRa(0,2) = mbtRa[0][2];
        mRa(1,0) = mbtRa[1][0]; mRa(1,1) = mbtRa[1][1]; mRa(1,2) = mbtRa[1][2];
        mRa(2,0) = mbtRa[2][0]; mRa(2,1) = mbtRa[2][1]; mRa(2,2) = mbtRa[2][2];
        ChVector<> mOa(m44Ta.getOrigin().x(), m44Ta.getOrigin().y(), m44Ta.getOrigin().z());

        ChMatrix33<> mRb; 
        mRb(0,0) = mbtRb[0][0]; mRb(0,1) = mbtRb[0][1]; mRb(0,2) = mbtRb[0][2];
        mRb(1,0) = mbtRb[1][0]; mRb(1,1) = mbtRb[1][1]; mRb(1,2) = mbtRb[1][2];
        mRb(2,0) = mbtRb[2][0]; mRb(2,1) = mbtRb[2][1]; mRb(2,2) = mbtRb[2][2];
        ChVector<> mOb(m44Tb.getOrigin().x(), m44Tb.getOrigin().y(), m44Tb.getOrigin().z());

        // transform points to absolute coords, since models might be roto-translated
        ChVector<> pA1 = mOa + mRa * (*triA->get_p1());
        ChVector<> pA2 = mOa + mRa * (*triA->get_p2());
        ChVector<> pA3 = mOa + mRa * (*triA->get_p3());
        ChVector<> pB1 = mOb + mRb * (*triB->get_p1());
        ChVector<> pB2 = mOb + mRb * (*triB->get_p2());
        ChVector<> pB3 = mOb + mRb * (*triB->get_p3());
        
        // edges
        ChVector<> eA1 = pA2 - pA1;
        ChVector<> eA2 = pA3 - pA2;
        ChVector<> eA3 = pA1 - pA3;
        ChVector<> eB1 = pB2 - pB1;
        ChVector<> eB2 = pB3 - pB2;
        ChVector<> eB3 = pB1 - pB3;

        // normals
        ChVector<> nA = Vcross(eA1, eA2).GetNormalized();
        ChVector<> nB = Vcross(eB1, eB2).GetNormalized();

        double min_dist = 1e20;
        ChVector<> candid_pA;
        ChVector<> candid_pB;
        double dist = 1e20;
        int is_into;
        ChVector<> p_projected;
        double mu, mv;
        double candid_mu, candid_mv; 


        // Shortcut: if two degenerate 'skinny' triangles with points 2&3 coincident (ex. used to
        // represent chunks of beams) just do an edge-edge test (as capsule-capsule) and return:
        if ((pA2 == pA3) && (pB2 == pB3) && triA->owns_e1() && triB->owns_e1()) {
            ChVector<> cA, cB, D;
            if (ChCollisionUtils::LineLineIntersect(pA1, pA2,  pB1, pB2,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {    
                    _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    resultOut->refreshContactPoints();
                    return;
                }
            }
        }

        // vertex-face tests:

        if (triA->owns_v1()) {
            dist = ChCollisionUtils::PointTriangleDistance(pA1, pB1, pB2, pB3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(pA1,p_projected, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pA = pA1; candid_pB = p_projected;  candid_mu = mu; candid_mv = mv;    
                }
            }
        }
        if (triA->owns_v2()) {
            dist = ChCollisionUtils::PointTriangleDistance(pA2, pB1, pB2, pB3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(pA2,p_projected, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pA = pA2; candid_pB = p_projected;
                }
            }
        }
        if (triA->owns_v3()) {
            dist = ChCollisionUtils::PointTriangleDistance(pA3, pB1, pB2, pB3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(pA3,p_projected, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pA = pA3; candid_pB = p_projected;
                }
            }
        }

        if (triB->owns_v1()) {
            dist = ChCollisionUtils::PointTriangleDistance(pB1, pA1, pA2, pA3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(p_projected, pB1, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pB = pB1; candid_pA = p_projected;
                }
            }
        }
        if (triB->owns_v2()) {
            dist = ChCollisionUtils::PointTriangleDistance(pB2, pA1, pA2, pA3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(p_projected, pB2, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pB = pB2; candid_pA = p_projected;
                }
            }
        }
        if (triB->owns_v3()) {
            dist = ChCollisionUtils::PointTriangleDistance(pB3, pA1, pA2, pA3, mu, mv, is_into, p_projected);
            if (is_into) {
                if (dist < max_allowed_dist && dist > min_allowed_dist) {
                    _add_contact(p_projected, pB3, dist, resultOut, offset_A, offset_B);
                }
                if (dist < min_dist) {
                    min_dist = dist; candid_pB = pB3; candid_pA = p_projected;
                }
            }
        }
        double beta_A1, beta_A2, beta_A3, beta_B1, beta_B2, beta_B3; // defaults for free edge
        ChVector<> tA1, tA2, tA3, tB1, tB2, tB3;
        ChVector<> lA1, lA2, lA3, lB1, lB2, lB3;

        //  edges-edges tests

        if (triA->owns_e1()) {
            tA1 = Vcross(eA1, nA).GetNormalized();
            if(triA->get_e1())
                lA1 = (mOa + mRa * (*triA->get_e1()) ) - pA1;
            else 
                lA1 = - tA1;
            beta_A1 = atan2(Vdot(lA1, tA1), Vdot(lA1, nA) );
            if (beta_A1 < 0)
                beta_A1 += CH_C_2PI;
        }
        if (triA->owns_e2()) {
            tA2 = Vcross(eA2, nA).GetNormalized();
            if (triA->get_e2())
                lA2 = (mOa + mRa * (*triA->get_e2()) ) - pA2;
            else
                lA2 = -tA2;
            beta_A2 = atan2( Vdot(lA2, tA2), Vdot(lA2, nA) );
            if (beta_A2 < 0) 
                beta_A2 += CH_C_2PI;
        }
        if (triA->owns_e3()) {
            tA3 = Vcross(eA3, nA).GetNormalized();
            if (triA->get_e3())
                lA3 = (mOa + mRa * (*triA->get_e3()) ) - pA3;
            else
                lA3 = -tA3;
            beta_A3 = atan2( Vdot(lA3, tA3), Vdot(lA3, nA) );
            if (beta_A3 < 0) 
                beta_A3 += CH_C_2PI;
        }
        if (triB->owns_e1()) {
            tB1 = Vcross(eB1, nB).GetNormalized();
            if (triB->get_e1())
                lB1 = (mOb + mRb * (*triB->get_e1()) ) - pB1;
            else
                lB1 = -tB1;
            beta_B1 = atan2( Vdot(lB1, tB1), Vdot(lB1, nB) );
            if (beta_B1 < 0) 
                beta_B1 += CH_C_2PI;
        }
        if (triB->owns_e2()) {
            tB2 = Vcross(eB2, nB).GetNormalized();
            if (triB->get_e2())
                lB2 = (mOb + mRb * (*triB->get_e2()) ) - pB2;
            else
                lB2 = -tB2;
            beta_B2 = atan2( Vdot(lB2, tB2), Vdot(lB2, nB) );
            if (beta_B2 < 0) 
                beta_B2 += CH_C_2PI;
        }
        if (triB->owns_e3()) {
            tB3 = Vcross(eB3, nB).GetNormalized();
            if (triB->get_e3())
                lB3 = (mOb + mRb * (*triB->get_e3()) ) - pB3;
            else
                lB3 = -tB3;
            beta_B3 = atan2( Vdot(lB3, tB3), Vdot(lB3, nB) );
            if (beta_B3 < 0) 
                beta_B3 += CH_C_2PI;
        }

        ChVector<> cA, cB, D;

        double edge_tol = 1e-3;
        //  + edge_tol to discard flat edges with some tolerance:
        double beta_convex_limit = CH_C_PI_2 + edge_tol; 
        //  +/- edge_tol to inflate arc of acceptance of edge vs edge, to cope with singular cases (ex. flat cube vs flat cube):
        double alpha_lo_limit = - edge_tol;              
        double CH_C_PI_mtol   = CH_C_PI   - edge_tol;
        double CH_C_PI_2_ptol = CH_C_PI_2 + edge_tol;

        // edge A1 vs edge B1
        if (triA->owns_e1() && triB->owns_e1())
          if (beta_A1 > beta_convex_limit && beta_B1 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA1, pA2,  pB1, pB2,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {    
                    double alpha_A = atan2( Vdot( D,tA1), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB1), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A1 vs edge B2
        if (triA->owns_e1() && triB->owns_e2())
          if (beta_A1 > beta_convex_limit && beta_B2 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA1, pA2,  pB2, pB3,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA1), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB2), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A1 vs edge B3
        if (triA->owns_e1() && triB->owns_e3())
          if (beta_A1 > beta_convex_limit && beta_B3 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA1, pA2,  pB3, pB1,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA1), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB3), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A2 vs edge B1
        if (triA->owns_e2() && triB->owns_e1())
          if (beta_A2 > beta_convex_limit && beta_B1 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA2, pA3,  pB1, pB2,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA2), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB1), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A2 vs edge B2
        if (triA->owns_e2() && triB->owns_e2())
          if (beta_A2 > beta_convex_limit && beta_B2 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA2, pA3,  pB2, pB3,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA2), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB2), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A2 vs edge B3
        if (triA->owns_e2() && triB->owns_e3())
          if (beta_A2 > beta_convex_limit && beta_B3 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA2, pA3,  pB3, pB1,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA2), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB3), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A3 vs edge B1
        if (triA->owns_e3() && triB->owns_e1())
          if (beta_A3 > beta_convex_limit && beta_B1 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA3, pA1,  pB1, pB2,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA3), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB1), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A3 vs edge B2
        if (triA->owns_e3() && triB->owns_e2())
          if (beta_A3 > beta_convex_limit && beta_B2 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA3, pA1,  pB2, pB3,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA3), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB2), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
        // edge A3 vs edge B3
        if (triA->owns_e3() && triB->owns_e3())
          if (beta_A3 > beta_convex_limit && beta_B3 > beta_convex_limit ) {
            if (ChCollisionUtils::LineLineIntersect(pA3, pA1,  pB3, pB1,  &cA, &cB,  &mu, &mv) ) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu >0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2( Vdot( D,tA3), Vdot( D,nA));
                    double alpha_B = atan2( Vdot(-D,tB3), Vdot(-D,nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol && (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
       
        resultOut->refreshContactPoints();
    }

private:
    void _add_contact(const ChVector<>& candid_pA, const ChVector<>& candid_pB, const double dist, btManifoldResult* resultOut, 
                      const double offsetA, const double offsetB) {

        // convert to Bullet vectors. Note: in absolute csys.
        btVector3 absA ((btScalar)candid_pA.x(), (btScalar)candid_pA.y(), (btScalar)candid_pA.z());
        btVector3 absB ((btScalar)candid_pB.x(), (btScalar)candid_pB.y(), (btScalar)candid_pB.z());
        ChVector<> dabsN_onB ((candid_pA-candid_pB).GetNormalized());
        btVector3 absN_onB ((btScalar)dabsN_onB.x(), (btScalar)dabsN_onB.y(), (btScalar)dabsN_onB.z());
        if (dist<0)
            absN_onB = - absN_onB; // flip norm to be coherent with dist sign
        resultOut->addContactPoint(absN_onB, absB + absN_onB * (btScalar)offsetB, (btScalar)(dist - (offsetA + offsetB)));
    }

  public:

    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) {
        // not yet
        return btScalar(1.);
    }

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) {
        if (m_manifoldPtr && m_ownManifold) {
            manifoldArray.push_back(m_manifoldPtr);
        }
    }

    virtual ~btCEtriangleShapeCollisionAlgorithm() {
        if (m_ownManifold) {
            if (m_manifoldPtr)
                m_dispatcher->releaseManifold(m_manifoldPtr);
        }
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               btCollisionObject* body0,
                                                               btCollisionObject* body1) {
            void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCEtriangleShapeCollisionAlgorithm));
            if (!m_swapped) {
                return new (mem) btCEtriangleShapeCollisionAlgorithm(0, ci, body0, body1, false);
            } else {
                return new (mem) btCEtriangleShapeCollisionAlgorithm(0, ci, body0, body1, true);
            }
        }
    };
};



////////////////////////////////////
////////////////////////////////////


ChCollisionSystemBullet::ChCollisionSystemBullet(unsigned int max_objects, double scene_size) {
    // btDefaultCollisionConstructionInfo conf_info(...); ***TODO***
    bt_collision_configuration = new btDefaultCollisionConfiguration();

    bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);
    //((btDefaultCollisionConfiguration*)bt_collision_configuration)->setConvexConvexMultipointIterations(4,4);

    //***OLD***
    btScalar sscene_size = (btScalar)scene_size;
    btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
    btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
    // bt_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax, max_objects, 0, true); // true for disabling
    // raycast accelerator

    //***NEW***
    bt_broadphase = new btDbvtBroadphase();

    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

    // custom collision for sphere-sphere case ***OBSOLETE*** // already registered by btDefaultCollisionConfiguration
    // bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new
    // btSphereSphereCollisionAlgorithm::CreateFunc);

    // custom collision for cylinder-sphere case, for improved precision
/*   
    btCollisionAlgorithmCreateFunc* m_collision_sph_cyl = new btSphereCylinderCollisionAlgorithm::CreateFunc;
    btCollisionAlgorithmCreateFunc* m_collision_cyl_sph = new btSphereCylinderCollisionAlgorithm::CreateFunc;
    m_collision_cyl_sph->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE, CYLINDER_SHAPE_PROXYTYPE, m_collision_sph_cyl);
    bt_dispatcher->registerCollisionCreateFunc(CYLINDER_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, m_collision_cyl_sph);
*/ 

    // custom collision for 2D arc-segment case
    m_collision_arc_seg = new btArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc = new btArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, SEGMENT_SHAPE_PROXYTYPE, m_collision_arc_seg);
    bt_dispatcher->registerCollisionCreateFunc(SEGMENT_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_seg_arc);

     // custom collision for 2D arc-arc case
    m_collision_arc_arc = new btArcArcCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_arc_arc);

     // custom collision for C::E triangles:
    m_collision_cetri_cetri = new btCEtriangleShapeCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(CE_TRIANGLE_SHAPE_PROXYTYPE, CE_TRIANGLE_SHAPE_PROXYTYPE, m_collision_cetri_cetri);

     // custom collision for point-point case (in point clouds, just never create point-point contacts)
    //btCollisionAlgorithmCreateFunc* m_collision_point_point = new btPointPointCollisionAlgorithm::CreateFunc;
    m_tmp_mem = btAlignedAlloc(sizeof(btEmptyAlgorithm::CreateFunc),16);
    m_emptyCreateFunc = new(m_tmp_mem) btEmptyAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE, m_emptyCreateFunc);
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, bt_collision_configuration->getCollisionAlgorithmCreateFunc(SPHERE_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE)); // just for speedup
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,   POINT_SHAPE_PROXYTYPE, bt_collision_configuration->getCollisionAlgorithmCreateFunc(BOX_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE)); // just for speedup

    // custom collision for GIMPACT mesh case too
    btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);
}

ChCollisionSystemBullet::~ChCollisionSystemBullet() {
    delete bt_collision_world;
    delete bt_broadphase;
    delete bt_dispatcher;
    delete bt_collision_configuration;

    delete m_collision_arc_seg;
    delete m_collision_seg_arc;
    delete m_collision_arc_arc;
    delete m_collision_cetri_cetri;
    m_emptyCreateFunc->~btCollisionAlgorithmCreateFunc();
    btAlignedFree(m_tmp_mem);
}

void ChCollisionSystemBullet::Clear(void) {
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        contactManifold->clearManifold();
    }
}

void ChCollisionSystemBullet::Add(ChCollisionModel* model) {
    if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape()) {
        model->SyncPosition();
        bt_collision_world->addCollisionObject(((ChModelBullet*)model)->GetBulletModel(),
                                               ((ChModelBullet*)model)->GetFamilyGroup(),
                                               ((ChModelBullet*)model)->GetFamilyMask());
    }
}

void ChCollisionSystemBullet::Remove(ChCollisionModel* model) {
    if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape()) {
        bt_collision_world->removeCollisionObject(((ChModelBullet*)model)->GetBulletModel());
    }
}

void ChCollisionSystemBullet::Run() {
    if (bt_collision_world) {
        bt_collision_world->performDiscreteCollisionDetection();
    }
}

void ChCollisionSystemBullet::ResetTimers() {
    bt_collision_world->timer_collision_broad.reset();
    bt_collision_world->timer_collision_narrow.reset();
}

double ChCollisionSystemBullet::GetTimerCollisionBroad() const {
    return bt_collision_world->timer_collision_broad();
}

double ChCollisionSystemBullet::GetTimerCollisionNarrow() const {
    return bt_collision_world->timer_collision_narrow();
}

void ChCollisionSystemBullet::ReportContacts(ChContactContainer* mcontactcontainer) {
    // This should remove all old contacts (or at least rewind the index)
    mcontactcontainer->BeginAddContact();

    // NOTE: Bullet does not provide information on radius of curvature at a contact point.
    // As such, for all Bullet-identified contacts, the default value will be used (SMC only). 
    ChCollisionInfo icontact;

    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
        btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

        icontact.modelA = (ChCollisionModel*)obA->getUserPointer();
        icontact.modelB = (ChCollisionModel*)obB->getUserPointer();

        double envelopeA = icontact.modelA->GetEnvelope();
        double envelopeB = icontact.modelB->GetEnvelope();

        double marginA = icontact.modelA->GetSafeMargin();
        double marginB = icontact.modelB->GetSafeMargin();

        // Execute custom broadphase callback, if any
        bool do_narrow_contactgeneration = true;
        if (this->broad_callback)
            do_narrow_contactgeneration = this->broad_callback->OnBroadphase(icontact.modelA, icontact.modelB);

        if (do_narrow_contactgeneration) {
            int numContacts = contactManifold->getNumContacts();
            //GetLog() << "numContacts=" << numContacts << "\n";
            for (int j = 0; j < numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);

                // Discard "too far" constraints (the Bullet engine also has its threshold)
                if (pt.getDistance() < marginA + marginB) {
                    btVector3 ptA = pt.getPositionWorldOnA();
                    btVector3 ptB = pt.getPositionWorldOnB();

                    icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
                    icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

                    icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(),
                                    -pt.m_normalWorldOnB.getZ());
                    icontact.vN.Normalize();

                    double ptdist = pt.getDistance();

                    icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
                    icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
                    icontact.distance = ptdist + envelopeA + envelopeB;

                    icontact.reaction_cache = pt.reactions_cache;

                    // Execute some user custom callback, if any
                    bool add_contact = true;
                    if (this->narrow_callback)
                        add_contact = this->narrow_callback->OnNarrowphase(icontact);

                    // Add to contact container
                    if (add_contact)
                        mcontactcontainer->AddContact(icontact);
                }
            }
        }

        // you can un-comment out this line, and then all points are removed
        // contactManifold->clearManifold();
    }
    mcontactcontainer->EndAddContact();
}

void ChCollisionSystemBullet::ReportProximities(ChProximityContainer* mproximitycontainer) {
    mproximitycontainer->BeginAddProximities();
    /*
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds(); 
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
        btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

        ChCollisionModel* modelA = (ChCollisionModel*)obA->getUserPointer();
        ChCollisionModel* modelB = (ChCollisionModel*)obB->getUserPointer();

        // Add to proximity container
        mproximitycontainer->AddProximity(modelA, modelB);
    }
    */
    int numPairs = bt_collision_world->getBroadphase()->getOverlappingPairCache()->getNumOverlappingPairs();
    for (int i = 0; i < numPairs; i++) {
        btBroadphasePair mp = bt_collision_world->getBroadphase()->getOverlappingPairCache()->getOverlappingPairArray().at(i);

        btCollisionObject* obA = static_cast<btCollisionObject*>(mp.m_pProxy0->m_clientObject);
        btCollisionObject* obB = static_cast<btCollisionObject*>(mp.m_pProxy1->m_clientObject);
        
        ChCollisionModel* modelA = (ChCollisionModel*)obA->getUserPointer();
        ChCollisionModel* modelB = (ChCollisionModel*)obB->getUserPointer();

        // Add to proximity container
        mproximitycontainer->AddProximity(modelA, modelB);
    }
    mproximitycontainer->EndAddProximities();
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) const {
    return RayHit(from, to, mresult, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChRayhitResult& mresult,
                                     short int filter_group,
                                     short int filter_mask) const {
    btVector3 btfrom((btScalar)from.x(), (btScalar)from.y(), (btScalar)from.z());
    btVector3 btto((btScalar)to.x(), (btScalar)to.y(), (btScalar)to.z());

    btCollisionWorld::ClosestRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    if (rayCallback.hasHit()) {
        mresult.hitModel = (ChCollisionModel*)(rayCallback.m_collisionObject->getUserPointer());
        if (mresult.hitModel) {
            mresult.hit = true;
            mresult.abs_hitPoint.Set(rayCallback.m_hitPointWorld.x(), rayCallback.m_hitPointWorld.y(),
                                     rayCallback.m_hitPointWorld.z());
            mresult.abs_hitNormal.Set(rayCallback.m_hitNormalWorld.x(), rayCallback.m_hitNormalWorld.y(),
                                      rayCallback.m_hitNormalWorld.z());
            mresult.abs_hitNormal.Normalize();
            mresult.hit = true;
            mresult.dist_factor = rayCallback.m_closestHitFraction;
            mresult.abs_hitPoint = mresult.abs_hitPoint - mresult.abs_hitNormal * mresult.hitModel->GetEnvelope();
            return true;
        }
    }
    mresult.hit = false;
    return false;
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& mresult) const {
    return RayHit(from, to, model, mresult, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& mresult,
                                     short int filter_group,
                                     short int filter_mask) const {
    btVector3 btfrom((btScalar)from.x(), (btScalar)from.y(), (btScalar)from.z());
    btVector3 btto((btScalar)to.x(), (btScalar)to.y(), (btScalar)to.z());

    btCollisionWorld::AllHitsRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    // Find the closest hit result on the specified model (if any)
    int hit = -1;
    btScalar fraction = 1;
    for (int i = 0; i < rayCallback.m_collisionObjects.size(); ++i) {
        if (rayCallback.m_collisionObjects[i]->getUserPointer() == model && rayCallback.m_hitFractions[i] < fraction) {
            hit = i;
            fraction = rayCallback.m_hitFractions[i];
        }
    }

    // Ray does not hit specified model
    if (hit == -1) {
        mresult.hit = false;
        return false;
    }

    // Return the closest hit on the specified model
    mresult.hit = true;
    mresult.hitModel = static_cast<ChCollisionModel*>(rayCallback.m_collisionObjects[hit]->getUserPointer());
    mresult.abs_hitPoint.Set(rayCallback.m_hitPointWorld[hit].x(), rayCallback.m_hitPointWorld[hit].y(),
                             rayCallback.m_hitPointWorld[hit].z());
    mresult.abs_hitNormal.Set(rayCallback.m_hitNormalWorld[hit].x(), rayCallback.m_hitNormalWorld[hit].y(),
                              rayCallback.m_hitNormalWorld[hit].z());
    mresult.abs_hitNormal.Normalize();
    mresult.dist_factor = fraction;
    mresult.abs_hitPoint = mresult.abs_hitPoint - mresult.abs_hitNormal * mresult.hitModel->GetEnvelope();
    return true;
}

void ChCollisionSystemBullet::SetContactBreakingThreshold(double threshold) {
    gContactBreakingThreshold = (btScalar)threshold;
}

}  // end namespace collision
}  // end namespace chrono
