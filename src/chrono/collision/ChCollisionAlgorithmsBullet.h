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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_COLLISION_ALGORITHMS_BULLET_H
#define CH_COLLISION_ALGORITHMS_BULLET_H

#include "chrono/core/ChVector.h"

#include "chrono/collision/bullet/btBulletCollisionCommon.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_bullet
/// @{

// ================================================================================================

/// Custom override of the default Bullet algorithm for capsule-box collision.
class btCapsuleBoxCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btCapsuleBoxCollisionAlgorithm(btPersistentManifold* mf,
                                   const btCollisionAlgorithmConstructionInfo& ci,
                                   const btCollisionObjectWrapper* col0,
                                   const btCollisionObjectWrapper* col1,
                                   bool isSwapped);
    btCapsuleBoxCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btCapsuleBoxCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for cylshell-box collision.
class btCylshellBoxCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btCylshellBoxCollisionAlgorithm(btPersistentManifold* mf,
                                    const btCollisionAlgorithmConstructionInfo& ci,
                                    const btCollisionObjectWrapper* col0,
                                    const btCollisionObjectWrapper* col1,
                                    bool isSwapped);
    btCylshellBoxCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btCylshellBoxCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for sphere-cylinder collision.
/// This replaces the default GJK algorithm in Bullet which is inaccurate if the cylinder is much larger than the sphere.
class btSphereCylinderCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btSphereCylinderCollisionAlgorithm(btPersistentManifold* mf,
                                       const btCollisionAlgorithmConstructionInfo& ci,
                                       const btCollisionObjectWrapper* col0,
                                       const btCollisionObjectWrapper* col1,
                                       bool isSwapped);
    btSphereCylinderCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btSphereCylinderCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for 2Dsegment-2Darc collision.
/// Note: works only if the two are coplanar.
class btArcSegmentCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btArcSegmentCollisionAlgorithm(btPersistentManifold* mf,
                                   const btCollisionAlgorithmConstructionInfo& ci,
                                   const btCollisionObjectWrapper* col0,
                                   const btCollisionObjectWrapper* col1,
                                   bool isSwapped);
    btArcSegmentCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btArcSegmentCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for 2Darc-2Darc collision.
/// Note: works only if the two are coplanar.
class btArcArcCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btArcArcCollisionAlgorithm(btPersistentManifold* mf,
                               const btCollisionAlgorithmConstructionInfo& ci,
                               const btCollisionObjectWrapper* col0,
                               const btCollisionObjectWrapper* col1,
                               bool isSwapped);
    btArcArcCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btArcArcCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for triangle-triangle collision.
class btCEtriangleShapeCollisionAlgorithm : public btActivatingCollisionAlgorithm {
  public:
    btCEtriangleShapeCollisionAlgorithm(btPersistentManifold* mf,
                                        const btCollisionAlgorithmConstructionInfo& ci,
                                        const btCollisionObjectWrapper* col0,
                                        const btCollisionObjectWrapper* col1,
                                        bool isSwapped);
    btCEtriangleShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);
    ~btCEtriangleShapeCollisionAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0,
                                  const btCollisionObjectWrapper* body1,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut) override;
    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                           btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(btManifoldArray& manifoldArray) override;

    struct CreateFunc : public btCollisionAlgorithmCreateFunc {
        virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* body0Wrap,
                                                               const btCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    void _add_contact(const ChVector<>& candid_pA,
                      const ChVector<>& candid_pB,
                      const double dist,
                      btManifoldResult* resultOut,
                      const double offsetA,
                      const double offsetB);

    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

/// @} collision_bullet

}  // namespace collision
}  // namespace chrono

#endif
