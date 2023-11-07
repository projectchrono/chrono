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

#include "chrono/collision/bullet/cbtBulletCollisionCommon.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtEmptyCollisionAlgorithm.h"

namespace chrono {

/// @addtogroup collision_bullet
/// @{

// ================================================================================================

/// Custom override of the default Bullet algorithm for capsule-box collision.
class cbtCapsuleBoxCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtCapsuleBoxCollisionAlgorithm(cbtPersistentManifold* mf,
                                    const cbtCollisionAlgorithmConstructionInfo& ci,
                                    const cbtCollisionObjectWrapper* col0,
                                    const cbtCollisionObjectWrapper* col1,
                                    bool isSwapped);
    cbtCapsuleBoxCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtCapsuleBoxCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for cylshell-box collision.
class cbtCylshellBoxCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtCylshellBoxCollisionAlgorithm(cbtPersistentManifold* mf,
                                     const cbtCollisionAlgorithmConstructionInfo& ci,
                                     const cbtCollisionObjectWrapper* col0,
                                     const cbtCollisionObjectWrapper* col1,
                                     bool isSwapped);
    cbtCylshellBoxCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtCylshellBoxCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for sphere-cylinder collision.
/// This replaces the default GJK algorithm in Bullet which is inaccurate if the cylinder is much larger than the
/// sphere.
class cbtSphereCylinderCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtSphereCylinderCollisionAlgorithm(cbtPersistentManifold* mf,
                                        const cbtCollisionAlgorithmConstructionInfo& ci,
                                        const cbtCollisionObjectWrapper* col0,
                                        const cbtCollisionObjectWrapper* col1,
                                        bool isSwapped);
    cbtSphereCylinderCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtSphereCylinderCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for 2Dsegment-2Darc collision.
/// Note: works only if the two are coplanar.
class cbtArcSegmentCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtArcSegmentCollisionAlgorithm(cbtPersistentManifold* mf,
                                    const cbtCollisionAlgorithmConstructionInfo& ci,
                                    const cbtCollisionObjectWrapper* col0,
                                    const cbtCollisionObjectWrapper* col1,
                                    bool isSwapped);
    cbtArcSegmentCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtArcSegmentCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for 2Darc-2Darc collision.
/// Note: works only if the two are coplanar.
class cbtArcArcCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtArcArcCollisionAlgorithm(cbtPersistentManifold* mf,
                                const cbtCollisionAlgorithmConstructionInfo& ci,
                                const cbtCollisionObjectWrapper* col0,
                                const cbtCollisionObjectWrapper* col1,
                                bool isSwapped);
    cbtArcArcCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtArcArcCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

// ================================================================================================

/// Custom override of the default Bullet algorithm for triangle-triangle collision.
class cbtCEtriangleShapeCollisionAlgorithm : public cbtActivatingCollisionAlgorithm {
  public:
    cbtCEtriangleShapeCollisionAlgorithm(cbtPersistentManifold* mf,
                                         const cbtCollisionAlgorithmConstructionInfo& ci,
                                         const cbtCollisionObjectWrapper* col0,
                                         const cbtCollisionObjectWrapper* col1,
                                         bool isSwapped);
    cbtCEtriangleShapeCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);
    ~cbtCEtriangleShapeCollisionAlgorithm();

    virtual void processCollision(const cbtCollisionObjectWrapper* body0,
                                  const cbtCollisionObjectWrapper* body1,
                                  const cbtDispatcherInfo& dispatchInfo,
                                  cbtManifoldResult* resultOut) override;
    virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0,
                                            cbtCollisionObject* body1,
                                            const cbtDispatcherInfo& dispatchInfo,
                                            cbtManifoldResult* resultOut) override;
    virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) override;

    struct CreateFunc : public cbtCollisionAlgorithmCreateFunc {
        virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci,
                                                                const cbtCollisionObjectWrapper* body0Wrap,
                                                                const cbtCollisionObjectWrapper* body1Wrap) override;
    };

  private:
    void _add_contact(const ChVector<>& candid_pA,
                      const ChVector<>& candid_pB,
                      const double dist,
                      cbtManifoldResult* resultOut,
                      const double offsetA,
                      const double offsetB);

    bool m_ownManifold;
    cbtPersistentManifold* m_manifoldPtr;
    bool m_isSwapped;
};

/// @} collision_bullet

}  // namespace chrono

#endif
