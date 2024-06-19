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

#ifndef CHLOADCONTACTSURFACEMESH_H
#define CHLOADCONTACTSURFACEMESH_H

#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/fea/ChLoadsNodeXYZRot.h"
#include "chrono/fea/ChContactSurfaceMesh.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Class for applying loads to a contact mesh as a cluster of forces on the nodes of the underlying finite elements.
/// Useful for cosimulation: one can pass this object's vertex & faces to an external software (e.g., CFD) that in turn
/// will perform collision detection with its entities, compute forces, send forces back to Chrono via this object.
/// Note that this is based on a cluster of ChLoadNodeXYZ, but the class itself could bypass all methods of
/// ChLoadNodeXYZ and directly implement a more efficient LoadIntLoadResidual_F.
class ChApi ChLoadContactSurfaceMesh : public ChLoadBase {
  public:
    ChLoadContactSurfaceMesh(std::shared_ptr<ChContactSurfaceMesh> contact_mesh);

    virtual ~ChLoadContactSurfaceMesh() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadContactSurfaceMesh* Clone() const override { return new ChLoadContactSurfaceMesh(*this); }

    // Functions that can be used
    // for cosimulation  A <----> B

    /// Get the collision mesh in a pointer-less way, where vertices are given in a vector of xyz points, and triangles
    /// are given as indexes to the three vertexes in that vector (similar to Wavefront OBJ meshes) Note, indexes are
    /// 0-based. These vectors can be later sent to another computing node that computes, say, CFD forces on the mesh.
    void OutputSimpleMesh(
        std::vector<ChVector3d>& vert_pos,  ///< array of vertexes (absolute xyz positions)
        std::vector<ChVector3d>& vert_vel,  ///< array of vertexes (absolute xyz velocities, might be useful)
        std::vector<ChVector3i>& triangles  ///< array of triangles (indexes to vertexes, ccw)
    );

    /// Set the forces to the nodes in a pointer-less way, where forces are given as a vector of xyz vectors and indexes
    /// to the referenced vertex, as obtained by OutputSimpleMesh.
    /// NOTE: do not insert/remove nodes from the collision mesh between the OutputSimpleMesh-InputSimpleForces pair!
    void InputSimpleForces(
        const std::vector<ChVector3d>& vert_forces,  ///< array of forces (absolute xyz forces in [N])
        const std::vector<int>& vert_ind             ///< array of indexes to vertexes to which forces are applied
    );

    /// Set the contact mesh (also resets the applied nodes)
    void SetContactMesh(std::shared_ptr<ChContactSurfaceMesh> contact_mesh);

    /// Get the contact mesh
    std::shared_ptr<ChContactSurfaceMesh> GetContactMesh() const { return m_contact_mesh; }

    /// Access the list of applied forces, so you can add new ones by using push_back(),
    /// remove them, count them, etc.
    /// Note that if you add nodes, these should belong to the referenced mesh.
    std::vector<std::shared_ptr<ChLoadNodeXYZ>>& GetForces() { return m_forces; }

  private:
    // ChLoadBase interface

    virtual int LoadGetNumCoordsPosLevel() override;
    virtual int LoadGetNumCoordsVelLevel() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    // simple.. field is x y z, hardcoded return val:
    virtual int LoadGetNumFieldCoords() override { return 3; }

    /// Compute the generalized load(s).
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Compute the K=-dQ/dx, R=-dQ/dv, M=-dQ/da Jacobians.
    /// Load the Jacobian matrices K, R, M in the structure 'm_jacobians'.
    virtual void ComputeJacobian(ChState* state_x,      ///< state position to evaluate jacobians
                                 ChStateDelta* state_w  ///< state speed to evaluate jacobians
                                 ) override;

    virtual bool IsStiff() override { return false; }

    virtual void CreateJacobianMatrices() override;
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) override;
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, double c) override {}
    virtual void LoadIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, double c) override {}
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    std::shared_ptr<ChContactSurfaceMesh> m_contact_mesh;
    std::vector<std::shared_ptr<ChLoadNodeXYZ>> m_forces;
    std::vector<std::shared_ptr<ChLoadNodeXYZRotForceAbs>> m_forces_rot;
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
