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

#ifndef CHLOADBODYMESH_H
#define CHLOADBODYMESH_H

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLoadsBody.h"

namespace chrono {

/// Loads applied to a triangle mesh associated with a ChBody, as a cluster of forces operating on the rigid body.
/// This is useful for data exchange during co-simulation in a force-displacement setting. The states (positions and
/// velocities) of the mesh vertices are extracted from the Chrono simulation (consistent with the kinematics of the
/// asociated body). Forces on the mesh vertices, computed externally, are then applied to the associated rigid body
/// using this object. Note that this class is based on a cluster of std::vector<std::shared_ptr<ChLoadBodyForce>>, but
/// the class itself could bypass all methods of ChLoadBodyForce and directly implement a more efficient
/// LoadIntLoadResidual_F.
class ChApi ChLoadBodyMesh : public ChLoadBase {
  public:
    ChLoadBodyMesh(std::shared_ptr<ChBody> body, const ChTriangleMeshConnected& mesh);

    virtual ~ChLoadBodyMesh() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyMesh* Clone() const override { return new ChLoadBodyMesh(*this); }

    // Functions that can be used for cosimulation  A <----> B

    /// Get the collision mesh where vertex are given in a vector of xyz points, expressed in absolute coordinates, and
    /// triangles are given as indexes to the three vertexes in that vector (similar to Wavefront OBJ meshes). Note that
    /// indexes are 0-based. These vectors can be later sent to another computing node that computes, say, CFD forces on
    /// the mesh.
    void OutputSimpleMesh(std::vector<ChVector3d>& vert_pos,  ///< array of vertex aabsolute xyz positions
                          std::vector<ChVector3d>& vert_vel,  ///< array of vertex absolute xyz velocities
                          std::vector<ChVector3i>& triangles  ///< array of triangles (vertex indices, ccw)
    );

    /// Set the forces to the body, where forces are given as a vector of xyz vectors (expressed in absolute
    /// coordinates) and indexes to the referenced vertex, as obtained by OutputSimpleMesh.
    /// NOTE: do not insert/remove nodes from the collision mesh between the OutputSimpleMesh-InputSimpleForces pair!
    void InputSimpleForces(const std::vector<ChVector3d> vert_forces,  ///< array of forces (in absolute frame)
                           const std::vector<int> vert_indices         ///< indices of vertices with applied forces
    );

    /// Set the contact mesh (also resets the applied nodes).
    void SetContactMesh(const ChTriangleMeshConnected& mesh);

    /// Get the contact mesh.
    ChTriangleMeshConnected& GetContactMesh() { return m_mesh; }

    /// Access the list of applied forces, to allow adding new ones, removing them, counting them, etc.
    /// Note that only nodes from the reference mesh should be added.
    std::vector<std::shared_ptr<ChLoadBodyForce>>& GetForces() { return forces; }

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

  private:
    std::shared_ptr<ChBody> m_body;                        ///< associated rigid body
    ChTriangleMeshConnected m_mesh;                        ///< triangular mesh attached to body
    std::vector<std::shared_ptr<ChLoadBodyForce>> forces;  ///< forces at mesh vertices
};

}  // end namespace chrono

#endif
