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

/// Class for applying loads to a triangle mesh belonging to a ChBody, as a cluster of forces operating on the
/// underlying rigid body. It is useful for doing cosimulation: one can pass this object's vertex & faces to an external
/// software (ex. CFD) that in turn will perform collision detection with its entities, compute forces, send forces back
/// to Chrono via this object.
/// Note, this is based on a cluster of std::vector<std::shared_ptr<ChLoadBodyForce>>, but the class itself could bypass
/// all methods of ChLoadBodyForce and directly implement a more efficient LoadIntLoadResidual_F.

class ChApi ChLoadBodyMesh : public ChLoadBase {
  public:
    ChLoadBodyMesh(std::shared_ptr<ChBody> cbody, geometry::ChTriangleMeshConnected& cmesh);

    virtual ~ChLoadBodyMesh() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadBodyMesh* Clone() const override { return new ChLoadBodyMesh(*this); }

    // Functions that can be used for cosimulation  A <----> B

    /// Get the collision mesh where vertex are given in a vector of xyz points, expressed in absolute coordinates, and
    /// triangles are given as indexes to the three vertexes in that vector (similar to Wavefront OBJ meshes). Note that
    /// indexes are 0-based. These vectors can be later sent to another computing node that computes, say, CFD forces on
    /// the mesh.
    void OutputSimpleMesh(
        std::vector<ChVector<>>& vert_pos,     ///< array of vertexes (absolute xyz positions)
        std::vector<ChVector<>>& vert_vel,     ///< array of vertexes (absolute xyz velocities, might be useful)
        std::vector<ChVector<int>>& triangles  ///< array of triangles (indexes to vertexes, ccw)
    );

    /// Set the forces to the body, where forces are given as a vector of xyz vectors (expressed in absolute
    /// coordinates) and indexes to the referenced vertex, as obtained by OutputSimpleMesh.
    /// NOTE: do not insert/remove nodes from the collision mesh between the OutputSimpleMesh-InputSimpleForces pair!
    void InputSimpleForces(const std::vector<ChVector<>> vert_forces,  ///< array of forces (in absolute frame)
                           const std::vector<int> vert_ind             ///< indexes of vertices with applied forces
    );

    /// Set the contact mesh (also resets the applied nodes).
    void SetContactMesh(geometry::ChTriangleMeshConnected& mmesh);

    /// Get the contact mesh.
    geometry::ChTriangleMeshConnected& GetContactMesh() { return this->contactmesh; }

    /// Access the list of applied forces, to allow adding new ones, removing them, counting them, etc.
    /// Note that only nodes from the reference mesh should be added.
    std::vector<std::shared_ptr<ChLoadBodyForce>>& GetForceList() { return forces; }

    //
    // ChLoadBase interface
    //

    virtual int LoadGet_ndof_x() override;
    virtual int LoadGet_ndof_w() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    // simple.. field is x y z, hardcoded return val:
    virtual int LoadGet_field_ncoords() override { return 3; }

    /// Compute Q, the generalized load.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Compute jacobians.
    /// Not needed when forces are constant, btw.
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrixRef mK,         ///< result dQ/dx
                                 ChMatrixRef mR,         ///< result dQ/dv
                                 ChMatrixRef mM          ///< result dQ/da
                                 ) override;

    virtual bool IsStiff() override { return false; }

    virtual void CreateJacobianMatrices() override;
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) override;
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, double c) override {}
    virtual void LoadIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, double c) override {}
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

  private:
    std::shared_ptr<ChBody> contactbody;
    geometry::ChTriangleMeshConnected contactmesh;

    std::vector<std::shared_ptr<ChLoadBodyForce>> forces;
};

}  // end namespace chrono

#endif
