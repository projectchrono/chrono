// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Template for a simplified deformable tire.
//
// =============================================================================

#ifndef CH_LUT_TIRE_H
#define CH_LUT_TIRE_H

#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

class LUTTireModel;

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Template for a simplified deformable tire.
class CH_VEHICLE_API ChLUTTire : public ChDeformableTire {
  public:
    ChLUTTire(const std::string& name);
    virtual ~ChLUTTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "LUTTire"; }

    /// Set the tire geometry.
    void SetTireGeometry(const std::vector<double>& ring_radii,    ///<
                         const std::vector<double>& ring_offsets,  ///<
                         int num_divs                              ///< number of divisions in circumferential direction
    );

    /// Set the total tire mass.
    void SetTireMass(double mass);

    /// Set tire material properties.
    void SetTireProperties(double eR,  ///< radial elastic coefficient
                           double dR,  ///< radial damping coefficient
                           double eC,  ///< circumferential elastic coefficient
                           double dC,  ///< circumferential damping coefficient
                           double eT,  ///< transversal elastic coefficient
                           double dT   ///< transversal damping coefficient
    );

    /// Set contact material properties.
    void SetTireContactMaterial(const ChContactMaterialData& mat_data);

    /// Get the tire radius.
    virtual double GetRadius() const override final;

    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const override final;

    /// Get the tire width.
    virtual double GetWidth() const override final;

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Get the tire force and moment expressed in the tire frame.
    /// Currently *NOT IMPLEMENTED*.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    /// A derived class should also set the current slip, longitudinal slip, and camber angle.
    virtual void Synchronize(double time, const ChTerrain& terrain) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

  protected:
    /// Initialize the tire subsystem inertia properties.
    virtual void InitializeInertiaProperties() override;

    /// Update the tire subsystem inertia properties.
    virtual void UpdateInertiaProperties() override;

  private:
    // Overrides of ChDeformableTire functions
    virtual void CreateContactMaterial() override final;

    // Overrides of ChDeformableTire functions not implemented (used) by ChLUTTire
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override final {}
    virtual void CreatePressureLoad() override final {}
    virtual void CreateContactSurface() override final {}
    virtual void CreateRimConnections(std::shared_ptr<ChBody> wheel) override final {}
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const final { return {}; }

  private:
    std::shared_ptr<LUTTireModel> m_model;
    ChContactMaterialData m_contact_mat_data;
};

// =============================================================================

// Underlying implementation of the LUT tire model.
class LUTTireModel : public ChPhysicsItem {
  private:
    // Construct the LUT tire relative to the specified frame (frame of associated wheel).
    void Construct(const ChFrameMoving<>& wheel_frame);

    // Calculate COG and inertia, expressed relative to the specified frame.
    void CalculateInertiaProperties(const ChFrameMoving<>& wheel_frame, ChVector<>& com, ChMatrix33<>& inertia);

    // Calculate nodal forces (expressed in the global frame).
    void CalculateForces(const ChFrameMoving<>& wheel_frame);

    // ChPhysicsItem overrides
    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem() override;
    virtual void RemoveCollisionModelsFromSystem() override;

    virtual int GetDOF() override { return m_dofs; }
    virtual int GetDOF_w() override { return m_dofs_w; }
    virtual void SetupInitial() override;
    virtual void Setup() override;
    virtual void Update(double t, bool update_assets = true) override;

    // State functions
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntStateGetIncrement(const unsigned int off_x,
                                      const ChState& x_new,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // Get the node index from the ring index and division index.
    // Return -1 if out-of-bounds ring and use a cyclic index for divisions on the ring.
    int NodeIndex(int ir, int id);

    int m_dofs;    // total degrees of freedom
    int m_dofs_w;  // total degrees of freedom, derivative (Lie algebra)

    int m_num_rings;  // number of rings
    int m_num_divs;   // number of nodes per ring
    int m_num_nodes;  // number of nodes and visualization vertices
    int m_num_faces;  // number of visualization triangles

    double m_node_mass;  // nodal mass

    std::vector<double> m_radii;    // ring radii (initial values)
    std::vector<double> m_offsets;  // ring offsets

    double m_eR;  // radial elastic coefficient
    double m_dR;  // radial damping coefficient
    double m_eC;  // circumferential elastic coefficient
    double m_dC;  // circumferential damping coefficient
    double m_eT;  // transversal elastic coefficient
    double m_dT;  // transversal damping coefficient

    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> m_nodes;  // FEA nodes
    std::shared_ptr<fea::ChContactSurface> m_contact_surf;    // contact surface
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;     // mesh visualization asset

    ChLUTTire* m_tire;  // owner ChLUTTire object
    ChBody* m_wheel;    // associate wheel body

    friend class ChLUTTire;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
