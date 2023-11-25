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
// Template for a multibody deformable tire.
//
// =============================================================================

#ifndef CH_MB_TIRE_H
#define CH_MB_TIRE_H

#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

class MBTireModel;

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Template for a multibody deformable tire.
class CH_VEHICLE_API ChMBTire : public ChDeformableTire {
  public:
    ChMBTire(const std::string& name);
    virtual ~ChMBTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "MBTire"; }

    /// Set the tire geometry.
    void SetTireGeometry(const std::vector<double>& ring_radii,    ///< radii of node rings
                         const std::vector<double>& ring_offsets,  ///< transversal offsets of node rings
                         int num_divs,                             ///< number of divisions in circumferential direction
                         double rim_radius                         ///< radius of wheel rim
    );

    /// Set the total tire mass.
    void SetTireMass(double mass);

    /// Set stiffness and damping coefficients for translational mesh springs.
    void SetMeshSpringCoefficients(double kC,  ///< circumferential spring elastic coefficient
                                   double cC,  ///< circumferential spring damping coefficient
                                   double kT,  ///< transversal spring elastic coefficient
                                   double cT   ///< transversal spring damping coefficient
    );

    /// Set stiffness and damping coefficients for rotational bending springs.
    void SetBendingSpringCoefficients(double kB,  ///< torsional spring elastic coefficient
                                      double cB   ///< torsional spring damping coefficient
    );

    /// Set stiffness and damping coefficients for radial springs.
    void SetRadialSpringCoefficients(double kR,  ///< radial spring elastic coefficient
                                     double cR   ///< radial spring damping coefficient
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
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

    /// Add visualization assets for the tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the tire subsystem.
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

    // Overrides of ChDeformableTire functions not implemented (used) by ChMBTire
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override final {}
    virtual void CreatePressureLoad() override final {}
    virtual void CreateContactSurface() override final {}
    virtual void CreateRimConnections(std::shared_ptr<ChBody> wheel) override final {}
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const final { return {}; }

  private:
    std::shared_ptr<MBTireModel> m_model;
    ChContactMaterialData m_contact_mat_data;
};

// =============================================================================

// Underlying (private) implementation of the MB tire model.
class MBTireModel : public ChPhysicsItem {
  private:
    // Construct the MB tire relative to the specified frame (frame of associated wheel).
    void Construct(const ChFrameMoving<>& wheel_frame);

    // Calculate COG and inertia, expressed relative to the specified frame.
    void CalculateInertiaProperties(const ChFrameMoving<>& wheel_frame, ChVector<>& com, ChMatrix33<>& inertia);

    // Calculate nodal forces (expressed in the global frame).
    void CalculateForces(const ChFrameMoving<>& wheel_frame);

    // ChPhysicsItem overrides
    virtual bool GetCollide() const override { return true; }

    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    virtual int GetDOF() override { return m_dofs; }
    virtual int GetDOF_w() override { return m_dofs_w; }
    virtual void SetupInitial() override;
    virtual void Setup() override;
    virtual void Update(double t, bool update_assets = true) override;

    // Add visualization assets for the tire subsystem.
    void AddVisualizationAssets(VisualizationType vis);

    // State functions
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;

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

    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    // Get the node index from the ring index and division index.
    // Return -1 if out-of-bounds ring and use a cyclic index for divisions on the ring.
    int NodeIndex(int ir, int id);

    // Get the rim node index from the ring index and division index.
    // Used to identify rim nodes.
    // Return -1 if out-of-bounds ring and use a cyclic index for divisions on the ring.
    int RimNodeIndex(int ir, int id);

    // Get normal and elemental area for the node with current ring and division indices.
    void CalcNormal(int ir, int id, ChVector<>& normal, double& area);

    int m_dofs;    // total degrees of freedom
    int m_dofs_w;  // total degrees of freedom, derivative (Lie algebra)

    int m_num_rings;      // number of rings
    int m_num_divs;       // number of nodes per ring
    int m_num_rim_nodes;  // number of nodes attached to rim
    int m_num_nodes;      // number of nodes and visualization vertices
    int m_num_faces;      // number of visualization triangles

    double m_node_mass;  // nodal mass

    std::vector<double> m_radii;    // ring radii (initial values)
    std::vector<double> m_offsets;  // ring offsets

    double m_rim_radius;  // rim radius

    double m_kR;  // radial elastic coefficient
    double m_cR;  // radial damping coefficient
    double m_kC;  // circumferential elastic coefficient
    double m_cC;  // circumferential damping coefficient
    double m_kT;  // transversal elastic coefficient
    double m_cT;  // transversal damping coefficient
    double m_kB;  // bending elastic coefficient
    double m_cB;  // bending damping coefficient

    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> m_rim_nodes;  // FEA nodes fixed to the rim
    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> m_nodes;      // FEA nodes
    std::shared_ptr<fea::ChContactSurface> m_contact_surf;        // contact surface
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;   // mesh visualization asset

    enum class SpringType { RADIAL, CIRCUMFERENTIAL, TRANSVERSAL };

    struct Spring2 {
        SpringType type;  // spring type
        int node1;        // index of first node
        int node2;        // index of second node
        double l0;        // spring free length
        double k;         // spring coefficient
        double c;         // damping coefficient
    };

    struct Spring3 {
        int node_c;  // index of central node
        int node_p;  // index of previous node
        int node_n;  // index of next node
        double a0;   // spring free angle
        double k;    // spring coefficient
        double c;    // damping coefficient
    };

    std::vector<Spring2> m_mesh_lin_springs;  // node-node translational springs
    std::vector<Spring2> m_edge_lin_springs;  // node-rim translational springs (first and last ring)
    std::vector<Spring3> m_mesh_rot_springs;  // node-node torsional springs
    std::vector<Spring3> m_edge_rot_springs;  // node-rim torsional springs (first and last ring)

    ChVector<> m_wheel_force;   // applied wheel spindle force
    ChVector<> m_wheel_torque;  // applied wheel spindle torque

    ChMBTire* m_tire;  // owner ChMBTire object
    ChBody* m_wheel;   // associated wheel body

    friend class ChMBTire;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
