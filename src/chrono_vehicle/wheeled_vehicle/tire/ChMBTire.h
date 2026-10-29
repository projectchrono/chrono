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
#include "chrono/solver/ChKRMBlock.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

class MBTireModel;

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Template for a multibody deformable tire.
class CH_VEHICLE_API ChMBTire : public ChDeformableTire {
  public:
    /// Statistics of the contacts between the tire contact surface and other objects.
    /// Only contacts with positive penetration are included. Penetrations are reported as positive values.
    struct ContactStats {
        int num_contacts = 0;        ///< number of active contacts
        double max_penetration = 0;  ///< maximum penetration depth
        double avg_penetration = 0;  ///< average penetration depth over the active contacts
        double normal_force = 0;     ///< sum of the normal contact force magnitudes
        ChVector3d force = VNULL;    ///< resultant contact force on the tire (global frame)
    };

    /// Interpretation of the spring stiffness and damping coefficients.
    enum class SpringCoefficientType {
        /// Coefficients are used as given for every spring of the given type, so the tire response depends on the
        /// number of divisions and on the ring spacing.
        PER_SPRING,
        /// Coefficients are distributed properties of the tire surface; the per-spring values are calculated from the
        /// mesh geometry, so the tire response does not depend on the mesh resolution:
        /// - translational springs: stiffness per unit width and unit strain (N/m); damping per unit width and unit strain
        ///   rate (N.s/m). A spring of length l covering a strip of width w gets k*w/l.
        /// - bending springs: bending moment per unit width and unit curvature (N.m); damping in N.m.s.
        ///   A bending spring at a node with tributary length l (along the bending direction) and width w gets k*w/l.
        DISTRIBUTED
    };

    ChMBTire(const std::string& name);
    virtual ~ChMBTire();

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

    /// Set the interpretation of the spring coefficients (default: DISTRIBUTED).
    void SetSpringCoefficientType(SpringCoefficientType type);

    /// Set stiffness and damping coefficients for translational mesh springs.
    void SetMeshSpringCoefficients(double kC,  ///< circumferential spring elastic coefficient
                                   double cC,  ///< circumferential spring damping coefficient
                                   double kT,  ///< transversal spring elastic coefficient
                                   double cT   ///< transversal spring damping coefficient
    );

    /// Set stiffness and damping coefficients for all rotational bending springs.
    /// The same coefficients are used for bending in the circumferential direction (along the node rings) and in the
    /// transversal direction (across the node rings, i.e., the sidewalls).
    void SetBendingSpringCoefficients(double kB,  ///< torsional spring elastic coefficient
                                      double cB   ///< torsional spring damping coefficient
    );

    /// Set stiffness and damping coefficients for rotational bending springs.
    void SetBendingSpringCoefficients(double kBC,  ///< circumferential bending spring elastic coefficient
                                      double cBC,  ///< circumferential bending spring damping coefficient
                                      double kBT,  ///< transversal bending spring elastic coefficient
                                      double cBT   ///< transversal bending spring damping coefficient
    );

    /// Set stiffness and damping coefficients for radial springs.
    void SetRadialSpringCoefficients(double kR,  ///< radial spring elastic coefficient
                                     double cR   ///< radial spring damping coefficient
    );

    /// Enable a follower pressure load (default: true).
    /// If enabled, the pressure force at a node acts along the current surface normal, with magnitude given by the
    /// pressure times the current area associated with the node. Otherwise, it acts along the direction from the wheel
    /// center to the node, with constant magnitude.
    void EnableFollowerPressure(bool val);

    /// Distribute the tire mass (and the non-follower pressure load) to the nodes in proportion to the surface area
    /// associated with each node (default: true). Otherwise, all nodes get the same mass and pressure force.
    void EnableAreaWeightedLumping(bool val);

    /// Declare the forces generated by this element as stiff (default: false).
    /// If stiff, Jacobian information will be generated.
    void IsStiff(bool val);

    /// Set contact material properties.
    void SetTireContactMaterial(const ChContactMaterialData& mat_data);

    /// Scale the contact material stiffness and damping with the mesh resolution (default: disabled).
    /// With a NODE_CLOUD contact surface, the number of nodes in the contact patch grows in proportion to the number of
    /// divisions, and so does the total contact stiffness. If a reference number of divisions is specified, the
    /// contact material Young's modulus and the normal and tangential stiffness and damping coefficients are scaled by
    /// ref_divs/num_divs, so that the specified contact material properties apply to a tire with ref_divs divisions.
    /// Note that this does not account for changes in the number of rings. A non-positive value disables scaling.
    void SetContactReferenceDivisions(int ref_divs);

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

    /// Get the tire nodes.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>>& GetGridNodes() const;

    /// Get the tire rim nodes.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>>& GetRimNodes() const;

    /// Get the tire contact surface.
    /// If contact is not enabled or the tire is not initialized, an empty shared pointer is returned.
    virtual std::shared_ptr<fea::ChContactSurface> GetContactSurface() const override;

    /// Get a triangle surface mesh over the tire grid nodes.
    /// If the tire contact surface is a triangle mesh, it is returned. Otherwise, a triangle surface mesh over the same
    /// faces is created on the first call; this surface is not used for collision.
    virtual std::shared_ptr<fea::ChContactSurfaceMesh> GetSurfaceMesh() const override;

    /// Get statistics of the current tire contacts.
    /// Use these to check that the contact compliance is small compared to the tire deflection (in which case results
    /// are insensitive to the contact stiffness). All values are zero if contact is disabled.
    ContactStats GetContactStats() const;

  protected:
    /// Initialize the tire subsystem inertia properties.
    virtual void InitializeInertiaProperties() override;

    /// Update the tire subsystem inertia properties.
    virtual void UpdateInertiaProperties() override;

  private:
    // Overrides of ChDeformableTire functions
    virtual void CreateContactMaterial() override final;

  private:
    std::shared_ptr<MBTireModel> m_model;
    ChContactMaterialData m_contact_mat_data;
    int m_contact_ref_divs;
};

// =============================================================================

// Underlying (private) implementation of the MB tire model.
class CH_VEHICLE_API MBTireModel : public ChPhysicsItem {
  public:
    MBTireModel();

  private:
    // Construct the MB tire relative to the associated wheel hub body.
    void Initialize(std::shared_ptr<ChBody> hub, ChTire::ContactSurfaceType surface_type, double surface_dim, int collision_family);

    // Create a triangle surface mesh over the grid nodes, using the specified contact material.
    std::shared_ptr<fea::ChContactSurfaceMesh> CreateSurfaceMesh(std::shared_ptr<ChContactMaterialSMC> mat, double surface_dim);

    // Calculate COG and inertia, expressed relative to the frame of the associated wheel hub body.
    void CalculateInertiaProperties(ChVector3d& com, ChMatrix33<>& inertia);

    // Set position and velocity of rim nodes from wheel hub state.
    void SetRimNodeStates();

    // Calculate nodal forces (expressed in the global frame).
    void CalculateForces();

    // ChPhysicsItem overrides
    virtual bool IsCollisionEnabled() const override { return true; }

    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    virtual unsigned int GetNumCoordsPosLevel() override { return m_dofs; }
    virtual unsigned int GetNumCoordsVelLevel() override { return m_dofs_w; }
    virtual void SetupInitial() override;
    virtual void Setup() override;
    virtual void Update(double t, UpdateFlags update_flags) override;

    // Add visualization assets for the tire subsystem.
    void AddVisualizationAssets(VisualizationType vis);

    // State functions
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void InjectKRMMatrices(ChSystemDescriptor& mdescriptor) override;

    virtual void IntStateGather(const unsigned int off_x, ChState& x, const unsigned int off_v, ChStateDelta& v, double& T) override;
    virtual void IntStateScatter(const unsigned int off_x, const ChState& x, const unsigned int off_v, const ChStateDelta& v, const double T, UpdateFlags update_flags) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;
    virtual void IntStateGetIncrement(const unsigned int off_x, const ChState& x_new, const ChState& x, const unsigned int off_v, ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L) override;

    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    // Get the node index from the ring index and division index.
    // Return -1 if out-of-bounds ring and use a cyclic index for divisions on the ring.
    int NodeIndex(int ir, int id) const;

    // Get the rim node index from the ring index and division index.
    // Used to identify rim nodes.
    // Return -1 if out-of-bounds ring and use a cyclic index for divisions on the ring.
    int RimNodeIndex(int ir, int id) const;

    // Calculate area of tire surface.
    double CalculateArea() const;

    // Calculate the geometric quantities of the undeformed mesh (segment lengths, chords, tributary widths and areas).
    void CalculateGeometry();

    // Get normal and elemental area for the node with specified ring and division indices.
    void CalculateNormal(int ir, int id, ChVector3d& normal, double& area);

    unsigned int m_dofs;    // total degrees of freedom (position level)
    unsigned int m_dofs_w;  // total degrees of freedom (velocity level)

    int m_num_rings;      // number of rings
    int m_num_divs;       // number of nodes per ring
    int m_num_rim_nodes;  // number of nodes attached to rim
    int m_num_nodes;      // number of nodes and visualization vertices
    int m_num_faces;      // number of visualization triangles

    std::vector<double> m_ring_mass;  // nodal mass for each ring

    std::vector<double> m_radii;    // ring radii (initial values)
    std::vector<double> m_offsets;  // ring offsets

    double m_rim_radius;  // rim radius

    // Undeformed mesh geometry
    std::vector<double> m_seg_len;  // lengths of the meridian segments between consecutive rings
    double m_rim_len[2];            // lengths of the meridian segments between the rim and the first and last rings
    std::vector<double> m_chord;    // distance between consecutive nodes on each ring
    double m_rim_chord;             // distance between consecutive rim nodes
    std::vector<double> m_width;    // tributary width (along the meridian) of the nodes on each ring
    std::vector<double> m_area;     // tributary area of the nodes on each ring

    ChMBTire::SpringCoefficientType m_coef_type;  // interpretation of spring coefficients

    double m_kR;   // radial elastic coefficient
    double m_cR;   // radial damping coefficient
    double m_kC;   // circumferential elastic coefficient
    double m_cC;   // circumferential damping coefficient
    double m_kT;   // transversal elastic coefficient
    double m_cT;   // transversal damping coefficient
    double m_kBC;  // circumferential bending elastic coefficient
    double m_cBC;  // circumferential bending damping coefficient
    double m_kBT;  // transversal bending elastic coefficient
    double m_cBT;  // transversal bending damping coefficient

    bool m_follower_pressure;  // pressure load along current surface normal
    bool m_area_lumping;       // mass and pressure distributed based on nodal area

    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> m_nodes;      // FEA grid nodes
    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> m_rim_nodes;  // FEA nodes fixed to the rim
    std::shared_ptr<fea::ChContactSurface> m_contact_surf;        // contact surface
    std::shared_ptr<fea::ChContactSurfaceMesh> m_surface_mesh;    // surface mesh (if not the contact surface)
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;   // mesh visualization asset

    // Auxiliary face information (for a triangle mesh surface)
    struct FaceAuxData {
        ChVector3i nbr_node;   // neighbor (opposite) vertex/node for each face vertex
        ChVector3b owns_node;  // vertex/node owned by the face?
        ChVector3b owns_edge;  // edge owned by the face?
    };
    std::vector<FaceAuxData> m_face_auxdata;  // auxiliary data for each mesh face

    // Base class for all force elements (loads) in an MBTire model.
    // A load can generate forces on 1, 2, or 3 nodes in the system.
    struct Load {
        virtual void CalculateForce() = 0;
        virtual void CalculateJacobian(double Kfactor, double Rfactor) = 0;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) = 0;
    };

    // Base class for a pressure load acting on a single FEA node.
    struct PressureLoad : public Load {
        int inode;  // index of associated node

        fea::ChNodeFEAxyz* node;

        ChBody* wheel;

        ChVector3d force;

        ChKRMBlock KRM;
        ChMatrixDynamic<double> J_fd;

        virtual void Initialize(bool stiff) = 0;
    };

    // Pressure load with constant magnitude, acting along the direction from the wheel center to the node.
    struct NodePressure : public PressureLoad {
        double p_times_a;

        virtual void Initialize(bool stiff) override;
        virtual void CalculateForce() override;
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;

        ChMatrix33<> CalculateJacobianBlock(double Kfactor, double Rfactor);
    };

    // Follower pressure load, acting along the current surface normal at the node.
    // The force is F = p/4 * (xN - xS) x (xE - xW), where N and S are the next and previous nodes on the same ring and
    // E and W are the nodes on the previous and next rings (or the rim nodes, for the first and last rings). This is the
    // pressure times the vector area of the quadrilateral with diagonals (xN - xS) and (xE - xW).
    // F does not depend on the node position. KRM variables: node (3), N (3), S (3), E (3, if grid node),
    // W (3, if grid node), wheel (6, if E or W is a rim node).
    struct FollowerPressure : public PressureLoad {
        double p4;  // pressure / 4

        fea::ChNodeFEAxyz* nodeN;
        fea::ChNodeFEAxyz* nodeS;
        fea::ChNodeFEAxyz* nodeE;  // grid node or rim node
        fea::ChNodeFEAxyz* nodeW;  // grid node or rim node
        bool rimE;                 // true if nodeE is a rim node
        bool rimW;                 // true if nodeW is a rim node
        ChVector3d local_posE;     // rim node location in wheel frame (if rimE)
        ChVector3d local_posW;     // rim node location in wheel frame (if rimW)

        virtual void Initialize(bool stiff) override;
        virtual void CalculateForce() override;
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;

        ChVector3d PosE() const;
        ChVector3d PosW() const;
    };

    // Linear spring between two FEA nodes.
    // Spring forces include both stiffness and damping.
    struct Spring2 : public Load {
        int inode1;  // index of first node
        int inode2;  // index of second node

        fea::ChNodeFEAxyz* node1;
        fea::ChNodeFEAxyz* node2;

        ChBody* wheel;

        double l0;  // spring free length
        double k;   // spring coefficient
        double c;   // damping coefficient

        ChVector3d force1;
        ChVector3d force2;

        ChKRMBlock KRM;
        ChMatrixDynamic<double> J_fd;

        void Initialize();
        virtual void CalculateForce() override;

        ChMatrix33<> CalculateJacobianBlock(double Kfactor, double Rfactor);
    };

    // Rotational spring between three FEA nodes.
    // Spring forces include only stiffness (no velocity-dependence).
    struct Spring3 : public Load {
        int inode_p;  // index of previous node
        int inode_c;  // index of central node
        int inode_n;  // index of next node

        fea::ChNodeFEAxyz* node_p;
        fea::ChNodeFEAxyz* node_c;
        fea::ChNodeFEAxyz* node_n;

        ChBody* wheel;

        ChVector3d t0;  // initial normal direction (in wheel frame)
        double a0;      // spring free angle
        double k;       // spring coefficient
        double c;       // damping coefficient

        ChVector3d force_p;
        ChVector3d force_c;
        ChVector3d force_n;

        ChKRMBlock KRM;
        ChMatrixDynamic<double> J_fd;

        void Initialize();
        virtual void CalculateForce() override;

        ChMatrixNM<double, 6, 9> CalculateJacobianBlockJ1(double Kfactor);
        ChMatrixNM<double, 6, 9> CalculateJacobianBlockJ2(double Kfactor);
    };

    // Linear spring between two grid nodes.
    // F = [ force1 ; force2 ]
    struct GridSpring2 : public Spring2 {
        void Initialize(bool stiff);
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;
    };

    // Linear spring between a wheel node and a grid node.
    // F = [ force_wheel ; torque_wheel ; force2]
    struct EdgeSpring2 : public Spring2 {
        void Initialize(bool stiff);
        virtual void CalculateForce() override;
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;

        ChMatrix33<> JacobianRotatedVector();

        ChVector3d local_pos;
        ChVector3d force_wheel;
        ChVector3d torque_wheel;
    };

    // Rotational spring between three grid nodes.
    // F = [ force_p ; force_c ; force_n ]
    struct GridSpring3 : public Spring3 {
        void Initialize(bool stiff);
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;
    };

    // Rotational spring between a wheel node and two grid nodes.
    // F = [ force_wheel ; torque_wheel ; force_c ; force_n ]
    struct EdgeSpring3 : public Spring3 {
        void Initialize(bool stiff);
        virtual void CalculateForce() override;
        virtual void CalculateJacobian(double Kfactor, double Rfactor) override;
        virtual void CalculateJacobianFD(double Kfactor, double Rfactor) override;

        ChMatrix33<> JacobianRotatedVector();

        ChVector3d local_pos;
        ChVector3d force_wheel;
        ChVector3d torque_wheel;
    };

    std::vector<std::shared_ptr<PressureLoad>> m_pressure_loads;   // node pressure loads
    std::vector<std::shared_ptr<GridSpring2>> m_grid_lin_springs;  // node-node translational springs
    std::vector<std::shared_ptr<EdgeSpring2>> m_edge_lin_springs;  // node-rim translational springs
    std::vector<std::shared_ptr<GridSpring3>> m_grid_rot_springs;  // node-node torsional springs
    std::vector<std::shared_ptr<EdgeSpring3>> m_edge_rot_springs;  // node-rim torsional springs

    int m_num_pressure_loads;    // number of node pressure loads
    int m_num_grid_lin_springs;  // number of node-node translational springs
    int m_num_edge_lin_springs;  // number of node-rim translational springs (first and last ring)
    int m_num_grid_rot_springs;  // number of node-node torsional springs
    int m_num_edge_rot_springs;  // number of node-rim torsional springs (first and last ring)

    std::vector<std::shared_ptr<Load>> m_loads;  // all loads in the system
    int m_num_loads;                             // number of all loads in the system

    std::shared_ptr<ChBody> m_hub;  // associated wheel hub body
    ChVector3d m_hub_force;         // applied wheel hub force (in global frame)
    ChVector3d m_hub_torque;        // applied wheel hub torque (in body frame)

    bool m_stiff;  // true if loads are stiff (triggers Jacobian calculation)

    ChMBTire* m_tire;  // owner ChMBTire object

    friend class ChMBTire;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
