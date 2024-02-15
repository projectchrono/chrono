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
// Authors: Kassem Mohamad, Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHELEMENTBEAMIGA_H
#define CHELEMENTBEAMIGA_H

//#define BEAM_VERBOSE

#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/geometry/ChBasisToolsBspline.h"
#include "chrono/core/ChQuadrature.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Isogeometric formulation (IGA) of a Cosserat rod, with large displacements, based on the Geometrically Exact Beam
/// Theory. User-defined order n (ex: 1=linear 2=quadratic, 3=cubic), where each element is a span of a b-spline, so
/// each element uses n+1 control points, ie. nodes of chrono::fea::ChNodeFEAxyzrot type. As a thick beam, shear effects
/// are possible, v. Timoshenko theory. Reduced integration to correct shear locking (*note, use order 1 for the moment,
/// this must be improved) Initial curved configuration is supported. The section is defined in a modular way, via a
/// chrono::fea::ChBeamSectionCosserat object that is composed via an elastic model, an inertial model, a damping
/// (optional) model, a plastic (optional) model. Some of the ready-to-use implementation of those models allow a very
/// generic beam where the center of mass, center of shear etc. are arbitrarily offset from the beam centerline, thus
/// allowing the simulation of advanced cases like helicopter blades etc.

class ChApi ChElementBeamIGA : public ChElementBeam, public ChLoadableU, public ChLoadableUVW {
  public:
    ChElementBeamIGA();

    ~ChElementBeamIGA() {}

    // To avoid issues with unique_ptr of ChBeamMaterialInternalData and declspec(dllexport), one must
    // prevent automatic creation of copy constructors - but maybe in future provide custom ones
    ChElementBeamIGA(const ChElementBeamIGA&) = delete;
    ChElementBeamIGA& operator=(const ChElementBeamIGA&) = delete;

    virtual int GetNnodes() override { return (int)nodes.size(); }
    virtual int GetNdofs() override { return GetNnodes() * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }
    virtual std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetNodes() { return nodes; }

    virtual void SetNodesCubic(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeD,
                               double knotA1,
                               double knotA2,
                               double knotB1,
                               double knotB2,
                               double knotB3,
                               double knotB4,
                               double knotB5,
                               double knotB6);

    virtual void SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,
                                      std::vector<double> myknots,
                                      int myorder);

    /// Set the integration points, for shear components and for bending components:
    void SetIntegrationPoints(int npoints_s, int npoints_b);

    //
    // FEM functions
    //

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionCosserat> my_material) { section = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionCosserat> GetSection() { return section; }

    /// Access the local knot sequence of this element (ex.for diagnostics)
    ChVectorDynamic<>& GetKnotSequence() { return this->knots; }

    /// Get the parametric coordinate at the beginning of the span
    double GetU1() { return knots(order); }
    /// Get the parametric coordinate at the end of the span
    double GetU2() { return knots(knots.size() - order - 1); }

    virtual void Update() override {
        // parent class update:
        ChElementGeneric::Update();
    };

    /// Get the plastic data, in a vector with as many elements as Gauss points.
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& GetPlasticData() { return plastic_data; }

    /// Get the stress, as cut-force [N], in a vector with as many elements as Gauss points.
    std::vector<ChVector<>>& GetStressN() { return this->stress_n; }

    /// Get the stress, as cut-torque [Nm], in a vector with as many elements as Gauss points.
    std::vector<ChVector<>>& GetStressM() { return this->stress_m; }

    /// Get the strain (total=elastic+plastic), as deformation (x is axial strain), in a vector with as many elements as
    /// Gauss points.
    std::vector<ChVector<>>& GetStrainE() { return this->strain_e; }

    /// Get the strain (total=elastic+plastic), as curvature (x is torsion), in a vector with as many elements as Gauss
    /// points.
    std::vector<ChVector<>>& GetStrainK() { return this->strain_k; }

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override {
        mD.resize((int)this->nodes.size() * 7);

        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(i * 7 + 0, 3) = nodes[i]->coord.pos.eigen();
            mD.segment(i * 7 + 3, 4) = nodes[i]->coord.rot.eigen();
        }
    }

    /// Add contribution of element inertia to total nodal masses
    virtual void ComputeNodalMass() override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    void ComputeInternalForces_impl(ChVectorDynamic<>& Fi,                 ///< output vector of internal forces
                                    ChState& state_x,                      ///< state position to evaluate Fi
                                    ChStateDelta& state_w,                 ///< state speed to evaluate Fi
                                    bool used_for_differentiation = false  ///< used during FD Jacobian evaluation?
    );

    /// Compute gravity forces, grouped in the Fg vector, one node after the other
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) override;

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) override {
        ChMatrixDynamic<> N(1, (int)nodes.size());

        /* To be completed: Created to be consistent with base class implementation*/
    }

    /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionPoint(const double eta, ChVector<>& point) {
        // compute parameter in knot space from eta-1..+1
        double u1 = knots(order);  // extreme of span
        double u2 = knots(knots.size() - order - 1);
        double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());

        geometry::ChBasisToolsBspline::BasisEvaluate(this->order, nspan, u, knots,
                                                     N);  ///< here return  in N

        point = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            point += N(i) * nodes[i]->coord.pos;
        }
    }

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) override {
        // compute parameter in knot space from eta-1..+1

        double u1 = knots(order);  // extreme of span
        double u2 = knots(knots.size() - order - 1);
        double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());

        geometry::ChBasisToolsBspline::BasisEvaluate(this->order, nspan, u, knots,
                                                     N);  ///< here return  in N

        point = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            point += N(i) * nodes[i]->coord.pos;
        }
        rot = QNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> myrot = nodes[i]->coord.rot;
            rot.e0() += N(i) * myrot.e0();
            rot.e1() += N(i) * myrot.e1();
            rot.e2() += N(i) * myrot.e2();
            rot.e3() += N(i) * myrot.e3();
        }
        rot.Normalize();
    }

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) override {
        /* To be completed: Created to be consistent with base class implementation*/
    }

    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV) override {
        /* To be completed: Created to be consistent with base class implementation*/
    }

    //
    // Functions for interfacing to the solver
    //            (***most not needed, thank to bookkeeping in parent class ChElementGeneric)

    virtual void EleDoIntegration() override {
        for (size_t i = 0; i < this->plastic_data.size(); ++i) {
            this->plastic_data_old[i]->Copy(*this->plastic_data[i]);
        }
    }

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return (int)nodes.size() * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return (int)nodes.size() * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(block_offset + i * 7 + 0, 3) = this->nodes[i]->GetPos().eigen();
            mD.segment(block_offset + i * 7 + 3, 4) = this->nodes[i]->GetRot().eigen();
        }
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(block_offset + i * 6 + 0, 3) = this->nodes[i]->GetPos_dt().eigen();
            mD.segment(block_offset + i * 6 + 3, 3) = this->nodes[i]->GetWvel_loc().eigen();
        }
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i * 7, x_new, x, off_v + i * 6, Dv);
        }
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return (int)nodes.size(); }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !nodes[nblock]->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mvars.push_back(&this->nodes[i]->Variables());
        }
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinates of the line, ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< eta parametric coordinate in line -1..+1
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override {
        return this->section->GetInertia()->GetMassPerUnitLength();
    }  // this->section->GetArea()* this->section->GetDensity();

    ///  For testing purposes:
    enum class QuadratureType { FULL_OVER, FULL_EXACT, REDUCED, SELECTIVE, CUSTOM1, URI2 };

    ///  For testing purposes:
    static QuadratureType quadrature_type;

    ///  For testing purposes:
    static double Delta;

    /// Set if the element mass matrix is computed in lumped or consistent way
    static bool lumped_mass;

    /// Set if the element forces will include the gyroscopic and centrifugal terms (slower performance, but might be
    /// needed esp. when center of mass is offset)
    static bool add_gyroscopic_terms;

  private:
    /// Initial setup. Precompute mass and matrices that do not change during the simulation. In particular, compute the
    /// arc-length parametrization.
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;  // also "control points"
    ChVectorDynamic<> knots;

    int order;

    int int_order_s;
    int int_order_b;

    std::vector<double> Jacobian_s;
    std::vector<double> Jacobian_b;

    std::vector<ChVector<>> strain_e_0;
    std::vector<ChVector<>> strain_k_0;

    std::vector<ChVector<>> stress_n;
    std::vector<ChVector<>> stress_m;
    std::vector<ChVector<>> strain_e;
    std::vector<ChVector<>> strain_k;

    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> plastic_data_old;
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> plastic_data;

    std::shared_ptr<ChBeamSectionCosserat> section;

    friend class ChExtruderBeamIGA;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
