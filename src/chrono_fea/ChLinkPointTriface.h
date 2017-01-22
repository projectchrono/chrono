// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLINKPOINTTRIFACE_H
#define CHLINKPOINTTRIFACE_H

#include <array>

#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono_fea/ChLinkInterface.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Utility class for using the ChLinkPointTriface constraint
class ChApiFea ChTriangleOfXYZnodes : public ChVariableTupleCarrier_3vars<3, 3, 3> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB1;
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB2;
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB3;

    virtual ChVariables* GetVariables1() override { return &mnodeB1->Variables(); }
    virtual ChVariables* GetVariables2() override { return &mnodeB2->Variables(); }
    virtual ChVariables* GetVariables3() override { return &mnodeB3->Variables(); }
};

/// Class for creating a constraint between a xyz FEA node (point)
/// and a triangular face given by three xyz FEA nodes, with linear
/// shape function (ex. the face of a tetrahedron or a triangular shell)
/// The node can be offset respect to the face.

class ChApiFea ChLinkPointTriface : public ChLinkInterface {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkPointTriface)

  private:
    ChVector<> react;

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZnodes> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZnodes> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZnodes> constraint3;

    std::shared_ptr<ChNodeFEAxyz> mnodeA;
    ChTriangleOfXYZnodes mtriangle;

    double s2, s3;
    double d;

  public:
    ChLinkPointTriface();
    ChLinkPointTriface(const ChLinkPointTriface& other);
    ~ChLinkPointTriface() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointTriface* Clone() const override { return new ChLinkPointTriface(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 3 + 3 + 3; }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 3; }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() override { return GetReactionOnNode(); }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    virtual ChCoordsys<> GetLinkAbsoluteCoords() override { return ChCoordsys<>(mnodeA->GetPos()); }

    /// Use this function after object creation, to initialize it, given
    /// the node and the triangle to join.
    /// The attachment position is the actual position of the node.
    /// Note, nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,   ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyz> anodeB1,  ///< triangle: corner n.1
                           std::shared_ptr<ChNodeFEAxyz> anodeB2,  ///< triangle: corner n.2
                           std::shared_ptr<ChNodeFEAxyz> anodeB3   ///< triangle: corner n.3
                           );

    /// Set the area coordinates to specify where the point A is connected on triangle.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms2, const double ms3) {
        s2 = ms2;
        s3 = ms3;
    }

    /// Get the area coordinates, as set respect to B2 and B3
    ///(the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms2, double& ms3) const {
        ms2 = s2;
        ms3 = s3;
    }

    /// Set an optional offset of point A respect to triangle, along triangle normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of point A respect to triangle
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNodeA() const { return this->mnodeA; }

    /// Get the connected triangle
    std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 3> GetConstrainedTriangle() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 3>{
            {this->mtriangle.mnodeB1, this->mtriangle.mnodeB2, this->mtriangle.mnodeB3}};
    }

    /// Get the reaction force considered as applied to node A, in abs coords.
    ChVector<> GetReactionOnNode() const { return -react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

/// Utility class for using the ChLinkPointTriface constraint
class ChApiFea ChTriangleOfXYZROTnodes : public ChVariableTupleCarrier_3vars<6, 6, 6> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB1;
    std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB2;
    std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB3;

    virtual ChVariables* GetVariables1() { return &mnodeB1->Variables(); };
    virtual ChVariables* GetVariables2() { return &mnodeB2->Variables(); };
    virtual ChVariables* GetVariables3() { return &mnodeB3->Variables(); };
};

/// Class for creating a constraint between a xyz FEA node (point)
/// and a triangular face given by three xyzrot FEA nodes, with linear
/// shape function (ex. the face of a tetrahedron or a triangular shell)
/// The node can be offset respect to the face.
class ChApiFea ChLinkPointTrifaceRot : public ChLinkInterface {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkPointTrifaceRot)

  private:
    ChVector<> react;

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZROTnodes> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZROTnodes> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleOfXYZROTnodes> constraint3;

    std::shared_ptr<ChNodeFEAxyz> mnodeA;
    ChTriangleOfXYZROTnodes mtriangle;

    double s2, s3;
    double d;

  public:
    ChLinkPointTrifaceRot();
    ChLinkPointTrifaceRot(const ChLinkPointTrifaceRot& other);
    ~ChLinkPointTrifaceRot() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointTrifaceRot* Clone() const override { return new ChLinkPointTrifaceRot(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 6 + 6 + 6; }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 3; }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() override { return GetReactionOnNode(); }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    virtual ChCoordsys<> GetLinkAbsoluteCoords() override { return ChCoordsys<>(mnodeA->GetPos()); }

    /// Use this function after object creation, to initialize it, given
    /// the node and the triangle to join.
    /// The attachment position is the actual position of the node.
    /// Note, nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,      ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyzrot> anodeB1,  ///< triangle: corner n.1
                           std::shared_ptr<ChNodeFEAxyzrot> anodeB2,  ///< triangle: corner n.2
                           std::shared_ptr<ChNodeFEAxyzrot> anodeB3   ///< triangle: corner n.3
                           );

    /// Set the area coordinates to specify where the point A is connected on triangle.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms2, const double ms3) {
        s2 = ms2;
        s3 = ms3;
    }

    /// Get the area coordinates, as set respect to B2 and B3
    ///(the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms2, double& ms3) const {
        ms2 = s2;
        ms3 = s3;
    }

    /// Set an optional offset of point A respect to triangle, along triangle normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of point A respect to triangle
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNodeA() const { return this->mnodeA; }

    /// Get the connected triangle
    std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 3> GetConstrainedTriangle() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 3>{
            {this->mtriangle.mnodeB1, this->mtriangle.mnodeB2, this->mtriangle.mnodeB3}};
    }

    /// Get the reaction force considered as applied to node A, in abs coords.
    ChVector<> GetReactionOnNode() const { return -react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
