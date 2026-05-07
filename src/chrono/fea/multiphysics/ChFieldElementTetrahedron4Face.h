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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHFIELDELEMENTTETRAHEDRON4FACE_H
#define CHFIELDELEMENTTETRAHEDRON4FACE_H

#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4.h"
#include "chrono/fea/multiphysics/ChField.h"
#include "chrono/fea/multiphysics/ChFieldData.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/multiphysics/ChNodeFEAfieldXYZ.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Tetrahedron face. Useful for visualization etc.
/// When wrapped in a ChFieldElementLoadableSurface, this can be also used to apply UV loads 
/// of ChLoaderUVdistributed type, for example ChLoaderPressure
/// if referencing the ChDomainDeformation, or Neumann boundary in general like imposed heat flow, etc.
/// Corner nodes, obtainable with GetNode(), are in counterclockwise order seen from the outside.

class ChApi ChFieldTetrahedron4Face : public ChFieldElementSurface {
public:
    /// Construct the specified face (0 <= id <= 3) on the given hexahedral element.
    /// The face id is: 
    /// id = 0 for face opposed to vertex 0
    /// id = 1 for face opposed to vertex 1 
    /// id = 2 for face opposed to vertex 2 
    /// id = 3 for face opposed to vertex 3
    ChFieldTetrahedron4Face(std::shared_ptr<ChFieldElementTetrahedron4> element, char id) : m_face_id(id), m_element(element) {}

    ~ChFieldTetrahedron4Face() {}

    //
    // INTERFACE to  ChFieldElement 
    //

    virtual unsigned int GetNumNodes() override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int i) override {

        return m_element->GetNode(GetFaceNodeMapping(this->m_face_id)[i]);
    }

    virtual int GetSpatialDimensions() const override { return 3; }

    /// Compute shape function for the face. Note, only U,V of eta are used, assuming the two 
    /// parametric coords of the face.
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {
        // Btw one could compute the shape functions directly as 
        //  N(0) = 1.0 - eta.x() - eta.y();
        //  N(1) = eta.x();
        //  N(2) = eta.y();
        // But here is more general, using the volume shape functions:

        ChVector3d eta_vol = ConvertFaceToVolumeCoords(m_face_id, eta);

        ChRowVectorDynamic<double> N_vol(4);
        this->m_element->ComputeN(eta_vol, N_vol);

        auto node_map = GetFaceNodeMapping(m_face_id);
        for (int i = 0; i < 3; ++i) {
            N[i] = N_vol[node_map[i]];
        }
    }
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override { throw std::logic_error("Not implemented, not needed."); }
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J)  override { 
        ChVector3d eta_vol = ConvertFaceToVolumeCoords(m_face_id, eta);
        // Compute the volume Jacobian at the face point
        ChMatrix33d J_vol;
        m_element->ComputeJ(eta_vol, J_vol);
        std::pair<int, int> used_eta;
        switch (m_face_id) {
        case 0: used_eta = { 1, 2 }; break; // Use eta1, eta2
        case 1: used_eta = { 0, 2 }; break; // Use eta0, eta2  
        case 2: used_eta = { 0, 1 }; break; // Use eta0, eta1
        case 3: used_eta = { 0, 1 }; break; // Use eta0, eta1
        default: used_eta = { 0, 1 };
        }
        // Build 3×3 face Jacobian by copying two columns and adding a zero column
        J.col(0) = J_vol.col(used_eta.first);   // First face direction
        J.col(1) = J_vol.col(used_eta.second);  // Second face direction  
        J.col(2) = VNULL.eigen();               // Dummy direction
        return (Vcross(J.GetAxisX(), J.GetAxisY()).Length()); // shortcut, as J has null column at the rigth
    }
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv)  override { throw std::logic_error("Not implemented, not needed."); }
    virtual int GetNumQuadraturePointsForOrder(const int order) const  override { throw std::logic_error("Not implemented, not needed."); }
    virtual void GetMaterialPointWeight(const int order, const int i, double& weight, ChVector3d& coords) const  override { throw std::logic_error("Not implemented, not needed."); }


    virtual bool IsTriangleIntegrationCompatible() const override { return true; }

    virtual ChVector3d ComputeNormal(const double U, const double V) override {
        ChVector3d p0 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(0)));
        ChVector3d p1 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(1)));
        ChVector3d p2 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(2)));
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }


private:

    // utility
    static std::array<int, 3>& GetFaceNodeMapping(int faceId) {
        static std::array<int, 3> ifa0{ 1, 3, 2 }; // Face opposite node 0 uses nodes 1,2,3
        static std::array<int, 3> ifa1{ 0, 2, 3 }; // Face opposite node 1 uses nodes 0,2,3
        static std::array<int, 3> ifa2{ 0, 3, 1 }; // Face opposite node 2 uses nodes 0,1,3
        static std::array<int, 3> ifa3{ 0, 1, 2 }; // Face opposite node 3 uses nodes 0,1,2
        switch (faceId) {
        case 0: return ifa0;
        case 1: return ifa1;
        case 2: return ifa2;
        case 3: return ifa3;
        default: return ifa3;
        }
    }

    // utility: from  eta_face(u,v,0)  to  eta_volume(..,..,..)
    static ChVector3d ConvertFaceToVolumeCoords(int faceId, const ChVector3d& eta_face) {
        switch (faceId) {
        case 0: // Face 0: parametric coords are (eta1, eta2)
            return ChVector3d(0.0, eta_face[0], eta_face[1]);
        case 1: // Face 1: parametric coords are (eta0, eta2)
            return ChVector3d(eta_face[0], 0.0, eta_face[1]);
        case 2: // Face 2: parametric coords are (eta0, eta1)  
            return ChVector3d(eta_face[0], eta_face[1], 0.0);
        case 3: // Face 3: parametric coords are (eta0, eta1)
            return ChVector3d(eta_face[0], eta_face[1], 1.0 - eta_face[0] - eta_face[1]);
        default:
            return VNULL;
        }
    }

    char m_face_id;                                  ///< id of the face on the tetrahedron
    std::shared_ptr<ChFieldElementTetrahedron4> m_element;  ///< associated tetrahedron element
    //std::shared_ptr<ChFieldBase> m_field;  ///< associated field
};






/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
