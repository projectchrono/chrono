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

#ifndef CHFIELDELEMENTHEXAHEDRON8FACE_H
#define CHFIELDELEMENTHEXAHEDRON8FACE_H

#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChField.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Face of an hexahedron.  Useful for visualization etc.
/// When wrapped in a ChFieldElementLoadableSurface, this can be also used to apply UV loads 
/// of ChLoaderUVdistributed type, for example ChLoaderPressure if referencing the ChDomainDeformation, 
/// or Neumann boundary in general like imposed heat flow, etc.
/// Corner nodes, obtainable with GetNode(), are in counterclockwise order seen from the outside.

class ChApi ChFieldHexahedron8Face : public ChFieldElementSurface {
public:
    /// Construct the specified face (0 <= id <= 5) on the given hexahedral element.
    /// Assuming the U,V,W parametrization of the hexahedron, the face id is: 
    /// id = 0 for face at U=-1, 
    /// id = 1 for face at U=+1, 
    /// id = 2 for face at V=-1, 
    /// id = 3 for face at V=+1,
    /// id = 4 for face at W=-1, 
    /// id = 5 for face at W=+1,
    ChFieldHexahedron8Face(std::shared_ptr<ChFieldElementHexahedron8> element, char id) : m_face_id(id), m_element(element) {}

    ~ChFieldHexahedron8Face() {}

    //
    // INTERFACE to  ChFieldElement 
    //

    virtual unsigned int GetNumNodes() override { return 4; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int i) override {

        return m_element->GetNode(GetFaceNodeMapping(this->m_face_id)[i]);

    }

    virtual int GetSpatialDimensions() const override { return 3; }

    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {

        ChVector3d eta_vol = ConvertFaceToVolumeCoords(m_face_id, eta);

        ChRowVectorDynamic<double> N_vol(8);
        this->m_element->ComputeN(eta_vol, N_vol);

        auto node_map = GetFaceNodeMapping(m_face_id);
        for (int i = 0; i < 4; ++i) {
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
        case 0:
        case 1:
            used_eta = { 1, 2 }; break; // Use eta1, eta2
        case 2:
        case 3:
            used_eta = { 0, 2 }; break; // Use eta0, eta2  
        case 4:
        case 5:
            used_eta = { 0, 1 }; break; // Use eta0, eta1
        default: 
            used_eta = { 0, 1 };
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

    // The following needed only if element is wrapped as a component of a ChLoaderUV.
    virtual bool IsTriangleIntegrationCompatible() const override { return false; }

    virtual ChVector3d ComputeNormal(const double U, const double V) override {
        ChMatrix33d J_face;
        this->ComputeJ(ChVector3d(U, V, 0), J_face);

        switch (this->m_face_id) {
        case 0: return -Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        case 1: return  Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        case 2: return  Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        case 3: return -Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        case 4: return -Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        case 5: return  Vcross(J_face.GetAxisX(), J_face.GetAxisY()).GetNormalized();
        default: return VECT_X;
        }

    }


private:

    // utility
    static std::array<int, 4>& GetFaceNodeMapping(int faceId) {
        static std::array<int, 4> ifa0 = { 0, 5, 7, 3 };
        static std::array<int, 4> ifa1 = { 1, 5, 6, 2 };
        static std::array<int, 4> ifa2 = { 0, 1, 5, 4 };
        static std::array<int, 4> ifa3 = { 2, 3, 7, 6 };
        static std::array<int, 4> ifa4 = { 0, 3, 2, 1 };
        static std::array<int, 4> ifa5 = { 4, 5, 6, 7 };
        switch (faceId) {
        case 0: return ifa0;
        case 1: return ifa1;
        case 2: return ifa2;
        case 3: return ifa3;
        case 4: return ifa4;
        case 5: return ifa5;
        default: return ifa5;
        }
    }

    // utility: from  eta_face(u,v,0)  to  eta_volume(..,..,..)
    static ChVector3d ConvertFaceToVolumeCoords(int faceId, const ChVector3d& eta_face) {
        switch (faceId) {
        case 0: // Face 0: eta0 = -1 
            return ChVector3d(-1.0, eta_face[0], eta_face[1]);
        case 1: // Face 1: eta0 = +1
            return ChVector3d( 1.0, eta_face[0], eta_face[1]);
        case 2: // Face 2: eta1 = -1
            return ChVector3d(eta_face[0], -1.0, eta_face[1]);
        case 3: // Face 3: eta1 = +1
            return ChVector3d(eta_face[0],  1.0, eta_face[1]);
        case 4: // Face 4: eta2 = -1
            return ChVector3d(eta_face[0], eta_face[1], -1.0);
        case 5: // Face 5: eta2 = +1
            return ChVector3d(eta_face[0], eta_face[1],  1.0);
        default:
            return VNULL;
        }
    }

    char m_face_id;                                  ///< id of the face on the hexahedron

    std::shared_ptr<ChFieldElementHexahedron8> m_element;  ///< associated hexahedron element

    //std::shared_ptr<ChFieldBase> m_field;  ///< associated field
};




/// @} chrono_fea

}  // end namespace fea


}  // end namespace chrono

#endif
