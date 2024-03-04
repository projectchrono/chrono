// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke
// =============================================================================

#include "chrono_vsg/shapes/GetSurfaceShapeData.h"

namespace chrono {
namespace vsg3d {

void GetSurfaceShapeData(std::shared_ptr<ChVisualShapeSurface> surface,
                         vsg::ref_ptr<vsg::vec3Array>& vertices,
                         vsg::ref_ptr<vsg::vec3Array>& normals,
                         vsg::ref_ptr<vsg::vec2Array>& texcoords,
                         vsg::ref_ptr<vsg::ushortArray>& indices) {
    auto sections_u = surface->GetResolutionU() * 4;  //***TEST*** (from irrlicht surface)
    auto sections_v = surface->GetResolutionV() * 4;  //***TEST***
    auto nvertices = (sections_u + 1) * (sections_v + 1);
    auto ntriangles = (sections_u) * (sections_v)*2;
    auto nindices = ntriangles * 3;

    vertices = vsg::vec3Array::create(nvertices);
    normals = vsg::vec3Array::create(nvertices);
    texcoords = vsg::vec2Array::create(nvertices);
    indices = vsg::ushortArray::create(nindices);

    int itri = 0;

    for (auto iv = 0; iv <= sections_v; ++iv) {
        double mV = iv / (double)sections_v;  // v abscissa

        for (auto iu = 0; iu <= sections_u; ++iu) {
            double mU = iu / (double)sections_u;  // u abscissa

            ChVector<> P = surface->GetSurfaceGeometry()->Evaluate(mU, mV);
            ////P = vis->Pos + vis->Rot * P;

            ChVector<> N = surface->GetSurfaceGeometry()->GetNormal(mU, mV);
            ////N = vis->Rot * N;

            // create two triangles per uv increment
            vertices->set(iu + iv * (sections_u + 1), vsg::vec3(P.x(), P.y(), P.z()));
            normals->set(iu + iv * (sections_u + 1), vsg::vec3(N.x(), N.y(), N.z()));
            texcoords->set(iu + iv * (sections_u + 1), vsg::vec2(mU, mV));

            if (iu > 0 && iv > 0) {
                indices->set(0 + itri * 3, iu - 1 + iv * (sections_u + 1));
                indices->set(1 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                indices->set(2 + itri * 3, iu + iv * (sections_u + 1));
                ++itri;
                indices->set(0 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                indices->set(1 + itri * 3, iu + (iv - 1) * (sections_u + 1));
                indices->set(2 + itri * 3, iu + iv * (sections_u + 1));
                ++itri;
            }
        }
    }
}

}  // namespace vsg3d
}  // namespace chrono
