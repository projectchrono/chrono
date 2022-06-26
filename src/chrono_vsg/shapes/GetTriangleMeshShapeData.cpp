
#include "chrono_vsg/shapes/GetTriangleMeshShapeData.h"

namespace chrono {
namespace vsg3d {
void GetTriangleMeshShapeData(std::shared_ptr<ChTriangleMeshShape> tms,
                              vsg::ref_ptr<vsg::vec3Array>& vertices,
                              vsg::ref_ptr<vsg::vec3Array>& normals,
                              vsg::ref_ptr<vsg::vec2Array>& texcoords,
                              vsg::ref_ptr<vsg::ushortArray>& indices,
                              float& boundingSphereRadius) {
    auto mesh = tms->GetMesh();
    int numTriangles = mesh->getNumTriangles();
    int iVert = 0;
    int nVert = 3 * numTriangles;
    vertices = vsg::vec3Array::create(nVert);
    int kVert = 0;
    normals = vsg::vec3Array::create(nVert);
    // u,v may be unknown or not appliable, phantasy settings
    texcoords = vsg::vec2Array::create(nVert);
    for (int iTri = 0; iTri < numTriangles; iTri++) {
        auto tri = mesh->getTriangle(iTri);
        auto trinorm = tri.GetNormal();
        vertices->set(kVert, vsg::vec3(tri.p1.x(), tri.p1.y(), tri.p1.z()));
        texcoords->set(kVert, vsg::vec2(0, 0));
        normals->set(kVert++, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));
        vertices->set(kVert, vsg::vec3(tri.p2.x(), tri.p2.y(), tri.p2.z()));
        texcoords->set(kVert, vsg::vec2(1, 0));
        normals->set(kVert++, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));
        vertices->set(kVert, vsg::vec3(tri.p3.x(), tri.p3.y(), tri.p3.z()));
        texcoords->set(kVert, vsg::vec2(1, 1));
        normals->set(kVert++, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));
    }

    indices = vsg::ushortArray::create(nVert);
    for (int iIdx = 0; iIdx < nVert; iIdx ++) {
        indices->set(iIdx, iIdx);
    }
    boundingSphereRadius = 1.1f * mesh->GetBoundingSphereRadius();
}
}  // namespace vsg3d
}  // namespace chrono