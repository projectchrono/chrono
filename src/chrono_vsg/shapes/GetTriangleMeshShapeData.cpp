
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
    GetLog() << "Num Triangles = " << numTriangles << " (Really?)\n";
    auto tri = mesh->getTriangle(0);
    const float a = 0.1;
    vertices = vsg::vec3Array::create(3);
    vertices->set(0, vsg::vec3(tri.p1.x(), tri.p1.y(), tri.p1.z()));
    vertices->set(1, vsg::vec3(tri.p2.x(), tri.p2.y(), tri.p2.z()));
    vertices->set(2, vsg::vec3(tri.p3.x(), tri.p3.y(), tri.p3.z()));
    auto trinorm = tri.GetNormal();
    normals = vsg::vec3Array::create(3);
    normals->set(0, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));
    normals->set(1, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));
    normals->set(2, vsg::vec3(trinorm.x(), trinorm.y(), trinorm.z()));

    // u,v may be unknown or not applicable, phantasy settings
    texcoords = vsg::vec2Array::create({{0,0},{0,1},{1,0}});

    indices = vsg::ushortArray::create({0,  1,  2});
    double xmin, xmax, ymin, ymax, zmin, zmax;
    ChMatrix33<> rot;
    tri.GetBoundingBox(xmin, xmax, ymin, ymax, zmin, zmax, &rot);
    // bounding sphere radius > sqrt(a^2+a^2+a^2)
    boundingSphereRadius = 1.1f * sqrt(pow(xmax-xmin,2)+pow(ymax-ymin,2)+pow(zmax-zmin,2));
}
}  // namespace vsg3d
}  // namespace chrono