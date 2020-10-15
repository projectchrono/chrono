#include "chrono_vsg/shapes/VSGPbrBox.h"
#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

VSGPbrBox::VSGPbrBox(std::shared_ptr<ChBody> body,
                     std::shared_ptr<ChAsset> asset,
                     vsg::ref_ptr<vsg::MatrixTransform> transform)
    : ChVSGPbrIdxMesh(body, asset, transform) {}

void VSGPbrBox::Initialize(vsg::vec3& lightPosition, ChVSGPbrMaterial& mat, std::string& texFilePath) {
    m_lightPosition = lightPosition;
    m_textureFilePath = texFilePath;

    // set up vertices, normals, texcoords, indices
    m_vertices = vsg::vec3Array::create(
        {{1, -1, -1},  {1, 1, -1},  {1, 1, 1},  {1, -1, 1},  {-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1},
         {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1},  {-1, 1, -1},  {-1, 1, 1},  {1, 1, 1},
         {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},  {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}});

    m_normals = vsg::vec3Array::create({{1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {-1, 0, 0}, {-1, 0, 0},
                                        {-1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0},
                                        {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 0, 1},  {0, 0, 1},
                                        {0, 0, 1},  {0, 0, 1},  {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}});

    m_indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                          12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});

    m_albedo = vsg::vec3Array::create(m_vertices->size(), mat.albedo);
    m_metallic = vsg::floatArray::create(m_vertices->size(), mat.metallic);
    m_roughness = vsg::floatArray::create(m_vertices->size(), mat.roughness);
    m_ao = vsg::floatArray::create(m_vertices->size(), mat.ao);
}
