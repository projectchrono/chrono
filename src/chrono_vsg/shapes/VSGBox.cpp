#include "chrono_vsg/shapes/VSGBox.h"
#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

VSGBox::VSGBox(std::shared_ptr<ChBody> body,
               std::shared_ptr<ChAsset> asset,
               vsg::ref_ptr<vsg::MatrixTransform> transform,
               bool polygonFilled)
    : VSGIndexMesh(body, asset, transform, polygonFilled) {}

void VSGBox::Tesselate(size_t tessFactor) {
    // define the basic structure for all shapes
    // set up vertices, normals, texcoords, indices
    m_vertices = vsg::vec3Array::create(
        {{1, -1, -1},  {1, 1, -1},  {1, 1, 1},  {1, -1, 1},  {-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1},
         {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1},  {-1, 1, -1},  {-1, 1, 1},  {1, 1, 1},
         {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},  {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}});

    m_normals = vsg::vec3Array::create({{1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {-1, 0, 0}, {-1, 0, 0},
                                        {-1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0},
                                        {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 0, 1},  {0, 0, 1},
                                        {0, 0, 1},  {0, 0, 1},  {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}});

    m_texcoords = vsg::vec2Array::create({{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1},
                                          {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1},
                                          {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1}});

    m_indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                          12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});
}
