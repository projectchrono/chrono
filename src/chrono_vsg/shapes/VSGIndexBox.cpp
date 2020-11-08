#include "chrono_vsg/shapes/VSGIndexBox.h"
#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

VSGIndexBox::VSGIndexBox(std::shared_ptr<ChBody> body,
                         std::shared_ptr<ChAsset> asset,
                         vsg::ref_ptr<vsg::MatrixTransform> transform)
    : ChVSGIndexMesh(body, asset, transform) {}

void VSGIndexBox::Initialize(ChTexturedPBR& textures, size_t tessFactor) {
    Tesselate(tessFactor);

    m_matMode = MaterialMode::MappedPBR;
    m_albedoMapPath = textures.GetAlbedoTextureFilename();
    m_normalMapPath = textures.GetNormalTextureFilename();
    m_metallicMapPath = textures.GetMetallicTextureFilename();
    m_roughnessMapPath = textures.GetRoughnessTextureFilename();
    m_aoMapPath = textures.GetAOTextureFilename();
}

void VSGIndexBox::Initialize(ChTexture& texture, size_t tessFactor) {
    Tesselate(tessFactor);

    m_matMode = MaterialMode::Textured;
    m_textureFilePath = texture.GetTextureFilename();
}

void VSGIndexBox::Initialize(ChColor& color, size_t tessFactor) {
    Tesselate(tessFactor);

    m_matMode = MaterialMode::SimplePhong;
    m_objectColor[0] = color.R;
    m_objectColor[1] = color.G;
    m_objectColor[2] = color.B;

    m_colors = vsg::vec3Array::create(m_vertices->size(), m_objectColor);
}

void VSGIndexBox::Initialize(ChPBRSetting& pbrSet, size_t tessFactor) {
    Tesselate(tessFactor);

    m_matMode = MaterialMode::PBR;

    m_objectColor[0] = pbrSet.GetAlbedo().R;
    m_objectColor[1] = pbrSet.GetAlbedo().G;
    m_objectColor[2] = pbrSet.GetAlbedo().B;

    m_colors = vsg::vec3Array::create(m_vertices->size(), m_objectColor);
    m_metallics = vsg::floatArray::create(m_vertices->size(), pbrSet.GetMetallic());
    m_roughnesses = vsg::floatArray::create(m_vertices->size(), pbrSet.GetRoughness());
    m_aos = vsg::floatArray::create(m_vertices->size(), pbrSet.GetAO());
}

void VSGIndexBox::Tesselate(size_t tessFactor) {
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
