#ifndef CH_VSG_INDEX_MESH_H
#define CH_VSG_INDEX_MESH_H

#include <iostream>
#include <string>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/assets/ChPBRSetting.h"
#include "chrono_vsg/assets/ChPBRMaps.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono_vsg/resources/ChVSGPhongMaterial.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGIndexMesh {
  public:
    typedef enum { Unknown, Textured, SimplePhong, PBR, PBRMaps } MaterialMode;

  public:
    ChVSGIndexMesh(std::shared_ptr<ChBody> body,
                   std::shared_ptr<ChAsset> asset,
                   vsg::ref_ptr<vsg::MatrixTransform> transform);

    virtual void Initialize(ChTexture& texture, size_t tessFactor = 3) = 0;
    virtual void Initialize(ChColor& color, size_t tessFactor = 3) = 0;
    virtual void Initialize(ChPBRSetting& pbrSet, size_t tessFactor = 3) = 0;
    virtual void Initialize(ChPBRMaps& pbrMaps, size_t tessFactor = 3) = 0;
    vsg::ref_ptr<vsg::Node> createVSGNode();

  protected:
    virtual void Tesselate(size_t tessFactor) = 0;
    MaterialMode m_matMode;
    vsg::ref_ptr<vsg::ShaderStage> readVertexShader(std::string filePath);
    vsg::ref_ptr<vsg::ShaderStage> readFragmentShader(std::string filePath);
    vsg::ref_ptr<vsg::vec4Array2D> createRGBATexture(std::string filePath);

    vsg::vec3 m_lightPosition;
    std::string m_textureFilePath;
    vsg::vec3 m_objectColor;
    float m_radiantIntensity;

    std::string m_albedoMapPath;
    std::string m_normalMapPath;
    std::string m_metallicMapPath;
    std::string m_roughnessMapPath;
    std::string m_aoMapPath;

    vsg::ref_ptr<vsg::vec3Array> m_vertices;
    vsg::ref_ptr<vsg::vec3Array> m_normals;
    vsg::ref_ptr<vsg::vec3Array> m_colors;
    vsg::ref_ptr<vsg::vec2Array> m_texcoords;
    vsg::ref_ptr<vsg::ushortArray> m_indices;

    vsg::ref_ptr<vsg::floatArray> m_metallics;
    vsg::ref_ptr<vsg::floatArray> m_roughnesses;
    vsg::ref_ptr<vsg::floatArray> m_aos;

    std::shared_ptr<ChBody> m_bodyPtr;
    std::shared_ptr<ChAsset> m_assetPtr;
    vsg::ref_ptr<vsg::MatrixTransform> m_transform;

    void genTexturedSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
    void genSimplePhongSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
    void genPBRSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
    void genPBRMapSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
};
}  // namespace vsg3d
}  // namespace chrono
#endif
