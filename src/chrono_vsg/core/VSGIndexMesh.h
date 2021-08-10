#ifndef VSG_INDEX_MESH_H
#define VSG_INDEX_MESH_H

#include <iostream>
#include <string>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>
#include "chrono_vsg/resources/VSGPhongMaterial.h"
#include "chrono_vsg/resources/VSGPbrMaterial.h"

namespace chrono {
namespace vsg3d {

class CH_VSG_API VSGIndexMesh {
  public:
    typedef enum { Unknown, Phong, PBR } MaterialMode;

  public:
    VSGIndexMesh(std::shared_ptr<ChBody> body,
                 std::shared_ptr<ChAsset> geometryAsset,
                 vsg::ref_ptr<vsg::MatrixTransform> transform,
                 bool polygonFilled);

    vsg::ref_ptr<vsg::Node> createVSGNode();

  protected:
    virtual void Tesselate(size_t tessFactor) = 0;
    MaterialMode m_matMode;
    bool m_polygonFilled;

    std::shared_ptr<ChBody> m_bodyPtr;
    std::shared_ptr<ChAsset> m_assetPtr;
    vsg::ref_ptr<vsg::MatrixTransform> m_transform;

    PhongMaterial phongMat;
    PbrMaterial pbrMat;

    vsg::ref_ptr<vsg::vec3Array> m_vertices;
    vsg::ref_ptr<vsg::vec3Array> m_normals;
    vsg::ref_ptr<vsg::vec2Array> m_texcoords;
    vsg::ref_ptr<vsg::ushortArray> m_indices;

    std::vector<std::string> m_defines;

    vsg::ref_ptr<vsg::GraphicsPipeline> createPipeline(vsg::ref_ptr<vsg::ShaderStage> vs,
                                                       vsg::ref_ptr<vsg::ShaderStage> fs,
                                                       vsg::ref_ptr<vsg::DescriptorSetLayout> descriptorSetLayout,
                                                       bool doubleSided,
                                                       bool enableBlend);

    void genPhongSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
    void genPBRSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph);
};
}  // namespace vsg3d
}  // namespace chrono
#endif
