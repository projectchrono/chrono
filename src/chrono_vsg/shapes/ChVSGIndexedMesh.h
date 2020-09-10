#ifndef CH_VSG_INDEXED_MESH_H
#define CH_VSG_INDEXED_MESH_H

#include <iostream>
#include <string>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono_vsg/resources/ChVSGPhongMaterial.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGIndexedMesh {
  public:
    ChVSGIndexedMesh();

    virtual void Initialize(vsg::vec3& lightPosition, ChVSGPhongMaterial& mat, std::string& texFilePath) = 0;
    vsg::ref_ptr<vsg::Node> createVSGNode(DrawMode drawMode, vsg::ref_ptr<vsg::MatrixTransform> transform);

  protected:
    vsg::ref_ptr<vsg::ShaderStage> readVertexShader(std::string filePath);
    vsg::ref_ptr<vsg::ShaderStage> readFragmentShader(std::string filePath);
    vsg::ref_ptr<vsg::vec4Array2D> createRGBATexture(std::string filePath);

    vsg::vec3 m_lightPosition;
    std::string m_textureFilePath;

    vsg::ref_ptr<vsg::vec3Array> m_vertices;
    vsg::ref_ptr<vsg::vec3Array> m_normals;
    vsg::ref_ptr<vsg::vec2Array> m_texcoords;
    vsg::ref_ptr<vsg::ushortArray> m_indices;

    vsg::ref_ptr<vsg::vec3Array> m_ambientColor;
    vsg::ref_ptr<vsg::vec3Array> m_diffuseColor;
    vsg::ref_ptr<vsg::vec3Array> m_specularColor;
    vsg::ref_ptr<vsg::floatArray> m_shininess;
    vsg::ref_ptr<vsg::floatArray> m_opacity;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
