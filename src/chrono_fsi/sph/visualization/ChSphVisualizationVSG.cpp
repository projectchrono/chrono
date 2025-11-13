// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/utils/SphUtilsTypeConvert.cuh"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSIStatsVSG(ChSphVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        vsg3d::ChVisualSystemVSG& vsys = m_vsysFSI->GetVisualSystemVSG();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin(m_vsysFSI->m_sysSPH->GetPhysicsProblemString().c_str());

        if (ImGui::BeginTable("SPH", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("SPH particles:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumFluidMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Boundary BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumBoundaryMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Rigid body BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumRigidBodyMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Flex body BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumFlexBodyMarkers()));

            ImGui::TableNextRow();

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(m_vsysFSI->m_sysSPH->GetSphIntegrationSchemeString().c_str());

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Step size:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.1e", m_vsysFSI->m_sysFSI->GetStepSizeCFD());
            } else {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Step size:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.1e", m_vsysFSI->m_sysSPH->GetStepSize());
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("RTF (fluid):");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_vsysFSI->m_sysSPH->GetRtf());

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("RTF (solid):");
                ImGui::TableNextColumn();
                ImGui::Text("%8.3f", m_vsysFSI->m_sysFSI->GetRtfMBD());
            }

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("MBS/CFD ratio:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.3f", m_vsysFSI->m_sysFSI->GetRatioMBD());
            }

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Particles", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool sph_visible = m_vsysFSI->m_sph_markers;
            if (ImGui::Checkbox("SPH", &sph_visible)) {
                m_vsysFSI->m_sph_markers = !m_vsysFSI->m_sph_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_sph_markers, ChSphVisualizationVSG::ParticleCloudTag::SPH);
            }

            ImGui::TableNextColumn();
            static bool bce_wall_visible = m_vsysFSI->m_bndry_bce_markers;
            if (ImGui::Checkbox("BCE wall", &bce_wall_visible)) {
                m_vsysFSI->m_bndry_bce_markers = !m_vsysFSI->m_bndry_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_bndry_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_WALL);
            }

            ImGui::TableNextColumn();
            static bool bce_rigid_visible = m_vsysFSI->m_rigid_bce_markers;
            if (ImGui::Checkbox("BCE rigid", &bce_rigid_visible)) {
                m_vsysFSI->m_rigid_bce_markers = !m_vsysFSI->m_rigid_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_rigid_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_RIGID);
            }

            ImGui::TableNextColumn();
            static bool bce_flex_visible = m_vsysFSI->m_flex_bce_markers;
            if (ImGui::Checkbox("BCE flex", &bce_flex_visible)) {
                m_vsysFSI->m_flex_bce_markers = !m_vsysFSI->m_flex_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_flex_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_FLEX);
            }

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("ActiveBoxes", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool boxes_visible = m_vsysFSI->m_active_boxes;
            if (ImGui::Checkbox("Active Domains", &boxes_visible)) {
                m_vsysFSI->m_active_boxes = !m_vsysFSI->m_active_boxes;
                m_vsysFSI->SetActiveBoxVisibility(m_vsysFSI->m_active_boxes, -1);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChSphVisualizationVSG* m_vsysFSI;
};

// ---------------------------------------------------------------------------

ChSphVisualizationVSG::ChSphVisualizationVSG(ChFsiSystemSPH* sysFSI)
    : m_sysFSI(sysFSI),
      m_sysSPH(&sysFSI->GetFluidSystemSPH()),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_active_boxes(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 1.0f, 0.30f)),
      m_flex_bce_color(ChColor(1.0f, 1.0f, 0.4f)),
            m_active_box_color(ChColor(1.0f, 1.0f, 0.0f)),
            m_colormap_type(ChColormap::Type::JET),
            m_write_images(false),
            m_image_dir("."),
            m_sph_cloud_index(-1) {  // start with invalid cache so we rescan once the VSG clouds are bound
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
    m_activeBoxScene = vsg::Switch::create();
}

ChSphVisualizationVSG::ChSphVisualizationVSG(ChFsiFluidSystemSPH* sysSPH)
    : m_sysFSI(nullptr),
      m_sysSPH(sysSPH),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 1.0f, 0.30f)),
      m_flex_bce_color(ChColor(1.0f, 1.0f, 0.4f)),
    m_write_images(false),
    m_image_dir("."),
    m_sph_cloud_index(-1) {  // ensure the SPH cloud lookup is revalidated on the first query
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
}

ChSphVisualizationVSG::~ChSphVisualizationVSG() {
    auto& systems = m_vsys->GetSystems();
    auto index = std::find(systems.begin(), systems.end(), m_sysMBS);
    if (index != systems.end())
        systems.erase(index);

    delete m_sysMBS;
}

ChColormap::Type ChSphVisualizationVSG::GetColormapType() const {
    return m_colormap_type;
}

const ChColormap& ChSphVisualizationVSG::GetColormap() const {
    return *m_colormap;
}

void ChSphVisualizationVSG::SetSPHColorCallback(std::shared_ptr<ParticleColorCallback> functor, ChColormap::Type type) {
    m_color_fun = functor;
    m_color_fun->m_vsys = this;

    m_colormap_type = type;
    if (m_colormap) {
        m_colormap->Load(type);
    }

    // Force the GPU colormap buffer to be regenerated for the new palette
    m_gpu_color.colormapDirty = true;
}

void ChSphVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChSphVisualizationVSG::OnInitialize() {
    // Create particle clouds for SPH particles, as well as wall, rigid, and flex BCE markers
    // Initialize their visibility flag
    {
        m_sph_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_sph_cloud->SetName("sph_particles");
        m_sph_cloud->SetTag(ParticleCloudTag::SPH);
        m_sph_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumFluidMarkers(); i++) {
            m_sph_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 2);
        sphere->SetColor(ChColor(0.10f, 0.40f, 0.65f));
        m_sph_cloud->AddVisualShape(sphere);
        m_sph_cloud->RegisterColorCallback(m_color_fun);
        m_sph_cloud->RegisterVisibilityCallback(m_vis_sph_fun);
        m_sysMBS->Add(m_sph_cloud);

        m_vsys->SetParticleCloudVisibility(m_sph_markers, ParticleCloudTag::SPH);
    }

    {
        m_bndry_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_bndry_bce_cloud->SetName("bce_boundary");
        m_bndry_bce_cloud->SetTag(ParticleCloudTag::BCE_WALL);
        m_bndry_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_bndry_bce_color);
        m_bndry_bce_cloud->AddVisualShape(sphere);
        m_bndry_bce_cloud->RegisterVisibilityCallback(m_vis_bndry_fun);
        m_sysMBS->Add(m_bndry_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_bndry_bce_markers, ParticleCloudTag::BCE_WALL);
    }

    {
        m_rigid_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_rigid_bce_cloud->SetName("bce_rigid");
        m_rigid_bce_cloud->SetTag(ParticleCloudTag::BCE_RIGID);
        m_rigid_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_rigid_bce_color);
        m_rigid_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_rigid_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_rigid_bce_markers, ParticleCloudTag::BCE_RIGID);
    }

    {
        m_flex_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_flex_bce_cloud->SetName("bce_flex");
        m_flex_bce_cloud->SetTag(ParticleCloudTag::BCE_FLEX);
        m_flex_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_flex_bce_color);
        m_flex_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_flex_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_flex_bce_markers, ParticleCloudTag::BCE_FLEX);
    }

    // Cache information about active domains
    m_use_active_boxes = m_sysSPH->GetParams().use_active_domain;
    m_active_box_hsize = ToChVector(m_sysSPH->GetParams().bodyActiveDomain);

    // Create colormap
    m_colormap = chrono_types::make_unique<ChColormap>(m_colormap_type);

    // Create custom GUI for the FSI plugin
    auto fsi_states = chrono_types::make_shared<FSIStatsVSG>(this);
    m_vsys->AddGuiComponent(fsi_states);

    // Add colorbar GUI
    if (m_color_fun) {
        m_vsys->AddGuiColorbar(m_color_fun->GetTile(), m_color_fun->GetDataRange(), m_colormap_type,
                               m_color_fun->IsBimodal(), 400.0f);
    }

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);

    // Issue performance warning if shadows are enabled for the containing visualization system
    if (m_vsys->AreShadowsEnabled()) {
        std::cerr << "WARNING:  Shadow rendering is enabled for the associated VSG visualization system.\n";
        std::cerr << "          This negatively affects rendering performance, especially for large particle systems."
                  << std::endl;
    }
}

void ChSphVisualizationVSG::OnBindAssets() {
    // Create the VSG group for the active domains
    m_vsys->GetVSGScene()->addChild(m_activeBoxScene);

    // Create the box for the computational domain
    BindComputationalDomain();

    if (!m_use_active_boxes)
        return;

    // Loop over all FSI bodies and bind a model for its active box
    for (const auto& fsi_body : m_sysFSI->GetBodies())
        BindActiveBox(fsi_body->body, fsi_body->body->GetTag());
}

void ChSphVisualizationVSG::SetActiveBoxVisibility(bool vis, int tag) {
    if (!m_vsys->IsInitialized())
        return;

    for (auto& child : m_activeBoxScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChSphVisualizationVSG::BindComputationalDomain() {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto hsize = m_sysSPH->GetComputationalDomain().Size() / 2;

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(m_sysSPH->GetComputationalDomain().Center(), QUNIT), hsize);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", nullptr);
    group->setValue("Tag", -1);
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

void ChSphVisualizationVSG::BindActiveBox(const std::shared_ptr<ChBody>& obj, int tag) {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(obj->GetPos(), QUNIT), m_active_box_hsize);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", obj);
    group->setValue("Tag", tag);
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

vsg3d::ChVisualSystemVSG::ParticleCloud* ChSphVisualizationVSG::GetSphParticleCloud() {
    if (!m_vsys)
        return nullptr;

    // Cache the mapping to the VSG particle cloud list so we do not search every frame
    // (less cpu work). But verify that the cached index is still valid (in case clouds were added/removed or dynamic range)
    auto& clouds = m_vsys->GetParticleClouds();

    if (m_sph_cloud_index >= 0 && m_sph_cloud_index < static_cast<int>(clouds.size())) {
        auto& cached = clouds[static_cast<size_t>(m_sph_cloud_index)];
        if (cached.pcloud.get() == m_sph_cloud.get())
            return &cached;
    }

    for (size_t i = 0; i < clouds.size(); ++i) {
        if (clouds[i].pcloud.get() == m_sph_cloud.get()) {
            m_sph_cloud_index = static_cast<int>(i);
            return &clouds[i];
        }
    }

    m_sph_cloud_index = -1;
    return nullptr;
}

ChSphVisualizationVSG::ColorMode ChSphVisualizationVSG::DetermineColorMode() const {
    // Translate the active color callback into the shader mode setting
    if (!m_color_fun)
        return ColorMode::NONE;

    if (std::dynamic_pointer_cast<ParticleHeightColorCallback>(m_color_fun))
        return ColorMode::HEIGHT;

    if (auto velocity = std::dynamic_pointer_cast<ParticleVelocityColorCallback>(m_color_fun)) {
        switch (velocity->GetComponent()) {
            case ParticleVelocityColorCallback::Component::NORM:
                return ColorMode::VELOCITY_MAG;
            case ParticleVelocityColorCallback::Component::X:
                return ColorMode::VELOCITY_X;
            case ParticleVelocityColorCallback::Component::Y:
                return ColorMode::VELOCITY_Y;
            case ParticleVelocityColorCallback::Component::Z:
                return ColorMode::VELOCITY_Z;
        }
    }

    if (std::dynamic_pointer_cast<ParticleDensityColorCallback>(m_color_fun))
        return ColorMode::DENSITY;

    if (std::dynamic_pointer_cast<ParticlePressureColorCallback>(m_color_fun))
        return ColorMode::PRESSURE;

    return ColorMode::NONE;
}

bool ChSphVisualizationVSG::ShouldUseGpuColoring(size_t num_particles) const {
    // Only enable the compute path when we have data and a supported colouring callback, else dont
    if (!m_color_fun) {
        std::cout << "GPU colouring disabled: no colour callback function set" << std::endl;
        return false;
    }
    if (num_particles == 0) {
        std::cout << "GPU colouring disabled: no particles to render" << std::endl;
        return false;
    }
    if (DetermineColorMode() == ColorMode::NONE) {
        std::cout << "GPU colouring disabled: unsupported colour mode" << std::endl;
        return false;
    }
    return true;
}

bool ChSphVisualizationVSG::IsColormapSupported() const {
    return true;
}

bool ChSphVisualizationVSG::InitializeGpuColoringResources(size_t num_particles) {
    if (num_particles == 0)
        return false;

    auto cloud = GetSphParticleCloud();
    // Defer initialisation until the visual system has bound the SPH cloud buffers
    if (!cloud || !cloud->position_bufferInfo || !cloud->color_bufferInfo)
        return false;

    auto shaderPath = vsg::findFile("vsg/shaders/fsiParticleColor.comp", m_vsys->GetOptions());
    // Abort silently if the compute shader is missing from the resource paths
    if (shaderPath.empty())
        return false;

    auto shaderStage = vsg::ShaderStage::read(VK_SHADER_STAGE_COMPUTE_BIT, "main", shaderPath, m_vsys->GetOptions());
    if (!shaderStage)
        return false;

    // Use vec4 storage for alignment with the compute shader
    m_gpu_color.positionData = vsg::vec4Array::create(num_particles);
    m_gpu_color.positionData->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_gpu_color.velocityData = vsg::vec4Array::create(num_particles);
    m_gpu_color.velocityData->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_gpu_color.propertyData = vsg::vec4Array::create(num_particles);
    m_gpu_color.propertyData->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_gpu_color.uniformData = vsg::vec4Array::create(3);
    m_gpu_color.uniformData->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_gpu_color.colormapData = vsg::vec4Array::create(static_cast<size_t>(m_gpu_color.colormapResolution));
    m_gpu_color.colormapData->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_gpu_color.positionDescriptor =
        vsg::DescriptorBuffer::create(m_gpu_color.positionData, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    m_gpu_color.velocityDescriptor =
        vsg::DescriptorBuffer::create(m_gpu_color.velocityData, 1, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    m_gpu_color.propertyDescriptor =
        vsg::DescriptorBuffer::create(m_gpu_color.propertyData, 2, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    m_gpu_color.colorDescriptor = vsg::DescriptorBuffer::create(
        vsg::BufferInfoList{cloud->color_bufferInfo}, 3, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    m_gpu_color.uniformDescriptor =
        vsg::DescriptorBuffer::create(m_gpu_color.uniformData, 4, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    m_gpu_color.colormapDescriptor =
        vsg::DescriptorBuffer::create(m_gpu_color.colormapData, 5, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);

    vsg::DescriptorSetLayoutBindings bindings = {
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {4, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}};

    m_gpu_color.descriptorSetLayout = vsg::DescriptorSetLayout::create(bindings);
    m_gpu_color.pipelineLayout = vsg::PipelineLayout::create(
        vsg::DescriptorSetLayouts{m_gpu_color.descriptorSetLayout}, vsg::PushConstantRanges{});
    m_gpu_color.pipeline = vsg::ComputePipeline::create(m_gpu_color.pipelineLayout, shaderStage);

    vsg::Descriptors descriptors{m_gpu_color.positionDescriptor, m_gpu_color.velocityDescriptor,
                                 m_gpu_color.propertyDescriptor, m_gpu_color.colorDescriptor,
                                 m_gpu_color.uniformDescriptor, m_gpu_color.colormapDescriptor};
    m_gpu_color.descriptorSet = vsg::DescriptorSet::create(m_gpu_color.descriptorSetLayout, descriptors);

    m_gpu_color.bindPipeline = vsg::BindComputePipeline::create(m_gpu_color.pipeline);
    m_gpu_color.bindDescriptorSets = vsg::BindDescriptorSets::create(
        VK_PIPELINE_BIND_POINT_COMPUTE, m_gpu_color.pipelineLayout, 0,
        vsg::DescriptorSets{m_gpu_color.descriptorSet});
    m_gpu_color.dispatch = vsg::Dispatch::create(1, 1, 1);
    auto memoryBarrier = vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT);
    m_gpu_color.barrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT, 0);
    m_gpu_color.barrier->add(memoryBarrier);

    // Build a reusable command list so we can toggle execution without re-creating nodes
    m_gpu_color.commands = vsg::Commands::create();
    m_gpu_color.commands->addChild(m_gpu_color.bindPipeline);
    m_gpu_color.commands->addChild(m_gpu_color.bindDescriptorSets);
    m_gpu_color.commands->addChild(m_gpu_color.dispatch);
    m_gpu_color.commands->addChild(m_gpu_color.barrier);

    // Register the compute work with the visualisation system's compute command graph
    m_vsys->AddComputeCommands(m_gpu_color.commands);
    cloud->compute_commands = m_gpu_color.commands;

    m_gpu_color.colormapDirty = true;
    UpdateGpuColormapBuffer();

    m_gpu_color.initialized = true;
    return true;
}

void ChSphVisualizationVSG::ConfigureGpuCommands(bool enable) {
    if (!m_gpu_color.commands)
        return;

    // Toggle whether the compute command list is submitted this frame
    if (enable) {
        if (m_gpu_color.commands->children.empty()) {
            m_gpu_color.commands->addChild(m_gpu_color.bindPipeline);
            m_gpu_color.commands->addChild(m_gpu_color.bindDescriptorSets);
            m_gpu_color.commands->addChild(m_gpu_color.dispatch);
            if (m_gpu_color.barrier)
                m_gpu_color.commands->addChild(m_gpu_color.barrier);
        }
    } else {
        if (!m_gpu_color.commands->children.empty())
            m_gpu_color.commands->children.clear();
    }
}

void ChSphVisualizationVSG::EnsureGpuColoringReady(size_t num_particles) {
    auto cloud = GetSphParticleCloud();
    if (!cloud) {
        // No cloud bound yet; make sure we stop submitting compute work
        ConfigureGpuCommands(false);
        m_gpu_color.active = false;
        return;
    }

    const bool enable = ShouldUseGpuColoring(num_particles);
    if (!enable) {
        // Fall back to the old CPU path when the colour callback is disabled or unsupported
        // .. could probably delete this handling and associated once confident the gpu path is good
        ConfigureGpuCommands(false);
        cloud->use_compute_colors = false;
        m_gpu_color.active = false;
        return;
    }

    if (!m_gpu_color.initialized) {
        // lazy allocate GPU resources once number of particles known
        if (!InitializeGpuColoringResources(num_particles)) {
            ConfigureGpuCommands(false);
            cloud->use_compute_colors = false;
            m_gpu_color.active = false;
            return;
        }
    }

    if (m_gpu_color.currentColormapType != m_colormap_type)
        m_gpu_color.colormapDirty = true;

    UpdateGpuColormapBuffer();

    // keep the compute dispatch in sync with the render pass
    ConfigureGpuCommands(true);
    cloud->use_compute_colors = true;
    m_gpu_color.active = true;
}

void ChSphVisualizationVSG::UpdateGpuColoring(size_t num_particles) {
    // Populate GPU buffers with each particle data and update dispatch parameters
    if (!m_gpu_color.initialized || !m_gpu_color.active || num_particles == 0)
        return;

    m_gpu_color.mode = DetermineColorMode();

    auto range = m_color_fun ? m_color_fun->GetDataRange() : ChVector2d(0, 1);
    const float dataMin = static_cast<float>(range.x());
    const float dataMax = static_cast<float>(range.y());
    const float diff = dataMax - dataMin;
    const float invRange = std::abs(diff) > 1e-6f ? 1.0f / diff : 0.0f;

    float upX = 0.0f;
    float upY = 0.0f;
    float upZ = 1.0f;
    if (m_gpu_color.mode == ColorMode::HEIGHT) {
        if (auto height = std::dynamic_pointer_cast<ParticleHeightColorCallback>(m_color_fun)) {
            const auto up = height->GetUpVector();
            upX = static_cast<float>(up.x);
            upY = static_cast<float>(up.y);
            upZ = static_cast<float>(up.z);
        }
    }

    const bool bimodal = m_color_fun && m_color_fun->IsBimodal();
    (*m_gpu_color.uniformData)[0].set(dataMin, dataMax, invRange,
                                      static_cast<float>(static_cast<int>(m_gpu_color.mode)));
    (*m_gpu_color.uniformData)[1].set(upX, upY, upZ, static_cast<float>(num_particles));
    (*m_gpu_color.uniformData)[2].set(static_cast<float>(m_gpu_color.colormapResolution),
                                      bimodal ? 1.0f : 0.0f, 0.0f, 0.0f);
    m_gpu_color.uniformData->dirty();

    const size_t positionCount = std::min(m_pos.size(), num_particles);
    // Helper for packing SoA float3 data into float4 buffers
    const auto copyVec3ToVec4 = [](auto* dst, const auto* src, size_t count, float w_value) {
        for (size_t i = 0; i < count; ++i) {
            auto& out = dst[i];
            out.x = static_cast<float>(src[i].x);
            out.y = static_cast<float>(src[i].y);
            out.z = static_cast<float>(src[i].z);
            out.w = w_value;
        }
    };

    auto* pos_dst = m_gpu_color.positionData->data();
    const auto* pos_src = m_pos.data();
    copyVec3ToVec4(pos_dst, pos_src, positionCount, 1.0f);
    if (positionCount < num_particles)
        std::fill_n(pos_dst + positionCount, num_particles - positionCount, vsg::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    m_gpu_color.positionData->dirty();

    const size_t velocityCount = std::min(m_vel.size(), num_particles);
    auto* vel_dst = m_gpu_color.velocityData->data();
    const auto* vel_src = m_vel.data();
    copyVec3ToVec4(vel_dst, vel_src, velocityCount, 0.0f);
    if (velocityCount < num_particles)
        std::fill_n(vel_dst + velocityCount, num_particles - velocityCount, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    m_gpu_color.velocityData->dirty();

    const size_t propertyCount = std::min(m_prop.size(), num_particles);
    auto* prop_dst = m_gpu_color.propertyData->data();
    const auto* prop_src = m_prop.data();
    copyVec3ToVec4(prop_dst, prop_src, propertyCount, 0.0f);
    if (propertyCount < num_particles)
        std::fill_n(prop_dst + propertyCount, num_particles - propertyCount, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    m_gpu_color.propertyData->dirty();

    const uint32_t groups = static_cast<uint32_t>((num_particles + m_gpu_color.workgroupSize - 1) /
                                                  m_gpu_color.workgroupSize);
    const uint32_t groupCount = groups > 0 ? groups : 1u;

    auto newDispatch = vsg::Dispatch::create(groupCount, 1u, 1u);
    if (m_gpu_color.dispatch) {
        auto& children = m_gpu_color.commands->children;
        for (auto& child : children) {
            if (child == m_gpu_color.dispatch) {
                child = newDispatch;
                break;
            }
        }
    } else {
        m_gpu_color.commands->addChild(newDispatch);
    }
    m_gpu_color.dispatch = newDispatch;
}

void ChSphVisualizationVSG::UpdateGpuColormapBuffer() {
    // Reupload the lookup table if the palette changed (so compute knows what to shade)
    if (!m_gpu_color.initialized || !m_gpu_color.colormapDirty || !m_gpu_color.colormapData)
        return;

    if (!m_colormap)
        m_colormap = chrono_types::make_unique<ChColormap>(m_colormap_type);

    if (m_gpu_color.currentColormapType != m_colormap_type)
        m_colormap->Load(m_colormap_type);

    const size_t resolution = std::max<size_t>(1u, static_cast<size_t>(m_gpu_color.colormapResolution));
    const size_t denom = resolution > 1 ? (resolution - 1) : 1;

    for (size_t i = 0; i < resolution && i < m_gpu_color.colormapData->size(); ++i) {
        const double t = (resolution > 1) ? static_cast<double>(i) / static_cast<double>(denom) : 0.0;
        const ChColor color = m_colormap->Get(t);
        m_gpu_color.colormapData->set(i, vsg::vec4(static_cast<float>(color.R), static_cast<float>(color.G),
                                                   static_cast<float>(color.B), 1.0f));
    }

    m_gpu_color.colormapData->dirty();
    m_gpu_color.currentColormapType = m_colormap_type;
    m_gpu_color.colormapDirty = false;
}

void ChSphVisualizationVSG::OnRender() {
    const size_t num_fluid_particles = m_sysSPH->GetNumFluidMarkers();

    // Copy SPH particle positions from device to host
    m_pos.clear();
    m_pos = m_sysSPH->GetPositions();
    if (m_color_fun) {
        m_vel.clear();
        m_vel = m_sysSPH->GetVelocities();
        ////m_acc.clear();
        ////m_acc = m_sysSPH->GetAccelerations();
        ////m_frc.clear();
        ////m_frc = m_sysSPH->GetForces();
        m_prop.clear();
        m_prop = m_sysSPH->GetProperties();

    }

    EnsureGpuColoringReady(num_fluid_particles);
    if (m_gpu_color.active)
        UpdateGpuColoring(num_fluid_particles);

    // Set members for the callback functors (if defined)
    if (m_color_fun) {
        m_color_fun->pos = m_pos.data();
        m_color_fun->vel = m_vel.data();
        m_color_fun->prop = m_prop.data();
        m_color_fun->cloud = m_sph_cloud.get();
    }
    if (m_vis_sph_fun) {
        m_vis_sph_fun->pos = m_pos.data();
    }
    if (m_vis_bndry_fun) {
        const auto n = m_sysSPH->GetNumFluidMarkers();
        m_vis_bndry_fun->pos = &m_pos.data()[n];
    }

    // For display in VSG GUI
    if (m_sysFSI) {
        m_sysMBS->SetChTime(m_sysFSI->GetSimTime());
        m_sysMBS->SetRTF(m_sysFSI->GetRtf());
    } else {
        m_sysMBS->SetChTime(m_sysSPH->GetSimTime());
        m_sysMBS->SetRTF(m_sysSPH->GetRtf());
    }

    // Set particle positions in the various particle clouds
    size_t p = 0;

    // Utility to push a contiguous block of positions into a target cloud without per-particle loops
    auto bulkWritePositions = [&](const std::shared_ptr<ChParticleCloud>& cloud, size_t offset, size_t count) {
        if (!cloud || count == 0)
            return;
#if defined(CHRONO_SPH_USE_DOUBLE)
        cloud->SetParticlePositions(reinterpret_cast<const double*>(m_pos.data() + offset), count);
#else
        cloud->SetParticlePositions(reinterpret_cast<const float*>(m_pos.data() + offset), count);
#endif
    };

    if (m_sph_markers) {
        bulkWritePositions(m_sph_cloud, p, m_sysSPH->GetNumFluidMarkers());
    }
    p += m_sysSPH->GetNumFluidMarkers();

    if (m_bndry_bce_markers) {
        bulkWritePositions(m_bndry_bce_cloud, p, m_sysSPH->GetNumBoundaryMarkers());
    }
    p += m_sysSPH->GetNumBoundaryMarkers();

    if (m_rigid_bce_markers) {
        bulkWritePositions(m_rigid_bce_cloud, p, m_sysSPH->GetNumRigidBodyMarkers());
    }
    p += m_sysSPH->GetNumRigidBodyMarkers();

    if (m_flex_bce_markers) {
        bulkWritePositions(m_flex_bce_cloud, p, m_sysSPH->GetNumFlexBodyMarkers());
    }

    // Update positions of all active boxes
    for (const auto& child : m_activeBoxScene->children) {
        std::shared_ptr<ChBody> obj;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Object", obj))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;
        if (obj == nullptr) {
            auto hsize = m_sysSPH->GetComputationalDomain().Size() / 2;
            transform->matrix = vsg::dmat4CH(ChFramed(m_sysSPH->GetComputationalDomain().Center(), QUNIT), hsize);
        } else {
            transform->matrix = vsg::dmat4CH(ChFramed(obj->GetPos(), QUNIT), m_active_box_hsize);
        }
    }
}

// ---------------------------------------------------------------------------

MarkerPlanesVisibilityCallback::MarkerPlanesVisibilityCallback(const std::vector<Plane>& planes, Mode mode) : m_planes(planes), m_mode(mode) {}

bool MarkerPlanesVisibilityCallback::get(unsigned int n) const {
    switch (m_mode) {
        case Mode::ANY:
            // Marker in front of any plane is not visible
            for (const auto& plane : m_planes) {
                if (Vdot(ToChVector(pos[n]) - plane.point, plane.normal) > 0)
                    return false;
            }
            return true;
        case Mode::ALL:
            // Marker behind any plane is visible
            for (const auto& plane : m_planes) {
                if (Vdot(ToChVector(pos[n]) - plane.point, plane.normal) <= 0)
                    return true;
            }
            return false;
    }

    return true;
}

// ---------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up)
    : m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

std::string ParticleHeightColorCallback::GetTile() const {
    return "Height (m)";
}

ChVector2d ParticleHeightColorCallback::GetDataRange() const {
    return ChVector2d(m_hmin, m_hmax);
}

ChColor ParticleHeightColorCallback::GetColor(unsigned int n) const {
    double h = dot(pos[n], m_up);  // particle height
    return m_vsys->GetColormap().Get(h, m_hmin, m_hmax);
}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(double vmin, double vmax, Component component)
    : m_vmin(vmin), m_vmax(vmax), m_component(component) {}

std::string ParticleVelocityColorCallback::GetTile() const {
    return "Velocity (m/s)";
}

ChVector2d ParticleVelocityColorCallback::GetDataRange() const {
    return ChVector2d(m_vmin, m_vmax);
}

ChColor ParticleVelocityColorCallback::GetColor(unsigned int n) const {
    double v = 0;
    switch (m_component) {
        case Component::NORM:
            v = length(vel[n]);
            break;
        case Component::X:
            v = std::abs(vel[n].x);
            break;
        case Component::Y:
            v = std::abs(vel[n].y);
            break;
        case Component::Z:
            v = std::abs(vel[n].z);
            break;
    }

    return m_vsys->GetColormap().Get(v, m_vmin, m_vmax);
}

ParticleDensityColorCallback::ParticleDensityColorCallback(double dmin, double dmax) : m_dmin(dmin), m_dmax(dmax) {}

std::string ParticleDensityColorCallback::GetTile() const {
    return "Density (kg/m3)";
}

ChVector2d ParticleDensityColorCallback::GetDataRange() const {
    return ChVector2d(m_dmin, m_dmax);
}

ChColor ParticleDensityColorCallback::GetColor(unsigned int n) const {
    double d = prop[n].x;
    return m_vsys->GetColormap().Get(d, m_dmin, m_dmax);
}

ParticlePressureColorCallback::ParticlePressureColorCallback(double pmin, double pmax, bool bimodal)
    : m_bimodal(bimodal), m_pmin(pmin), m_pmax(pmax) {
    assert(!m_bimodal || m_pmin < 0);
}

std::string ParticlePressureColorCallback::GetTile() const {
    return "Pressure (N/m2)";
}

ChVector2d ParticlePressureColorCallback::GetDataRange() const {
    return ChVector2d(m_pmin, m_pmax);
}

ChColor ParticlePressureColorCallback::GetColor(unsigned int n) const {
    double p = prop[n].y;

    if (m_bimodal) {
        if (p < 0) {
            float factor = (float)(p / m_pmin);  // color scaling factor (1...0)
            return m_vsys->GetColormap().Get(0.5 * (1 - factor));
        } else {
            float factor = (float)(p / m_pmax);  // color scaling factor (0...1)
            return m_vsys->GetColormap().Get(0.5 * (1 + factor));
        }
    } else {
        return m_vsys->GetColormap().Get(p, m_pmin, m_pmax);
    }
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
