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
// Radu Serban, Alessandro Tasora
// =============================================================================

//// RADU TODO
//// Allow attaching more than one ChSystem to the same Irrlicht visualization

#include <codecvt>
#include <locale>

#include "chrono/utils/ChProfiler.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeBarrel.h"
#include "chrono/assets/ChVisualShapeCapsule.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include "chrono_irrlicht/ChIrrSkyBoxSceneNode.h"

namespace chrono {
namespace irrlicht {

using namespace irr;
using namespace irr::scene;

static std::shared_ptr<video::SMaterial> default_material;

ChVisualSystemIrrlicht::ChVisualSystemIrrlicht()
    : m_device_params(irr::SIrrlichtCreationParameters()),
      m_device(nullptr),
      m_monospace_font(nullptr),
      m_container(nullptr),
      m_win_title(""),
      m_yup(true),
      m_use_effects(false),
      m_quality(0) {
    // Set default device parameter values
    m_device_params.AntiAlias = true;
    m_device_params.Bits = 32;
    m_device_params.Fullscreen = false;
    m_device_params.DriverType = video::EDT_DIRECT3D9;
    m_device_params.WindowSize = core::dimension2d<irr::u32>(640, 480);
    m_device_params.Stencilbuffer = false;
    m_device_params.LoggingLevel = irr::ELL_INFORMATION;

    // Create the GUI
    m_gui = std::unique_ptr<ChIrrGUI>(new ChIrrGUI());

    // Create shared meshes
    sphereMesh = createEllipticalMesh(1.0, 1.0, -2, +2, 0, 15, 8);
    cubeMesh = createCubeMesh(core::vector3df(2, 2, 2));  // -/+ 1 unit each xyz axis
    cylinderMesh = createCylinderMesh(1, 1, 32);
    capsuleMesh = createCapsuleMesh(1, 1, 32, 32);
    coneMesh = createConeMesh(1, 1, 32);

    // if (sphereMesh)
    //  sphereMesh->grab();
    if (cubeMesh)
        cubeMesh->grab();
    if (cylinderMesh)
        cylinderMesh->grab();
    if (capsuleMesh)
        capsuleMesh->grab();
    if (coneMesh)
        coneMesh->grab();
}

ChVisualSystemIrrlicht::ChVisualSystemIrrlicht(ChSystem* sys,
                                               const ChVector3d& camera_pos,
                                               const ChVector3d& camera_targ)
    : ChVisualSystemIrrlicht() {
    AttachSystem(sys);
    SetWindowSize(800, 600);
    SetWindowTitle("Chrono::Engine");
    Initialize();

    AddLogo();
    AddSkyBox();
    AddTypicalLights();
    AddCamera(camera_pos, camera_targ);
}

ChVisualSystemIrrlicht::~ChVisualSystemIrrlicht() {
    if (sphereMesh)
        sphereMesh->drop();
    if (cubeMesh)
        cubeMesh->drop();
    if (cylinderMesh)
        cylinderMesh->drop();
    if (capsuleMesh)
        capsuleMesh->drop();

    if (m_device)
        m_device->drop();
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::SetAntialias(bool val) {
    m_device_params.AntiAlias = val;
}

void ChVisualSystemIrrlicht::SetFullscreen(bool val) {
    m_device_params.Fullscreen = val;
}

void ChVisualSystemIrrlicht::SetShadows(bool val) {
    m_device_params.Stencilbuffer = val;
}

void ChVisualSystemIrrlicht::SetDriverType(video::E_DRIVER_TYPE driver_type) {
    m_device_params.DriverType = driver_type;
}

void ChVisualSystemIrrlicht::SetWindowSize(unsigned int width, unsigned int height) {
    m_device_params.WindowSize = core::dimension2d<irr::u32>((u32)width, (u32)height);
}

void ChVisualSystemIrrlicht::SetWindowTitle(const std::string& win_title) {
    m_win_title = win_title;
}

void ChVisualSystemIrrlicht::SetWindowId(void* window_id) {
    m_device_params.WindowId = window_id;
};

void ChVisualSystemIrrlicht::SetLogLevel(irr::ELOG_LEVEL log_level) {
    m_device_params.LoggingLevel = log_level;
}

void ChVisualSystemIrrlicht::SetCameraVertical(CameraVerticalDir vert) {
    m_yup = (vert == CameraVerticalDir::Y);
}
CameraVerticalDir ChVisualSystemIrrlicht::GetCameraVertical() {
    return (m_yup == true ? CameraVerticalDir::Y : CameraVerticalDir::Z);
}

void ChVisualSystemIrrlicht::SetSymbolScale(double scale) {
    m_gui->symbolscale = scale;
    if (m_gui->initialized)
        m_gui->SetSymbolScale(scale);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::AttachSystem(ChSystem* sys) {
    ChVisualSystem::AttachSystem(sys);

    // If the visualization system is already initialized
    if (m_device) {
        assert(!m_gui->initialized);
        m_gui->Initialize(this);

        // Parse the mechanical assembly and create a ChIrrNodeModel for each physics item with a visual model.
        // This is a recursive call to accomodate any existing sub-assemblies.
        BindAll();
    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::Initialize() {
    if (m_initialized)
        return;

    if (!m_verbose)
        m_device_params.LoggingLevel = irr::ELL_NONE;

    // Create Irrlicht device using current parameter values.
    m_device = irr::createDeviceEx(m_device_params);
    if (!m_device) {
        std::cerr << "Cannot use default video driver - fall back to OpenGL" << std::endl;
        m_device_params.DriverType = video::EDT_OPENGL;
        m_device = irr::createDeviceEx(m_device_params);
        if (!m_device) {
            std::cerr << "Failed to create the video driver - giving up" << std::endl;
            return;
        }
    }

    m_device->grab();

    std::wstring title = std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_win_title);
    m_device->setWindowCaption(title.c_str());

    // Xeffects for shadow maps!
    if (m_device_params.AntiAlias)
        m_effect_handler = std::unique_ptr<EffectHandler>(
            new EffectHandler(m_device, GetVideoDriver()->getScreenSize() * 2, true, false, true));
    else
        m_effect_handler = std::unique_ptr<EffectHandler>(
            new EffectHandler(m_device, GetVideoDriver()->getScreenSize(), true, false, true));
    m_effect_handler->setAmbientColor(video::SColor(255, 122, 122, 122));
    m_use_effects = false;  // will be true as sson as a light with shadow is added

    // Create a fixed-size font.
    m_monospace_font = GetGUIEnvironment()->getFont(GetChronoDataFile("fonts/jetbrainmono6_bold.png").c_str());

    // Create the container Irrlicht scene node
    m_container = GetSceneManager()->addEmptySceneNode();

    // Create default Irrlicht material
    if (!default_material) {
        auto irr_mat = tools::ToIrrlichtMaterial(ChVisualMaterial::Default(), GetVideoDriver());
        default_material = std::make_shared<video::SMaterial>(irr_mat);
    }

    // If the visualization system is already attached to a ChSystem
    if (!m_systems.empty()) {
        // Create an Irrlicht GUI
        assert(!m_gui->initialized);
        m_gui->Initialize(this);

        // Parse the mechanical assembly and create a ChIrrNodeModel for each physics item with a visual model.
        // This is a recursive call to accomodate any existing sub-assemblies.
        BindAll();
    }

    m_initialized = true;
}

// -----------------------------------------------------------------------------

bool ChVisualSystemIrrlicht::Run() {
    assert(!m_systems.empty());
    return m_device->run();
}

void ChVisualSystemIrrlicht::Quit() {
    m_device->closeDevice();
}

void ChVisualSystemIrrlicht::OnSetup(ChSystem* sys) {
    PurgeIrrNodes();
}

void ChVisualSystemIrrlicht::OnUpdate(ChSystem* sys) {
    for (auto& node : m_nodes) {
        node.second->UpdateChildren();
    }
}

void ChVisualSystemIrrlicht::OnClear(ChSystem* sys) {
    for (auto& node : m_nodes) {
        node.second->removeAll();
        node.second->remove();
    }
    m_nodes.clear();
}

void ChVisualSystemIrrlicht::PurgeIrrNodes() {
    // Remove Irrlicht nodes associated with a deleted physics item
    std::vector<ChPhysicsItem*> items_to_remove;
    for (auto& node : m_nodes) {
        if (node.second->GetPhysicsItem().expired()) {
            node.second->removeAll();
            node.second->remove();
            items_to_remove.emplace_back(node.first);
        }
    }

    //// RADU TODO - what if the visual model of the associated node was modified?!?!
    ////   We may now have Irrlicht scene nodes associated with visual shapes that no longer exist!

    for (auto&& item : items_to_remove)
        m_nodes.erase(item);
}

// -----------------------------------------------------------------------------

int ChVisualSystemIrrlicht::AddCamera(const ChVector3d& pos, ChVector3d targ) {
    if (!m_device)
        return -1;

    // create and init camera
    auto camera = chrono_types::make_shared<RTSCamera>(m_device, GetSceneManager()->getRootSceneNode(),
                                                       GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);
    // camera->bindTargetAndRotation(true);
    if (!m_yup)
        camera->setZUp();
    camera->setPosition(core::vector3dfCH(pos));
    camera->setTarget(core::vector3dfCH(targ));

    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);

    m_cameras.push_back(camera);
    return (int)m_cameras.size() - 1;
}

void ChVisualSystemIrrlicht::AddGrid(double x_step, double y_step, int nx, int ny, ChCoordsys<> pos, ChColor col) {
    m_grids.push_back({x_step, y_step, nx, ny, pos, col});
}

void ChVisualSystemIrrlicht::UpdateGrid(int id, const ChCoordsys<>& csys) {
    m_grids[id].csys = csys;
}

void ChVisualSystemIrrlicht::SetCameraPosition(int id, const ChVector3d& pos) {
    m_cameras[id]->setPosition(core::vector3dfCH(pos));
}

void ChVisualSystemIrrlicht::SetCameraTarget(int id, const ChVector3d& target) {
    m_cameras[id]->setTarget(core::vector3dfCH(target));
}

void ChVisualSystemIrrlicht::SetCameraPosition(const ChVector3d& pos) {
    GetActiveCamera()->setPosition(core::vector3dfCH(pos));
}

void ChVisualSystemIrrlicht::SetCameraTarget(const ChVector3d& target) {
    GetActiveCamera()->setTarget(core::vector3dfCH(target));
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::AddLogo(const std::string& logo_filename) {
    if (!m_device)
        return;

    GetGUIEnvironment()->addImage(GetVideoDriver()->getTexture(logo_filename.c_str()),
                                  core::position2d<irr::s32>(10, 10));
}

void ChVisualSystemIrrlicht::AddTypicalLights() {
    if (!m_device)
        return;

    if (m_yup) {
        AddLight(ChVector3d(30, 80, +30), 280, ChColor(0.7f, 0.7f, 0.7f));
        AddLight(ChVector3d(30, 80, -30), 280, ChColor(0.7f, 0.7f, 0.7f));
    } else {
        AddLight(ChVector3d(30, +30, 80), 280, ChColor(0.7f, 0.7f, 0.7f));
        AddLight(ChVector3d(30, -30, 80), 280, ChColor(0.7f, 0.7f, 0.7f));
    }
}

void ChVisualSystemIrrlicht::AddSkyBox(const std::string& texture_dir) {
    if (!m_device)
        return;

    // create sky box
    std::string str_lf = texture_dir + "sky_lf.jpg";
    std::string str_up = texture_dir + "sky_up.jpg";
    std::string str_dn = texture_dir + "sky_dn.jpg";

    video::ITexture* map_skybox_side = GetVideoDriver()->getTexture(str_lf.c_str());

    // Create a skybox scene node
    auto skybox =
        new CSkyBoxSceneNode(GetVideoDriver()->getTexture(str_up.c_str()), GetVideoDriver()->getTexture(str_dn.c_str()),
                             map_skybox_side, map_skybox_side, map_skybox_side, map_skybox_side,
                             GetSceneManager()->getRootSceneNode(), GetSceneManager(), -1);
    skybox->drop();

    if (!m_yup)
        skybox->setRotation(core::vector3df(90, 0, 0));
}

ILightSceneNode* ChVisualSystemIrrlicht::AddLightDirectional(double elevation,
                                                             double azimuth,
                                                             ChColor ambient,
                                                             ChColor specular,
                                                             ChColor diffuse) {
    if (!m_device)
        return nullptr;

    ILightSceneNode* light = GetSceneManager()->addLightSceneNode();
    light->setPosition(core::vector3df(0, 0, 0));
    light->setRotation(core::vector3df(0, 90 + ChClamp(elevation, 0.0, 90.0), ChClamp(azimuth, 0.0, 360.0)));

    video::SLight& l = light->getLightData();
    l.Type = video::ELT_DIRECTIONAL;
    l.Direction = core::vector3df(0, 0, 0);
    l.AmbientColor = tools::ToIrrlichtSColorf(ambient);
    l.SpecularColor = tools::ToIrrlichtSColorf(specular);
    l.DiffuseColor = tools::ToIrrlichtSColorf(diffuse);
    l.CastShadows = false;

    return light;
}

ILightSceneNode* ChVisualSystemIrrlicht::AddLight(const ChVector3d& pos, double radius, ChColor color) {
    if (!m_device)
        return nullptr;

    ILightSceneNode* light = GetSceneManager()->addLightSceneNode(0, core::vector3dfCH(pos),
                                                                  tools::ToIrrlichtSColorf(color), (irr::f32)radius);
    return light;
}

ILightSceneNode* ChVisualSystemIrrlicht::AddLightWithShadow(const ChVector3d& pos,
                                                            const ChVector3d& aim,
                                                            double radius,
                                                            double near_value,
                                                            double far_value,
                                                            double angle,
                                                            unsigned int resolution,
                                                            ChColor color,
                                                            bool directional,
                                                            bool clipborder) {
    if (!m_device)
        return nullptr;

    ILightSceneNode* light = GetSceneManager()->addLightSceneNode(0, core::vector3dfCH(pos),
                                                                  tools::ToIrrlichtSColorf(color), (irr::f32)radius);

    m_effect_handler->addShadowLight(SShadowLight((irr::u32)resolution, core::vector3dfCH(pos), core::vector3dfCH(aim),
                                                  tools::ToIrrlichtSColorf(color), (irr::f32)near_value,
                                                  (irr::f32)far_value, (irr::f32)angle * core::DEGTORAD, directional));

    if (!clipborder)
        m_effect_handler->getShadowLight(m_effect_handler->getShadowLightCount() - 1).setClipBorder(false);

    m_use_effects = true;
    return light;
}

void ChVisualSystemIrrlicht::AddUserEventReceiver(irr::IEventReceiver* receiver) {
    m_gui->AddUserEventReceiver(receiver);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::EnableShadows(std::shared_ptr<ChPhysicsItem> item) {
    if (!m_device) {
        std::cerr << "EnableShadows - visualization system not initialized" << std::endl;
        return;
    }

    if (m_systems.empty()) {
        std::cerr << "EnableShadows - visualization system not attached to a ChSystem" << std::endl;
        return;
    }

    if (item) {
        auto node = m_nodes.find(item.get());
        if (node != m_nodes.end())
            AddShadowToIrrNode(node->second.get());
    } else {
        for (auto& body : m_systems[0]->GetBodies()) {
            EnableShadows(body);
        }
        for (auto& link : m_systems[0]->GetLinks()) {
            EnableShadows(link);
        }
        for (auto& mesh : m_systems[0]->GetMeshes()) {
            EnableShadows(mesh);
        }
        for (auto& ph : m_systems[0]->GetOtherPhysicsItems()) {
            EnableShadows(ph);
        }
    }
}

void ChVisualSystemIrrlicht::AddShadowToIrrNode(ISceneNode* node) {
    ISceneNodeList::ConstIterator it = node->getChildren().begin();
    for (; it != node->getChildren().end(); ++it) {
        AddShadowToIrrNode(*it);
    }

    // Add shadow only to leaves
    if (node->getChildren().getSize() == 0)
        m_effect_handler->addShadowToNode(node);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::EnableContactDrawing(ContactsDrawMode mode) {
    if (m_gui->initialized)
        m_gui->SetContactsDrawMode(mode);
}

void ChVisualSystemIrrlicht::EnableLinkDrawing(LinkDrawMode mode) {
    if (m_gui->initialized)
        m_gui->SetLinksDrawMode(mode);
}

void ChVisualSystemIrrlicht::EnableBodyFrameDrawing(bool val) {
    if (m_gui->initialized)
        m_gui->SetPlotCOGFrames(val);
}

void ChVisualSystemIrrlicht::EnableLinkFrameDrawing(bool val) {
    if (m_gui->initialized)
        m_gui->SetPlotLinkFrames(val);
}

void ChVisualSystemIrrlicht::EnableCollisionShapeDrawing(bool val) {
    if (m_gui->initialized)
        m_gui->SetPlotCollisionShapes(val);
}

void ChVisualSystemIrrlicht::EnableAbsCoordsysDrawing(bool val) {
    if (m_gui->initialized)
        m_gui->SetPlotAbsCoordsys(val);
}

void ChVisualSystemIrrlicht::ShowInfoPanel(bool val) {
    m_gui->show_infos = val;
}

void ChVisualSystemIrrlicht::SetInfoTab(int ntab) {
    if (m_gui->initialized)
        m_gui->SetInfoTab(ntab);
}

void ChVisualSystemIrrlicht::ShowProfiler(bool val) {
    m_gui->show_profiler = val;
}

void ChVisualSystemIrrlicht::ShowExplorer(bool val) {
    m_gui->show_explorer = val;
}

void ChVisualSystemIrrlicht::ShowConvergencePlot(bool val) {
    m_gui->SetPlotConvergence(val);
}

// -----------------------------------------------------------------------------

// Clean canvas at beginning of scene.

void ChVisualSystemIrrlicht::BeginScene() {
    BeginScene(true, true, ChColor(0, 0, 0));
}

void ChVisualSystemIrrlicht::BeginScene(bool backBuffer, bool zBuffer, ChColor color) {
    assert(!m_systems.empty());

    utils::ChProfileManager::Reset();
    utils::ChProfileManager::Start_Profile("Irrlicht loop");
    utils::ChProfileManager::Increment_Frame_Counter();

    GetVideoDriver()->beginScene(backBuffer, zBuffer, tools::ToIrrlichtSColor(color));

    m_gui->BeginScene();
}

// Call this to end the scene draw at the end of each animation frame.
void ChVisualSystemIrrlicht::EndScene() {
    assert(!m_systems.empty());

    utils::ChProfileManager::Stop_Profile();

    m_gui->EndScene();

    GetVideoDriver()->endScene();
}

void ChVisualSystemIrrlicht::Render() {
    assert(!m_systems.empty());

    if (m_use_effects)
        m_effect_handler->update();  // draw 3D scene using Xeffects for shadow maps
    else
        GetSceneManager()->drawAll();  // draw 3D scene the usual way, if no shadow maps

    for (auto& g : m_grids) {
        irrlicht::tools::drawGrid(this, g.x_step, g.y_step, g.nx, g.ny, g.csys, g.col, true);
    }

    m_gui->Render();
}

void ChVisualSystemIrrlicht::RenderFrame(const ChFrame<>& frame, double axis_length) {
    const auto& loc = frame.GetPos();
    const auto& u = frame.GetRotMat().GetAxisX();
    const auto& v = frame.GetRotMat().GetAxisY();
    const auto& w = frame.GetRotMat().GetAxisZ();
    irrlicht::tools::drawSegment(this, loc, loc + u * axis_length, ChColor(1, 0, 0));
    irrlicht::tools::drawSegment(this, loc, loc + v * axis_length, ChColor(0, 1, 0));
    irrlicht::tools::drawSegment(this, loc, loc + w * axis_length, ChColor(0, 0, 1));
}

void ChVisualSystemIrrlicht::RenderCOGFrames(double axis_length) {
    irrlicht::tools::drawAllCOGs(this, axis_length);
}

void ChVisualSystemIrrlicht::WriteImageToFile(const std::string& filename) {
    video::IImage* image = GetVideoDriver()->createScreenShot();
    if (image) {
        GetVideoDriver()->writeImageToFile(image, filename.c_str(), m_quality);
        image->drop();
    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (m_systems.empty() || !m_device)
        return;

    CreateIrrNode(item);
}

void ChVisualSystemIrrlicht::BindAll() {
    if (m_systems.empty() || !m_device)
        return;

    PurgeIrrNodes();

    std::unordered_set<const ChAssembly*> trace;
    CreateIrrNodes(&m_systems[0]->GetAssembly(), trace);
}

void ChVisualSystemIrrlicht::UnbindItem(std::shared_ptr<ChPhysicsItem> item) {
    auto node = m_nodes.find(item.get());
    if (node != m_nodes.end()) {
        node->second->removeAll();
        node->second->remove();
    }
}

// -----------------------------------------------------------------------------

int ChVisualSystemIrrlicht::AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) {
    if (!m_device)
        return -1;

    // Create an Irrlicht scene node for a visualization-only model and populate it
    auto node = chrono_types::make_shared<ChIrrNodeVisual>(m_container, GetSceneManager());
    PopulateIrrNode(node.get(), model, frame);

    core::matrix4CH irrMat(frame);
    node->setPosition(irrMat.getTranslation());
    node->setRotation(irrMat.getRotationDegrees());

    // Cache the new node and return its ID
    m_vis_nodes.push_back(node);
    return (int)m_vis_nodes.size() - 1;
}

int ChVisualSystemIrrlicht::AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    auto model = chrono_types::make_shared<ChVisualModel>();
    model->AddShape(shape);
    return AddVisualModel(model, frame);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::UpdateVisualModel(int id, const ChFrame<>& frame) {
    assert(id >= 0 && id < m_vis_nodes.size());

    core::matrix4CH irrMat(frame);
    m_vis_nodes[id]->setPosition(irrMat.getTranslation());
    m_vis_nodes[id]->setRotation(irrMat.getRotationDegrees());
}

void ChVisualSystemIrrlicht::CreateIrrNodes(const ChAssembly* assembly, std::unordered_set<const ChAssembly*>& trace) {
    // Do nothing if the assembly was already processed
    if (!trace.insert(assembly).second)
        return;

    for (auto& body : assembly->GetBodies()) {
        CreateIrrNode(body);
    }

    for (auto& link : assembly->GetLinks()) {
        CreateIrrNode(link);
    }

    for (auto& mesh : assembly->GetMeshes()) {
        CreateIrrNode(mesh);
    }

    for (auto& ph : assembly->GetOtherPhysicsItems()) {
        CreateIrrNode(ph);

        // Recursively process sub-assemblies
        if (auto a = std::dynamic_pointer_cast<ChAssembly>(ph)) {
            CreateIrrNodes(a.get(), trace);
        }
    }
}

void ChVisualSystemIrrlicht::CreateIrrNode(std::shared_ptr<ChPhysicsItem> item) {
    if (!item->GetVisualModel())
        return;

    // Do nothing if an Irrlicht node already exists for this physics item
    if (m_nodes.find(item.get()) != m_nodes.end())
        return;

    // Create a new ChIrrNodeModel and populate it
    auto node = chrono_types::make_shared<ChIrrNodeModel>(item, m_container, GetSceneManager(), 0);
    bool ok = m_nodes.insert({item.get(), node}).second;
    assert(ok);

    // Remove all Irrlicht scene nodes from the ChIrrNodeModel
    node->removeAll();

    // If the physics item uses clones of its visual model, create an intermediate Irrlicht scene node
    ISceneNode* fillnode = node.get();
    if (item->GetNumVisualModelClones() > 0) {
        fillnode = GetSceneManager()->addEmptySceneNode(node.get());
    }

    // Recursively populate the ChIrrNodeModel with Irrlicht scene nodes for each visual shape.
    // Begin with identity transform relative to the physics item.
    ChFrame<> frame;
    PopulateIrrNode(fillnode, item->GetVisualModel(), frame);
}

static void mflipSurfacesOnX(IMesh* mesh) {
    if (!mesh)
        return;

    const u32 bcount = mesh->getMeshBufferCount();
    for (u32 b = 0; b < bcount; ++b) {
        IMeshBuffer* buffer = mesh->getMeshBuffer(b);

        const u32 vertcnt = buffer->getVertexCount();
        for (u32 i = 0; i < vertcnt; i++) {
            buffer->getPosition(i).X = -buffer->getPosition(i).X;  // mirror vertex
            buffer->getNormal(i).X = -buffer->getNormal(i).X;      // mirrors normal on X
        }
    }
}

static void SetVisualMaterial(ISceneNode* node, std::shared_ptr<ChVisualShape> shape) {
    if (shape->GetMaterials().empty()) {
        // Use default material
        for (u32 i = 0; i < node->getMaterialCount(); i++)
            node->getMaterial(i) = *default_material;
    } else {
        // ChVisualShape might have one or many materials associated
        // a) associate ChVisualShape material to Irrlicht node material until ChVisualShape are consumed
        for (u32 i = 0; i < std::min(node->getMaterialCount(), (u32)shape->GetNumMaterials()); i++)
            node->getMaterial(i) =
                tools::ToIrrlichtMaterial(shape->GetMaterial(i), node->getSceneManager()->getVideoDriver());

        // b) if more materials are required by Irrlicht node it will rollback to ChVisualShape::GetMaterial(0)
        //    in this way the user can set just one material and it will be propagated to all Irrlicht nodes
        //    (e.g. each OBJ files might have many materials indeed)
        if ((u32)shape->GetNumMaterials() < node->getMaterialCount()) {
            for (u32 i = (u32)shape->GetNumMaterials(); i < node->getMaterialCount(); i++)
                node->getMaterial(i) =
                    tools::ToIrrlichtMaterial(shape->GetMaterial(0), node->getSceneManager()->getVideoDriver());
        }
    }

    for (u32 i = 0; i < node->getMaterialCount(); i++)
        node->getMaterial(i).ColorMaterial = video::ECM_NONE;
}

void ChVisualSystemIrrlicht::PopulateIrrNode(ISceneNode* node,
                                             std::shared_ptr<ChVisualModel> model,
                                             const ChFrame<>& parent_frame) {
    for (const auto& shape_instance : model->GetShapeInstances()) {
        auto& shape = shape_instance.first;
        auto& shape_frame = shape_instance.second;
        core::matrix4CH shape_m4(shape_frame);

        if (!shape->IsVisible())
            continue;

        if (auto obj = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
            bool irrmesh_already_loaded = false;
            if (GetSceneManager()->getMeshCache()->getMeshByName(obj->GetFilename().c_str()))
                irrmesh_already_loaded = true;
            IAnimatedMesh* genericMesh = GetSceneManager()->getMesh(obj->GetFilename().c_str());
            if (genericMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(obj, node);
                ISceneNode* mchildnode = GetSceneManager()->addAnimatedMeshSceneNode(genericMesh, mproxynode);
                mproxynode->drop();

                // mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);

                // Note: the Irrlicht loader of .OBJ files flips the X to correct its left-handed nature, but
                // this goes wrong with our assemblies and links. Restore the X flipping of the mesh.
                if (!irrmesh_already_loaded)
                    mflipSurfacesOnX(((IAnimatedMeshSceneNode*)mchildnode)->getMesh());

                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());
                mchildnode->setScale(core::vector3dfCH(obj->GetScale()));

                SetVisualMaterial(mchildnode, shape);
                mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true);
            }
        } else if (auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
            // Create a number of Irrlicht mesh buffers equal to the number of materials.
            // If no materials defined, create a single mesh buffer.
            SMesh* smesh = new SMesh;
            int nbuffers = (int)trimesh->GetNumMaterials();
            nbuffers = std::max(nbuffers, 1);
            for (int ibuffer = 0; ibuffer < nbuffers; ibuffer++) {
                CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
                smesh->addMeshBuffer(buffer);
                buffer->drop();
            }

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(trimesh, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(smesh, mproxynode);
            smesh->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            SetVisualMaterial(mchildnode, shape);
            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, trimesh->IsWireframe());
            mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trimesh->IsBackfaceCull());
        } else if (auto surf = std::dynamic_pointer_cast<ChVisualShapeSurface>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(surf, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            SetVisualMaterial(mchildnode, shape);
            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, surf->IsWireframe());
        } else if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
            if (sphereMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(sphere, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
                mproxynode->drop();

                double mradius = sphere->GetRadius();
                mchildnode->setScale(core::vector3dfCH(ChVector3d(mradius, mradius, mradius)));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, sphere);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
            if (sphereMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(ellipsoid, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
                mproxynode->drop();

                mchildnode->setScale(core::vector3dfCH(ellipsoid->GetSemiaxes()));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, ellipsoid);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
            if (cylinderMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(cylinder, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(cylinderMesh, mproxynode);
                mproxynode->drop();

                double rad = cylinder->GetRadius();
                double height = cylinder->GetHeight();

                core::vector3df irrsize((f32)rad, (f32)rad, (f32)(height / 2));
                mchildnode->setScale(irrsize);
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, cylinder);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto capsule = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
            if (capsuleMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(capsule, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(capsuleMesh, mproxynode);
                mproxynode->drop();

                double rad = capsule->GetRadius();
                double height = capsule->GetHeight();

                core::vector3df irrsize((f32)rad, (f32)rad, (f32)(rad / 2 + height / 4));
                mchildnode->setScale(irrsize);
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, capsule);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
            if (cubeMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(box, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(cubeMesh, mproxynode);
                mproxynode->drop();

                mchildnode->setScale(core::vector3dfCH(box->GetHalflengths()));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, box);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto barrel = std::dynamic_pointer_cast<ChVisualShapeBarrel>(shape)) {
            auto mbarrelmesh = createEllipticalMesh((irr::f32)(barrel->GetRhor()), (irr::f32)(barrel->GetRvert()),
                                                    (irr::f32)(barrel->GetHlow()), (irr::f32)(barrel->GetHsup()),
                                                    (irr::f32)(barrel->GetRoffset()), 15, 8);
            ISceneNode* mproxynode = new ChIrrNodeShape(barrel, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(mbarrelmesh, mproxynode);
            mproxynode->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            SetVisualMaterial(mchildnode, barrel);
            mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
        } else if (auto glyphs = std::dynamic_pointer_cast<ChGlyphs>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(glyphs, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            SetVisualMaterial(mchildnode, glyphs);

            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
        } else if (std::dynamic_pointer_cast<ChVisualShapePath>(shape) ||
                   std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(shape, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            SetVisualMaterial(mchildnode, shape);

            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
        } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
            if (coneMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(cone, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(coneMesh, mproxynode);
                mproxynode->drop();

                double rad = cone->GetRadius();
                double height = cone->GetHeight();

                core::vector3df irrsize((irr::f32)rad, (irr::f32)rad, (irr::f32)height);
                mchildnode->setScale(irrsize);
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, cone);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        }
    }
}

}  // namespace irrlicht
}  // namespace chrono
