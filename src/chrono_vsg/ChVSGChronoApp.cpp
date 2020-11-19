// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
// Vulkan Scene Graph viewer, this class will hopefully draw the system to the
// screen and handle input some day
// =============================================================================

#include "chrono_vsg/core/ChApiVSG.h"

#include "ChVSGChronoApp.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono_vsg/resources/ChVSGPhongMaterial.h"
#include "chrono_vsg/assets/ChPBRSetting.h"
#include "chrono_vsg/shapes/VSGIndexBox.h"
#include "chrono_vsg/shapes/VSGIndexSphere.h"
#include "chrono_vsg/shapes/VSGIndexCylinder.h"

using namespace chrono::vsg3d;

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer, ChVSGChronoApp* appPtr) : m_viewer(viewer), m_app_ptr(appPtr) {}
    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'i') {
            chrono::GetLog() << "Key 'i' pressed " << ++_counter << " times.\n";
        }
    }

  private:
    size_t _counter;
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    ChVSGChronoApp* m_app_ptr;
};

ChVSGChronoApp::ChVSGChronoApp(ChSystem* sys, std::string& windowTitle, size_t windowWidth, size_t windowHeight)
    : m_system(sys), m_build_graph(true) {
    // window parameters
    m_windowTraits = vsg::WindowTraits::create();
    m_windowTraits->windowTitle = windowTitle;
    m_windowTraits->width = windowWidth;
    m_windowTraits->height = windowHeight;

    // set the scene graphs to defaults
    m_scenegraph = vsg::Group::create();
    m_scenegraphText = vsg::Group::create();

    m_clearColor.R = 0.0;
    m_clearColor.G = 0.0;
    m_clearColor.B = 0.0;
    m_clearColor.A = 1.0;

    // set the text font
    m_fontFilename = "fonts/times.vsgb";
    m_searchPaths = ::vsg::getEnvPaths("VSG_FILE_PATH");

    auto options = vsg::Options::create();
    options->paths = m_searchPaths;
#ifdef USE_VSGXCHANGE
    options->readerWriter = vsgXchange::ReaderWriter_all::create();
#endif

    m_font = vsg::read_cast<vsg::Font>(m_fontFilename, options);

    if (!m_font) {
        std::cout << "Failling to read font : " << m_fontFilename << std::endl;
        return;
    }
}

ChVSGChronoApp::~ChVSGChronoApp() {}

bool ChVSGChronoApp::OpenWindow() {
    // create viewer
    m_viewer = ::vsg::Viewer::create();
    // create window
    m_window = ::vsg::Window::create(m_windowTraits);
    //
    bool ok = true;
    if (!m_window) {
        GetLog() << "Could not create windows.\n";
        return ok;
    }
    m_viewer->addWindow(m_window);
    if (m_build_graph) {
        BuildSceneGraph();
        GenerateText();
    }

    uint32_t width = m_window->extent2D().width;
    uint32_t height = m_window->extent2D().height;

    auto renderGraph = vsg::RenderGraph::create(m_window);

    // set clear color
    renderGraph->clearValues[0] = {m_clearColor.R, m_clearColor.G, m_clearColor.B, m_clearColor.A};

    // create view1
    m_mainCamera = createMainCamera(0, 0, width, height);
    auto view1 = vsg::View::create(m_mainCamera, m_scenegraph);
    renderGraph->addChild(view1);

    // clear the depth buffer before view2 gets rendered
    VkClearAttachment attachment{VK_IMAGE_ASPECT_DEPTH_BIT, 1, VkClearValue{1.0f, 0.0f}};
    VkClearRect rect{VkRect2D{VkOffset2D{0, 0}, VkExtent2D{width, height}}, 0, 1};
    auto clearAttachments = vsg::ClearAttachments::create(vsg::ClearAttachments::Attachments{attachment},
                                                          vsg::ClearAttachments::Rects{rect});
    renderGraph->addChild(clearAttachments);

    // create view2
    m_textCamera = createTextCamera(0, 0, width, height);
    auto view2 = vsg::View::create(m_textCamera, m_scenegraphText);
    renderGraph->addChild(view2);

    // special key for application control
    m_viewer->addEventHandler(AppKeyboardHandler::create(m_viewer, this));

    // add close handler to respond the close window button and pressing escape
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    m_viewer->addEventHandler(vsg::Trackball::create(m_mainCamera));

    auto commandGraph = vsg::CommandGraph::create(m_window);

    commandGraph->addChild(renderGraph);

    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    m_viewer->compile();

    return ok;
}

void ChVSGChronoApp::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
}

vsg::ref_ptr<vsg::Camera> ChVSGChronoApp::createMainCamera(int32_t x, int32_t y, uint32_t width, uint32_t height) {
    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    m_scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.001;

    // set up the camera
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 20.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    auto perspective = vsg::Perspective::create(30.0, static_cast<double>(width) / static_cast<double>(height),
                                                nearFarRatio * radius, radius * 4.5);

    auto viewportstate = vsg::ViewportState::create(x, y, width, height);

    return vsg::Camera::create(perspective, lookAt, viewportstate);
}

vsg::ref_ptr<vsg::Camera> ChVSGChronoApp::createTextCamera(int32_t x, int32_t y, uint32_t width, uint32_t height) {
    // compute the bounds of the scene graph to help position camera
    vsg::dvec3 centre = {0.0, 1.0, 0.0};
    double radius = 10.0;
    double nearFarRatio = 0.001;

    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    double halfDim = radius * 1.1;
    double halfHeight, halfWidth;
    double aspectRatio =
        static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height);

    if (m_window->extent2D().width > m_window->extent2D().height) {
        halfHeight = halfDim;
        halfWidth = halfDim * aspectRatio;
    } else {
        halfWidth = halfDim;
        halfHeight = halfDim / aspectRatio;
    }
    auto projection =
        vsg::Orthographic::create(-halfWidth, halfWidth, -halfHeight, halfHeight, nearFarRatio * radius, radius * 4.5);

    return vsg::Camera::create(projection, lookAt, vsg::ViewportState::create(m_window->extent2D()));
}

void ChVSGChronoApp::BuildSceneGraph() {
    // analyse system, look for bodies and assets
    for (auto body : m_system->Get_bodylist()) {
        // position of the body
        const Vector pos = body->GetFrame_REF_to_abs().GetPos();
        // rotation of the body
        Quaternion rot = body->GetFrame_REF_to_abs().GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);
        bool textureFound = false;
        bool colorFound = false;
        bool pbrMapsFound = false;
        bool pbrSetFound = false;
        ChTexture bodyTexture;
        ChPBRSetting bodyPBRSet;
        ChPBRMaps bodyPBRMaps;
        ChColor bodyColor(1.0, 0.0, 0.0, 1.0);
        for (int i = 0; i < body->GetAssets().size(); i++) {
            auto asset = body->GetAssets().at(i);
            if (std::dynamic_pointer_cast<ChColorAsset>(asset)) {
                ChColorAsset* color_asset = (ChColorAsset*)(asset.get());
                bodyColor.R = color_asset->GetColor().R;
                bodyColor.G = color_asset->GetColor().G;
                bodyColor.B = color_asset->GetColor().B;
                bodyColor.A = color_asset->GetColor().A;
                colorFound = true;
            }
            if (std::dynamic_pointer_cast<ChTexture>(asset)) {
                ChTexture* texture_asset = (ChTexture*)(asset.get());
                bodyTexture.SetTextureFilename(texture_asset->GetTextureFilename());
                textureFound = true;
            }
            if (std::dynamic_pointer_cast<ChPBRSetting>(asset)) {
                ChPBRSetting* texture_asset = (ChPBRSetting*)(asset.get());
                bodyPBRSet.SetAlbedo(texture_asset->GetAlbedo());
                bodyPBRSet.SetMetallic(texture_asset->GetMetallic());
                bodyPBRSet.SetRoughness(texture_asset->GetRoughness());
                bodyPBRSet.SetAO(texture_asset->GetAO());
                pbrSetFound = true;
            }
            if (std::dynamic_pointer_cast<ChPBRMaps>(asset)) {
                ChPBRMaps* texture_asset = (ChPBRMaps*)(asset.get());
                bodyPBRMaps.SetAlbedoMapPath(texture_asset->GetAlbedoMapPath());
                bodyPBRMaps.SetNormalMapPath(texture_asset->GetNormalMapPath());
                bodyPBRMaps.SetMetallicMapPath(texture_asset->GetMetallicMapPath());
                bodyPBRMaps.SetRoughnessMapPath(texture_asset->GetRoughnessMapPath());
                bodyPBRMaps.SetAoMapPath(texture_asset->GetAoMapPath());
                pbrMapsFound = true;
            }
        }
        for (int i = 0; i < body->GetAssets().size(); i++) {
            auto asset = body->GetAssets().at(i);

            ChVisualization* visual_asset = ((ChVisualization*)(asset.get()));
            // position of the asset
            Vector center = visual_asset->Pos;
            // rotate asset pos into global frame
            center = rot.Rotate(center);
            // Get the local rotation of the asset
            Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
            // add the local rotation to the rotation of the body
            lrot = rot % lrot;
            lrot.Normalize();
            lrot.Q_to_AngAxis(angle, axis);
            if (ChBoxShape* box_shape = dynamic_cast<ChBoxShape*>(asset.get())) {
                // GetLog() << "Found BoxShape!\n";
                ChVector<> size = box_shape->GetBoxGeometry().GetSize();
                ChVector<> pos_final = pos + center;
                auto transform = vsg::MatrixTransform::create();
                transform->setMatrix(vsg::translate(pos_final.x(), pos_final.y(), pos_final.z()) *
                                     vsg::rotate(angle, axis.x(), axis.y(), axis.z()) *
                                     vsg::scale(size.x(), size.y(), size.z()));
                VSGIndexBox box(body, asset, transform);
                if (textureFound) {
                    box.Initialize(bodyTexture);
                } else if (colorFound) {
                    box.Initialize(bodyColor);
                } else if (pbrSetFound) {
                    box.Initialize(bodyPBRSet);
                } else if (pbrMapsFound) {
                    box.Initialize(bodyPBRMaps);
                }
                vsg::ref_ptr<vsg::Node> node = box.createVSGNode();
                m_scenegraph->addChild(node);
            }
            if (ChSphereShape* sphere_shape = dynamic_cast<ChSphereShape*>(asset.get())) {
                // GetLog() << "Found SphereShape!\n";
                double radius = sphere_shape->GetSphereGeometry().rad;
                ChVector<> size(radius, radius, radius);
                ChVector<> pos_final = pos + center;

                auto transform = vsg::MatrixTransform::create();
                transform->setMatrix(vsg::translate(pos_final.x(), pos_final.y(), pos_final.z()) *
                                     vsg::rotate(angle, axis.x(), axis.y(), axis.z()) *
                                     vsg::scale(size.x(), size.y(), size.z()));

                VSGIndexSphere sphere(body, asset, transform);
                if (textureFound) {
                    sphere.Initialize(bodyTexture);
                } else if (colorFound) {
                    sphere.Initialize(bodyColor);
                } else if (pbrSetFound) {
                    sphere.Initialize(bodyPBRSet);
                } else if (pbrMapsFound) {
                    sphere.Initialize(bodyPBRMaps);
                }
                vsg::ref_ptr<vsg::Node> node = sphere.createVSGNode();
                m_scenegraph->addChild(node);
            }
            if (ChEllipsoidShape* ellipsoid_shape = dynamic_cast<ChEllipsoidShape*>(asset.get())) {
                // GetLog() << "Found ElipsoidShape!\n";
                Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
                ChVector<> size(radius.x(), radius.y(), radius.z());
                ChVector<> pos_final = pos + center;
                auto transform = vsg::MatrixTransform::create();

                transform->setMatrix(vsg::translate(pos_final.x(), pos_final.y(), pos_final.z()) *
                                     vsg::rotate(angle, axis.x(), axis.y(), axis.z()) *
                                     vsg::scale(size.x(), size.y(), size.z()));

                VSGIndexSphere ellipsoid(body, asset, transform);
                if (textureFound) {
                    ellipsoid.Initialize(bodyTexture);
                } else if (colorFound) {
                    ellipsoid.Initialize(bodyColor);
                } else if (pbrSetFound) {
                    ellipsoid.Initialize(bodyPBRSet);
                } else if (pbrMapsFound) {
                    ellipsoid.Initialize(bodyPBRMaps);
                }
                vsg::ref_ptr<vsg::Node> node = ellipsoid.createVSGNode();
                m_scenegraph->addChild(node);
            }
            if (ChCylinderShape* cylinder_shape = dynamic_cast<ChCylinderShape*>(asset.get())) {
                // GetLog() << "Found CylinderShape!\n";
                double radius = cylinder_shape->GetCylinderGeometry().rad;
                ChVector<> dir = cylinder_shape->GetCylinderGeometry().p1 - cylinder_shape->GetCylinderGeometry().p2;
                double height = dir.Length();
                ChVector<> pos_final = pos + center;
                auto transform = vsg::MatrixTransform::create();
                transform->setMatrix(vsg::translate(pos_final.x(), pos_final.y(), pos_final.z()) *
                                     vsg::rotate(angle, axis.x(), axis.y(), axis.z()) *
                                     vsg::scale(radius, radius, height));
                VSGIndexCylinder cylinder(body, asset, transform);
                if (textureFound) {
                    cylinder.Initialize(bodyTexture);
                } else if (colorFound) {
                    cylinder.Initialize(bodyColor);
                } else if (pbrSetFound) {
                    cylinder.Initialize(bodyPBRSet);
                } else if (pbrMapsFound) {
                    cylinder.Initialize(bodyPBRMaps);
                }
                vsg::ref_ptr<vsg::Node> node = cylinder.createVSGNode();
                m_scenegraph->addChild(node);
            }
        }
    }
    m_build_graph = false;
    size_t numCh = m_scenegraph->getNumChildren();
    GetLog() << "Found 3D-Shapes: " << numCh << "\n";
}

void ChVSGChronoApp::GenerateText() {
    {
        auto layout = vsg::LeftAlignment::create();
        layout->position = vsg::vec3(0.0, 0.0, 0.0);
        layout->horizontal = vsg::vec3(1.0, 0.0, 0.0);
        layout->vertical = vsg::vec3(0.0, 0.0, 1.0);
        layout->color = vsg::vec4(1.0, 0.5, 0.3, 1.0);
        layout->outlineWidth = 0.2;

        auto text = vsg::Text::create();
        text->text = vsg::stringValue::create("This is overlayed text.");
        text->font = m_font;
        text->layout = layout;
        text->setup();
        m_scenegraphText->addChild(text);
    }
    {
        auto layout = vsg::LeftAlignment::create();
        layout->position = vsg::vec3(-14.0, 0.0, -10.0);
        layout->horizontal = vsg::vec3(0.5, 0.0, 0.0);
        layout->vertical = vsg::vec3(0.0, 0.0, 0.5);
        layout->color = vsg::vec4(0.0, 0.7, 0.7, 1.0);
        layout->outlineWidth = 0.2;

        auto text = vsg::Text::create();
        text->text = vsg::stringValue::create("This is different overlayed text. It is on a different place.");
        text->font = m_font;
        text->layout = layout;
        text->setup();
        m_scenegraphText->addChild(text);
    }
}
