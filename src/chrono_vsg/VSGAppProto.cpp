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
// Vulkan Scene Graph viewer class
// =============================================================================

#include <iostream>

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

#include "chrono_vsg/VSGAppProto.h"
#include "chrono_vsg/shapes/VSGAsys.h"
#include "chrono_vsg/shapes/VSGBsys.h"
#include "chrono_vsg/shapes/VSGCsys.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCapsuleShape.h"

namespace chrono {
    namespace vsg3d {

        struct Params : public vsg::Inherit<vsg::Object, Params> {
            bool showGui = true;  // you can toggle this with your own EventHandler and key
            bool showGlobalCsys = false;
            bool showBodySys = false;
            bool showAssetSys = false;
            float symSize = 1.0;
            int drawMode = 0;
        };

        class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
        public:
            AppKeyboardHandler(vsg::Viewer *viewer) : m_viewer(viewer) {
            }

            void SetParams(vsg::ref_ptr<Params> params, VSGAppProto *appPtr) {
                _params = params;
                m_appPtr = appPtr;
            }

            void apply(vsg::KeyPressEvent &keyPress) override {
                if (keyPress.keyBase == '1') {
                    _params->drawMode = 0;
                    m_appPtr->UpdateDrawMode(_params->drawMode);
                }
                if (keyPress.keyBase == '2') {
                    _params->drawMode = 1;
                    m_appPtr->UpdateDrawMode(_params->drawMode);
                }
                if (keyPress.keyBase == '3') {
                    _params->drawMode = 2;
                    m_appPtr->UpdateDrawMode(_params->drawMode);
                }
                if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
                    _params->showGui = !_params->showGui;
                }
            }

        private:
            vsg::observer_ptr<vsg::Viewer> m_viewer;
            vsg::ref_ptr<Params> _params;
            VSGAppProto *m_appPtr;
        };

        class MyGuiComponent {
        public:
            MyGuiComponent(vsg::ref_ptr<Params> params, VSGAppProto *appPtr) : _params(params), m_appPtr(appPtr) {
            }

            // Example here taken from the Dear imgui comments (mostly)
            bool operator()() {
                bool visibleComponents = false;
                ImGuiIO &io = ImGui::GetIO();
#ifdef __APPLE__
                io.FontGlobalScale = 2.0;
#else
                io.FontGlobalScale = 1.0;
#endif

                // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
                if (_params->showGui) {
                    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
                    ImGui::Begin("Chrono::VSG GUI");  // Create a window called "Hello, world!" and append into it.

                    ImGui::Text("3D Draw Mode: ");
                    ImGui::SameLine();

                    if (ImGui::RadioButton("Filled", &_params->drawMode, 0)) {
                        m_appPtr->UpdateDrawMode(_params->drawMode);
                    }
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Wireframe", &_params->drawMode, 1)) {
                        m_appPtr->UpdateDrawMode(_params->drawMode);
                    }
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Ref. Frames", &_params->drawMode, 2)) {
                        m_appPtr->UpdateDrawMode(_params->drawMode);
                    }

                    ImGui::Text("Show Ref. Frames: ");
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Global", &_params->showGlobalCsys)) {
                        m_appPtr->setGlobalFrameVisibility(_params->showGlobalCsys);
                    }
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Bodies", &_params->showBodySys)) {
                        m_appPtr->setBodyFrameVisibility(_params->showBodySys);
                    }
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Assets", &_params->showAssetSys)) {
                        m_appPtr->setAssetFrameVisibility(_params->showAssetSys);
                    }

                    ImGui::Text("     Symbol Size: ");
                    ImGui::SameLine();
                    ImGui::SliderFloat("", &_params->symSize, 0.1, 10.0);
                    m_appPtr->UpdateFrameSize(_params->symSize);

                    if (ImGui::Button(
                            "Quit"))  // Buttons return true when clicked (most widgets return true when edited/activated)
                        m_appPtr->Quit();

                    ImGui::End();
                    visibleComponents = true;
                }

                return visibleComponents;
            }

        private:
            vsg::ref_ptr<Params> _params;
            VSGAppProto *m_appPtr;
        };


        VSGAppProto::VSGAppProto() : m_up_vector(vsg::dvec3(0, 0, 1)), m_system(nullptr) {
        }

        VSGAppProto::~VSGAppProto() {
        }

        bool VSGAppProto::Initialize(int windowWidth, int windowHeight, const char *windowTitle, ChSystem *system) {
            m_system = system;

            auto options = vsg::Options::create();
            options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
            options->objectCache = vsg::ObjectCache::create();

            auto windowTraits = vsg::WindowTraits::create();
            windowTraits->windowTitle = windowTitle;
            windowTraits->width = windowWidth;
            windowTraits->height = windowHeight;

            // enable transfer from the colour and depth buffer images
            windowTraits->swapchainPreferences.imageUsage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
            windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

            // initialize stateInfos for builder class
            m_stateInfoPolygon.lighting = true;
            m_stateInfoPolygon.wireframe = false;

            m_stateInfoWireFrame.lighting = true;
            m_stateInfoWireFrame.wireframe = true;

            // create the viewer and assign window(s) to it
            m_viewer = vsg::Viewer::create();

            m_window = vsg::Window::create(windowTraits);
            if (!m_window) {
                std::cout << "Could not create windows." << std::endl;
                return 1;
            }

            auto device = m_window->getOrCreateDevice();

            m_builderLighting = vsg::Builder::create();
            m_builderLighting->options = options;

            m_viewer->addWindow(m_window);

            m_globalSymTransform = vsg::MatrixTransform::create();

            // holds whole 3d stuff
            m_scenegraph = vsg::Group::create();

            // add ambient light
            auto ambientLight = vsg::AmbientLight::create();
            ambientLight->name = "ambient";
            ambientLight->color.set(1.0, 1.0, 1.0);
            ambientLight->intensity = 0.1;

            auto directionalLight = vsg::DirectionalLight::create();
            directionalLight->name = "head light";
            directionalLight->color.set(1.0, 1.0, 1.0);
            directionalLight->intensity = 0.9;
            directionalLight->direction.set(0.0, 0.0, -1.0);

            auto absoluteTransform = vsg::AbsoluteTransform::create();
            absoluteTransform->addChild(ambientLight);
            absoluteTransform->addChild(directionalLight);

            m_scenegraph->addChild(absoluteTransform);

            // subparts, only one active, for easy access
            m_dot_subgraph = vsg::Switch::create();
            m_line_subgraph = vsg::Switch::create();
            m_polygon_subgraph = vsg::Switch::create();
            m_global_sym_subgraph = vsg::Switch::create();
            m_body_sym_subgraph = vsg::Switch::create();
            m_asset_sym_subgraph = vsg::Switch::create();

            // add switch nodes to m_scenegraph
            m_scenegraph->addChild(m_dot_subgraph);
            m_scenegraph->addChild(m_line_subgraph);
            m_scenegraph->addChild(m_polygon_subgraph);
            m_scenegraph->addChild(m_global_sym_subgraph);
            m_scenegraph->addChild(m_body_sym_subgraph);
            m_scenegraph->addChild(m_asset_sym_subgraph);

            // use chrono multibody system the generate 3d info
            BuildSceneGraph();

            // compute the bounds of the scene graph to help position camera
            vsg::ComputeBounds computeBounds;
            m_scenegraph->accept(computeBounds);
            vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
            double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;

            // These are set statically because the geometry in the class is expanded in the shader
            double nearFarRatio = 0.01;

            // set up the camera
            m_lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

            m_perspective = vsg::Perspective::create(
                    30.0, static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
                    nearFarRatio * radius, radius * 400.5);

            m_camera = vsg::Camera::create(m_perspective, m_lookAt, vsg::ViewportState::create(m_window->extent2D()));

            // The commandGraph will contain a 2 stage renderGraph 1) 3D scene 2) ImGui (by default also includes clear depth
            // buffers)
            m_commandGraph = vsg::CommandGraph::create(m_window);
            m_renderGraph = vsg::RenderGraph::create(m_window);
            m_commandGraph->addChild(m_renderGraph);

            // create the normal 3D view of the scene
            m_renderGraph->addChild(vsg::View::create(m_camera, m_scenegraph));

            // Create the ImGui node and add it to the renderGraph
            auto params = Params::create();
            m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, MyGuiComponent(params, this)));

            // Add the ImGui event handler first to handle events early
            m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

            auto kbHandler = AppKeyboardHandler::create(m_viewer);
            kbHandler->SetParams(params, this);
            m_viewer->addEventHandler(kbHandler);

            // add close handler to respond the close window button and pressing escape
            m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

            m_viewer->addEventHandler(vsg::Trackball::create(m_camera));

            auto event = vsg::Event::create(m_window->getOrCreateDevice());  // Vulkan creates vkEvent in an unsignled state
            // Add ScreenshotHandler to respond to keyboard and mouse events.
            m_screenshotHandler = VSGScreenshotHandler::create(event);
            if (event)
                m_commandGraph->addChild(vsg::SetEvent::create(event, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT));

            m_viewer->addEventHandler(m_screenshotHandler);

            m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});

            m_viewer->compile();

            return true;
        }

        void VSGAppProto::UpdateDrawMode(int mode) {
            m_drawMode = mode;
            m_drawModeChanged = true;
        }

        void VSGAppProto::setGlobalFrameVisibility(bool v) {
            m_global_sym_subgraph->setAllChildren(v);
        }

        void VSGAppProto::setBodyFrameVisibility(bool v) {
            m_body_sym_subgraph->setAllChildren(v);
        }

        void VSGAppProto::setAssetFrameVisibility(bool v) {
            m_asset_sym_subgraph->setAllChildren(v);
        }

        void VSGAppProto::Quit() {
            m_viewer->close();
        }

        void VSGAppProto::UpdateFrameSize(float newSize) {
            if (newSize != m_symSize) {
                m_symSize = newSize;
                // exactly one global symbol, always at {0;0;0}, never rotated
                m_globalSymTransform->matrix = vsg::scale(m_symSize, m_symSize, m_symSize);
                // exactly one body reference, can be at any position and rotation
                for (size_t i = 0; i < m_system->Get_bodylist().size(); i++) {
                    auto body = m_system->Get_bodylist().at(i);
                    // position of the body
                    const Vector pos = body->GetFrame_REF_to_abs().GetPos();
                    // rotation of the body
                    Quaternion rot = body->GetFrame_REF_to_abs().GetRot();
                    double angle;
                    Vector axis;
                    rot.Q_to_AngAxis(angle, axis);
                    double s = m_symSize;
                    vsg::ref_ptr<vsg::MatrixTransform> body_tf;
                    bool tf_ok = m_body_sym_subgraph->children[i].node->getValue("transform", body_tf);
                    if (tf_ok) {
                        body_tf->matrix = vsg::translate(pos.x(), pos.y(), pos.z())
                                * vsg::rotate(angle, axis.x(), axis.y(), axis.z())
                                * vsg::scale(s, s, s);
                    }
                }
                // each body can have an arbitrary number of assets (0 included) at any position and orientation
                size_t kNode = 0;
                size_t maxNode = m_asset_sym_subgraph->children.size() - 1;
                for (size_t i = 0; i < m_system->Get_bodylist().size(); i++) {
                    auto body = m_system->Get_bodylist().at(i);
                    for (size_t j = 0; j < body->GetAssets().size(); j++) {
                        if (kNode > maxNode) {
                            GetLog() << "Asset Symbols Graph is inconsistent with MBS\n";
                        }
                        auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(body->GetAssets().at(j));
                        ChFrame<> g_X_b(body->GetFrame_REF_to_abs().GetPos(), body->GetFrame_REF_to_abs().GetRot());
                        // asset position and rotation wrt body frame
                        const ChFrame<> b_X_a(visual_asset->Pos, visual_asset->Rot);
                        // asset position and rotation wrt global frame
                        auto g_X_a = g_X_b * b_X_a;
                        double angle;
                        Vector axis;
                        g_X_a.GetRot().Q_to_AngAxis(angle, axis);
                        double s = m_symSize;
                        bool tf_ok = false;
                        vsg::ref_ptr<vsg::MatrixTransform> tf;
                        tf_ok = m_asset_sym_subgraph->children[kNode++].node->getValue("transform", tf);
                        tf->matrix = vsg::translate(g_X_a.GetPos().x(), g_X_a.GetPos().y(), g_X_a.GetPos().z())
                                * vsg::rotate(angle, axis.x(), axis.y(), axis.z())
                                * vsg::scale(s, s, s);
                    }
                }
            }
        }

        static vsg::dmat4 vsgXform(const ChFrame<> &X, const ChVector<> &s) {
            const ChVector<> &t = X.GetPos();
            const ChQuaternion<> &q = X.GetRot();
            double angle;
            ChVector<> axis;
            q.Q_to_AngAxis(angle, axis);

            auto m1 = vsg::translate(t.x(), t.y(), t.z());
            auto m2 = vsg::rotate(angle, axis.x(), axis.y(), axis.z());
            auto m3 = vsg::scale(s.x(), s.y(), s.z());

            return m1 * m2 * m3;
        }

        void VSGAppProto::BuildSceneGraph() {
            GetLog() << "Bodies found = " << m_system->Get_bodylist().size() << "\n";

            for (auto body: m_system->Get_bodylist()) {
                GetLog() << "processing body " << body.get()->GetId() << "\n";
                // Provide 3D symbol for each body
                // body position and rotation wrt global frame
                ChFrame<> g_X_b(body->GetFrame_REF_to_abs().GetPos(), body->GetFrame_REF_to_abs().GetRot());

                vsg::GeometryInfo geomDot;
                geomDot.transform = vsgXform(g_X_b, ChVector<>(0.1));

                m_dot_subgraph->addChild(false, m_builderLighting->createSphere(geomDot, m_stateInfoPolygon));

                // Provide reference frame symbol for each body
                // position of the body
                const Vector pos = body->GetFrame_REF_to_abs().GetPos();
                // rotation of the body
                Quaternion rot = body->GetFrame_REF_to_abs().GetRot();
                double angle;
                Vector axis;
                rot.Q_to_AngAxis(angle, axis);
                double s = m_symSize;
                vsg::ref_ptr<vsg::MatrixTransform> body_tf = vsg::MatrixTransform::create();
                body_tf->matrix = vsg::translate(pos.x(), pos.y(), pos.z())
                        * vsg::rotate(angle, axis.x(), axis.y(), axis.z())
                        * vsg::scale(s, s, s);
                VSGBsys bsys(body);
                bsys.genSubgraph(m_body_sym_subgraph, body_tf);
                for (const auto &asset: body->GetAssets()) {
                    auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset);
                    if (!visual_asset) {
                        continue;
                    }

                    // asset position and rotation wrt body frame
                    const ChFrame<> b_X_a(visual_asset->Pos, visual_asset->Rot);

                    // asset position and rotation wrt global frame
                    auto g_X_a = g_X_b * b_X_a;
                    const Vector apos = g_X_a.GetPos();
                    Quaternion arot = g_X_a.GetRot();
                    double a_angle;
                    Vector a_axis;
                    arot.Q_to_AngAxis(a_angle, a_axis);
                    double as = m_symSize;
                    vsg::ref_ptr<vsg::MatrixTransform> asset_tf = vsg::MatrixTransform::create();
                    asset_tf->matrix = vsg::translate(apos.x(), apos.y(), apos.z())
                            * vsg::rotate(a_angle, a_axis.x(), a_axis.y(), a_axis.z())
                            * vsg::scale(as, as, as);
                    VSGAsys asys(body, visual_asset);
                    asys.genSubgraph(m_asset_sym_subgraph, asset_tf);
                    if (ChSphereShape *sphere_shape = dynamic_cast<ChSphereShape *>(asset.get())) {
                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_a, ChVector<>(2 * sphere_shape->GetSphereGeometry().rad));

                        m_polygon_subgraph->addChild(true, m_builderLighting->createSphere(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createSphere(geomInfo, m_stateInfoWireFrame));
                    } else if (ChEllipsoidShape *ellipsoid_shape = dynamic_cast<ChEllipsoidShape *>(asset.get())) {
                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_a, ellipsoid_shape->GetEllipsoidGeometry().rad * 2);

                        m_polygon_subgraph->addChild(true, m_builderLighting->createSphere(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createSphere(geomInfo, m_stateInfoWireFrame));
                    } else if (ChBoxShape *box_shape = dynamic_cast<ChBoxShape *>(asset.get())) {
                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_a, box_shape->GetBoxGeometry().GetLengths());

                        m_polygon_subgraph->addChild(true, m_builderLighting->createBox(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createBox(geomInfo, m_stateInfoWireFrame));
                    } else if (ChCylinderShape *cylinder_shape = dynamic_cast<ChCylinderShape *>(asset.get())) {
                        double rad = cylinder_shape->GetCylinderGeometry().rad;
                        const ChVector<> &p1 = cylinder_shape->GetCylinderGeometry().p1;
                        const ChVector<> &p2 = cylinder_shape->GetCylinderGeometry().p2;

                        ChVector<> dir = p2 - p1;
                        double height = dir.Length();
                        dir /= height;

                        ChVector<> mx, my, mz;
                        dir.DirToDxDyDz(mx, my, mz);
                        ChMatrix33<> M;
                        M.Set_A_axis(mx, my, mz);                           // mx || dir
                        M *= ChMatrix33<>(CH_C_PI_2, ChVector<>(0, 1, 0));  // mz || dir
                        ChQuaternion<> mrot = M.Get_A_quaternion();

                        // cylinder position and rotation wrt asset
                        ChFrame<> a_X_c(0.5 * (p2 + p1), M);
                        // cylinder position and rotation wrt global frame
                        auto g_X_c = g_X_a * a_X_c;

                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_c, ChVector<>(2 * rad, 2 * rad, height));

                        m_polygon_subgraph->addChild(true, m_builderLighting->createCylinder(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createCylinder(geomInfo, m_stateInfoWireFrame));
                    } else if (ChConeShape *cone_shape = dynamic_cast<ChConeShape *>(asset.get())) {
                        double rx = cone_shape->GetConeGeometry().rad.x();
                        double ry = cone_shape->GetConeGeometry().rad.z();
                        double height = cone_shape->GetConeGeometry().rad.y();

                        // cone position and rotation wrt asset frame
                        ChFrame<> a_X_c(ChVector<>(0), Q_from_AngX(-CH_C_PI_2));
                        // cone position and rotation wrt global frame
                        auto g_X_c = g_X_a * a_X_c;

                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_c, ChVector<>(2 * rx, 2 * ry, height));

                        m_polygon_subgraph->addChild(true, m_builderLighting->createCone(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createCone(geomInfo, m_stateInfoWireFrame));
                    } else if (ChCapsuleShape *capsule_shape = dynamic_cast<ChCapsuleShape *>(asset.get())) {
                        double rad = capsule_shape->GetCapsuleGeometry().rad;
                        double hlen = capsule_shape->GetCapsuleGeometry().hlen;

                        // capsule position and rotation wrt asset frame
                        ChFrame<> a_X_c(ChVector<>(0), Q_from_AngX(-CH_C_PI_2));
                        // capsule position and rotation wrt global frame
                        auto g_X_c = g_X_a * a_X_c;

                        vsg::GeometryInfo geomInfo;
                        geomInfo.transform = vsgXform(g_X_c, ChVector<>(2 * rad, 2 * rad, 2 * hlen));

                        m_polygon_subgraph->addChild(true, m_builderLighting->createCapsule(geomInfo, m_stateInfoPolygon));
                        m_line_subgraph->addChild(false, m_builderLighting->createCapsule(geomInfo, m_stateInfoWireFrame));
                    }
                }
            }
            VSGCsys csys;
            csys.genSubgraph(m_global_sym_subgraph, m_globalSymTransform);
            m_global_sym_subgraph->setAllChildren(false);
            m_body_sym_subgraph->setAllChildren(false);
            m_asset_sym_subgraph->setAllChildren(false);
            m_dot_subgraph->setAllChildren(false);
            m_line_subgraph->setAllChildren(false);
            m_polygon_subgraph->setAllChildren(true);
        }

        void VSGAppProto::UpdateSceneGraph() {
        }

        void VSGAppProto::Render() {
            m_viewer->handleEvents();
            m_viewer->update();
            m_viewer->recordAndSubmit();
            // adjust draw mode, if necessary
            if (m_drawModeChanged) {
                switch (m_drawMode) {
                    case 0:
                        m_dot_subgraph->setAllChildren(false);
                        m_line_subgraph->setAllChildren(false);
                        m_polygon_subgraph->setAllChildren(true);
                        break;
                    case 1:
                        m_dot_subgraph->setAllChildren(false);
                        m_line_subgraph->setAllChildren(true);
                        m_polygon_subgraph->setAllChildren(false);
                        break;
                    case 2:
                        m_dot_subgraph->setAllChildren(true);
                        m_line_subgraph->setAllChildren(false);
                        m_polygon_subgraph->setAllChildren(false);
                        break;
                }
                m_drawModeChanged = false;
            }
            if (m_screenshotHandler->do_image_capture)
                m_screenshotHandler->screenshot_image(m_window);
            if (m_screenshotHandler->do_depth_capture)
                m_screenshotHandler->screenshot_depth(m_window);
            m_viewer->present();
        }

    }  // namespace vsg3d
}  // namespace chrono
