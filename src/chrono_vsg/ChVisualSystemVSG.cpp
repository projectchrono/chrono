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
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>
#include "chrono_vsg/tools/createSkybox.h"
#include "chrono_vsg/tools/createQuad.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChObjFileShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/physics/ChParticleCloud.h"
#include "tools/exportScreenshot.h"
#include "tools/exportScreenshotNoBlit.h"
#include "ChVisualSystemVSG.h"
#include <algorithm>
#include <string>
#include <cstddef>
#include <cctype>

namespace chrono {
namespace vsg3d {

using namespace std;

ChVisualSystemVSG::ChVisualSystemVSG() {
    m_windowTitle = string("Window Title");
    m_clearColor = ChColor(0.0, 0.0, 0.0);
    m_skyboxPath = string("vsg/textures/chrono_skybox.ktx2");
    m_useSkybox = false;
    m_cameraUpVector = vsg::dvec3(0, 0, 1);
    m_yup = false;
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::SetWindowSize(ChVector2<int> size) {
    m_windowWidth = size[0];
    m_windowHeight = size[1];
}

void ChVisualSystemVSG::SetWindowPosition(ChVector2<int> pos) {
    m_windowX = pos[0];
    m_windowY = pos[1];
}

void ChVisualSystemVSG::SetWindowTitle(std::string title) {
    m_windowTitle = title;
}

void ChVisualSystemVSG::SetClearColor(ChColor color) {
    m_clearColor = color;
}

void ChVisualSystemVSG::SetUseSkyBox(bool yesno) {
    m_useSkybox = yesno;
}

void ChVisualSystemVSG::AddCamera(const ChVector<>& pos, ChVector<> targ) {
    m_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
    m_cameraTarget = vsg::dvec3(targ.x(), targ.y(), targ.z());
}

void ChVisualSystemVSG::SetCameraVertical(CameraVerticalDir upDir) {
    switch (upDir) {
        case CameraVerticalDir::Y:
            m_cameraUpVector = vsg::dvec3(0, 1, 0);
            m_yup = true;
            break;
        case CameraVerticalDir::Z:
            m_cameraUpVector = vsg::dvec3(0, 0, 1);
            m_yup = false;
            break;
    }
}

void ChVisualSystemVSG::SetLightDirection(double acimut, double elevation) {
    m_acimut = ChClamp(acimut, -CH_C_PI, CH_C_PI);
    m_elevation = ChClamp(elevation, 0.0, CH_C_PI_2);
}

void ChVisualSystemVSG::Initialize() {
    // set up defaults and read command line arguments to override them
    auto options = vsg::Options::create();
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    options->paths.push_back(GetChronoDataPath());
    // add vsgXchange's support for reading and writing 3rd party file formats, mandatory for chrono_vsg!
    options->add(vsgXchange::all::create());
    options->sharedObjects = vsg::SharedObjects::create();
    m_shapeBuilder = ShapeBuilder::create();
    m_shapeBuilder->m_options = options;
    m_shapeBuilder->m_sharedObjects = options->sharedObjects;

    auto builder = vsg::Builder::create();
    builder->options = options;

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = m_windowTitle;
    windowTraits->width = m_windowWidth;
    windowTraits->height = m_windowHeight;
    windowTraits->x = m_windowX;
    windowTraits->y = m_windowY;
    windowTraits->deviceExtensionNames = {VK_KHR_MULTIVIEW_EXTENSION_NAME, VK_KHR_MAINTENANCE2_EXTENSION_NAME,
            VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
            VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME};
    windowTraits->swapchainPreferences.imageUsage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 1.0f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 1.0f);

    vsg::StateInfo stateInfo;
    vsg::dvec3 centre = {0.0, 0.0, 0.0};

    m_scene = vsg::Group::create();

    double radius = 50.0;
    vsg::dbox bound;

    if (m_useSkybox) {
        vsg::Path fileName(m_skyboxPath);
        auto skyPtr = createSkybox(fileName, options, m_yup);
        if (skyPtr)
            m_scene->addChild(skyPtr);
        else
            m_useSkybox = false;
    }


    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0, 1.0, 1.0);
    ambientLight->intensity = 0.01;

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "head light";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = m_lightIntensity;
    if (m_yup) {
        directionalLight->direction.set(-cos(m_elevation) * cos(m_acimut), -sin(m_elevation),
                                        -cos(m_elevation) * sin(m_acimut));
    } else {
        directionalLight->direction.set(-cos(m_elevation) * cos(m_acimut), -cos(m_elevation) * sin(m_acimut),
                                        -sin(m_elevation));
    }

    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);

    m_scene->addChild(absoluteTransform);

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }
    m_window->clearColor() = VkClearColorValue{{m_clearColor.R, m_clearColor.G, m_clearColor.B, 1}};
    m_viewer->addWindow(m_window);

    vsg::ref_ptr<vsg::LookAt> lookAt;

    // compute the bounds of the scene graph to help position camera
    // vsg::ComputeBounds computeBounds;
    // scene->accept(computeBounds);
    // vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    // double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6 * 10.0;

    // set up the camera
    lookAt = vsg::LookAt::create(m_cameraEye, m_cameraTarget, m_cameraUpVector);

    double nearFarRatio = 0.001;
    auto perspective = vsg::Perspective::create(
        m_cameraAngleDeg, static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(m_window->extent2D()));

    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    m_viewer->addEventHandler(vsg::Trackball::create(camera));

    auto commandGraph = vsg::createCommandGraphForView(m_window, camera, m_scene);
    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    // assign a CompileTraversal to the Builder that will compile for all the views assigned to the viewer,
    // must be done after Viewer.assignRecordAndSubmitTasksAndPresentations();
    auto compileTraversal = vsg::CompileTraversal::create(*m_viewer);
    m_shapeBuilder->assignCompileTraversal(compileTraversal);

    m_viewer->compile();
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->advanceToNextFrame();
}

void ChVisualSystemVSG::Render() {
    // pass any events into EventHandlers assigned to the Viewer
    m_viewer->handleEvents();

    m_viewer->update();

    m_viewer->recordAndSubmit();
#ifdef WIN32
    if (m_do_image_export) exportScreenshotNoBlit(m_window,m_imageFilename);
#else
    if (m_do_image_export) exportScreenshot(m_window,m_imageFilename);
#endif
    m_do_image_export = false;

    m_viewer->present();
}

void ChVisualSystemVSG::WriteImageToFile(const string& filename) {
    GetLog() << "Dumping Image\n";
    m_imageFilename = filename;
    m_do_image_export = true;
}

void ChVisualSystemVSG::BindAll() {
    cout << "BindAll() called!" << endl;
    if (!m_system) {
        cout << "No system attached, nothing to bind!" << endl;
        return;
    }
    if (m_system->Get_bodylist().size() < 1) {
        cout << "Attached system must have at least 1 rigid body, nothing to bind!" << endl;
        return;
    }
    for (auto& body : m_system->GetAssembly().Get_bodylist()) {
        // CreateIrrNode(body);
        GetLog() << "Body# " << body->GetId() << "\n";
        if (!body->GetVisualModel()) {
            GetLog() << "   ... has no visual representation\n";
            continue;
        }
        // Get the visual model reference frame
        const ChFrame<>& X_AM = body->GetVisualModelFrame();

        for (const auto& shape_instance : body->GetVisualModel()->GetShapes()) {
            auto& shape = shape_instance.first;
            auto& X_SM = shape_instance.second;
            ChFrame<> X_SA = X_AM * X_SM;
            auto pos = X_SA.GetPos();
            auto rot = X_SA.GetRot();
            double rotAngle;
            ChVector<> rotAxis;
            rot.Q_to_AngAxis(rotAngle, rotAxis);
            std::shared_ptr<ChVisualMaterial> material;
            if (shape->GetMaterials().empty()) {
                material = chrono_types::make_shared<ChVisualMaterial>();
                material->SetDiffuseColor(ChColor(1.0, 1.0, 1.0));
                material->SetAmbientColor(ChColor(0.1, 0.1, 0.1));
            } else {
                material = shape->GetMaterial(0);
            }
            if (!shape->IsVisible()) {
                continue;
            }
            if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
                GetLog() << "... has a box shape\n";
                ChVector<> scale = box->GetBoxGeometry().Size;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::BOX_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
                GetLog() << "... has a sphere shape\n";
                ChVector<> scale = sphere->GetSphereGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::SPHERE_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
                GetLog() << "... has a ellipsoid shape\n";
                ChVector<> scale = ellipsoid->GetEllipsoidGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::SPHERE_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
                GetLog() << "... has a capsule shape\n";
                double rad = capsule->GetCapsuleGeometry().rad;
                double height = capsule->GetCapsuleGeometry().hlen;
                auto transform = vsg::MatrixTransform::create();
                ChVector<> scale(rad, height, rad);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::CAPSULE_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            } else if (auto barrel = std::dynamic_pointer_cast<ChBarrelShape>(shape)) {
                GetLog() << "... has a barrel shape (to do)\n";
            } else if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
                GetLog() << "... has a cone shape\n";
                Vector rad = cone->GetConeGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(rad.x(), rad.y(), rad.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::CONE_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            } else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
                GetLog() << "... has a triangle mesh shape\n";
                ChVector<> scale = trimesh->GetScale();
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::TRIANGLE_MESH_SHAPE, body,
                        shape_instance, material, transform,
                        m_draw_as_wireframe, trimesh));
            } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
                GetLog() << "... has a surface mesh shape\n";
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(1.0, 1.0, 1.0);
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::SURFACE_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe, nullptr,
                        surface));
            } else if (auto obj = std::dynamic_pointer_cast<ChObjFileShape>(shape)) {
                GetLog() << "... has a obj file shape (to do)\n";
            } else if (auto line = std::dynamic_pointer_cast<ChLineShape>(shape)) {
                GetLog() << "... has a line shape\n";
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(1.0, 1.0, 1.0);
                m_scene->addChild(
                        m_shapeBuilder->createLineShape(body, shape_instance, material, transform, line));
            } else if (auto path = std::dynamic_pointer_cast<ChPathShape>(shape)) {
                GetLog() << "... has a path shape\n";
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(1.0, 1.0, 1.0);
                m_scene->addChild(
                        m_shapeBuilder->createPathShape(body, shape_instance, material, transform, path));
            } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                GetLog() << "... has a cylinder shape\n";
                double rad = cylinder->GetCylinderGeometry().rad;
                const auto& P1 = cylinder->GetCylinderGeometry().p1;
                const auto& P2 = cylinder->GetCylinderGeometry().p2;

                ChVector<> dir = P2 - P1;
                double height = dir.Length();
                dir.Normalize();
                ChVector<> mx, my, mz;
                dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
                ChMatrix33<> R_CS;
                R_CS.Set_A_axis(mx, my, mz);

                auto t_CS = 0.5 * (P2 + P1);
                ChFrame<> X_CS(t_CS, R_CS);
                ChFrame<> X_CA = X_SA * X_CS;

                pos = X_CA.GetPos();
                rot = X_CA.GetRot();
                rot.Q_to_AngAxis(rotAngle, rotAxis);

                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(rad, height, rad);
                m_scene->addChild(m_shapeBuilder->createShape(ShapeBuilder::CYLINDER_SHAPE, body, shape_instance,
                        material, transform, m_draw_as_wireframe));
            }
        }
    }
}

void ChVisualSystemVSG::OnUpdate() {
    //GetLog() << "Update requested.\n";
    for (auto child: m_scene->children) {
        std::shared_ptr<ChPhysicsItem> item;
        ChVisualModel::ShapeInstance shapeInstance;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child->getValue("ItemPtr", item))
            continue;
        if (!child->getValue("ShapeInstancePtr", shapeInstance))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        // begin matrix update

        // Get the visual model reference frame
        const ChFrame<> &X_AM = item->GetVisualModelFrame();
        auto shape = shapeInstance.first;
        const auto &X_SM = shapeInstance.second;

        ChFrame<> X_SA = X_AM * X_SM;
        vsg::dvec3 pos(X_SA.GetPos().x(), X_SA.GetPos().y(), X_SA.GetPos().z());
        auto rot = X_SA.GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);
        vsg::dvec3 rotax(axis.x(), axis.y(), axis.z());
        if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            vsg::dvec3 size(box->GetBoxGeometry().GetSize().x(), box->GetBoxGeometry().GetSize().y(),
                    box->GetBoxGeometry().GetSize().z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            double radius = sphere->GetSphereGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius, radius, radius);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto line = std::dynamic_pointer_cast<ChLineShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto path = std::dynamic_pointer_cast<ChPathShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
            vsg::dvec3 size(trimesh->GetScale().x(), trimesh->GetScale().y(), trimesh->GetScale().z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            ChVector<> radius = ellipsoid->GetEllipsoidGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius.x(), radius.y(), radius.z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
            ChVector<> radius = cone->GetConeGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius.x(), radius.y(), radius.z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
            double rad = capsule->GetCapsuleGeometry().rad;
            double height = capsule->GetCapsuleGeometry().hlen;
            ChVector<> scale(rad, height, rad);
            transform->matrix =
                    vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(scale.x(), scale.y(), scale.z());
        } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            double radius = cylinder->GetCylinderGeometry().rad;
            double rad = cylinder->GetCylinderGeometry().rad;
            const auto &P1 = cylinder->GetCylinderGeometry().p1;
            const auto &P2 = cylinder->GetCylinderGeometry().p2;

            ChVector<> dir = P2 - P1;
            double height = dir.Length();
            dir.Normalize();
            ChVector<> mx, my, mz;
            dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
            ChMatrix33<> R_CS;
            R_CS.Set_A_axis(mx, my, mz);

            auto t_CS = 0.5 * (P2 + P1);
            ChFrame<> X_CS(t_CS, R_CS);
            ChFrame<> X_CA = X_SA * X_CS;

            auto pos = X_CA.GetPos();
            rot = X_CA.GetRot();
            double rotAngle;
            ChVector<> rotAxis;
            rot.Q_to_AngAxis(rotAngle, rotAxis);

            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                    vsg::scale(rad, height, rad);
        }
    }
}

}  // namespace vsg3d
}  // namespace chrono