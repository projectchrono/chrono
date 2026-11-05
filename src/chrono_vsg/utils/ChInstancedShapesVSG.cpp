// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
#include <iostream>
#include <map>
#include <tuple>

#include "chrono/assets/ChVisualShapes.h"
#include "chrono/collision/ChCollisionShapes.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChInstancedShapesVSG.h"
#include "chrono_vsg/utils/ChShaderUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

namespace chrono {
namespace vsg3d {

namespace {

// Template kinds: unit primitives (scaled per instance) and distinct geometries
enum TemplateKind { SPHERE, BOX, CYLINDER, CONE, CAPSULE, ROUNDED_BOX, ROUNDED_CYLINDER, MESH };

}  // namespace

ChInstancedShapesVSG::ChInstancedShapesVSG()
    : m_source(ShapeSource::COLLISION),
      m_size_factor(10),
      m_num_divs(24),
      m_material(ChVisualMaterial::Default()),
      m_default_color(ChColor(0.75f, 0.75f, 0.75f)),
      m_show_inactive(false),
      m_visible(true),
      m_median_size(0),
      m_num_instances(0),
      m_num_visible(0) {}

ChInstancedShapesVSG::~ChInstancedShapesVSG() {}

void ChInstancedShapesVSG::SetBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    m_selection = bodies;
}

void ChInstancedShapesVSG::SetResolution(int num_divs) {
    m_num_divs = std::max(num_divs, 4);
}

void ChInstancedShapesVSG::SetVisibility(bool visible) {
    m_visible = visible;
    if (m_root)
        m_root->children[0].mask = vsg::boolToMask(visible);
}

double ChInstancedShapesVSG::GetBodySize(const ChBody& body, ShapeSource source) {
    ChAABB aabb;
    if (source == ShapeSource::COLLISION) {
        auto model = body.GetCollisionModel();
        if (!model || model->GetNumShapes() == 0)
            return 0;
        aabb = model->GetBoundingBox(true);
    } else {
        auto model = body.GetVisualModel();
        if (!model || model->GetNumShapes() == 0)
            return 0;
        aabb = model->GetBoundingBox();
    }
    if (aabb.IsInverted())
        return 0;
    auto size = aabb.Size();
    return std::max({size.x(), size.y(), size.z()});
}

// -----------------------------------------------------------------------------

ChInstancedShapesVSG::Geometry ChInstancedShapesVSG::CreateMeshGeometry(const ChTriangleMeshConnected& mesh) {
    const auto& vertices = mesh.GetCoordsVertices();
    const auto& faces = mesh.GetIndicesVertices();
    size_t num_vertices = 3 * faces.size();

    Geometry geometry;
    geometry.vertices = vsg::vec3Array::create(num_vertices);
    geometry.normals = vsg::vec3Array::create(num_vertices);
    auto indices = vsg::uintArray::create(num_vertices);

    for (size_t f = 0; f < faces.size(); f++) {
        const auto& v0 = vertices[faces[f][0]];
        const auto& v1 = vertices[faces[f][1]];
        const auto& v2 = vertices[faces[f][2]];
        auto n = Vcross(v1 - v0, v2 - v0).GetNormalized();
        for (int k = 0; k < 3; k++) {
            const auto& v = vertices[faces[f][k]];
            (*geometry.vertices)[3 * f + k].set(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
            (*geometry.normals)[3 * f + k].set(static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z()));
            (*indices)[3 * f + k] = static_cast<uint32_t>(3 * f + k);
        }
    }
    geometry.indices = indices;

    return geometry;
}

bool ChInstancedShapesVSG::GetShapes(const ChBody& body, std::vector<ShapeInfo>& shapes) const {
    int num_divs = m_num_divs;

    // Unit primitives: the template is created once per kind
    auto unit = [&](int kind, const ChFramed& frame, const ChVector3d& scale) {
        ShapeInfo info;
        info.kind = kind;
        info.key = nullptr;
        info.frame = frame;
        info.scale.set(static_cast<float>(scale.x()), static_cast<float>(scale.y()), static_cast<float>(scale.z()));
        switch (kind) {
            case SPHERE:
                info.create = [num_divs]() {
                    ShapeBuilder::SphereShapeData data(num_divs);
                    return Geometry{data.vertices, data.normals, data.indices};
                };
                break;
            case BOX:
                info.create = []() {
                    ShapeBuilder::BoxShapeData data;
                    return Geometry{data.vertices, data.normals, data.indices};
                };
                break;
            case CYLINDER:
                info.create = [num_divs]() {
                    ShapeBuilder::CylinderShapeData data(num_divs);
                    return Geometry{data.vertices, data.normals, data.indices};
                };
                break;
            case CONE:
                info.create = [num_divs]() {
                    ShapeBuilder::ConeShapeData data(num_divs);
                    return Geometry{data.vertices, data.normals, data.indices};
                };
                break;
        }
        shapes.push_back(info);
    };

    // Distinct geometries: the template is created once per set of dimensions (or once per mesh)
    auto distinct = [&](int kind, const ChFramed& frame, const std::vector<double>& params, const void* key, std::function<Geometry()> create,
                        const ChVector3d& scale = ChVector3d(1)) {
        ShapeInfo info;
        info.kind = kind;
        info.params = params;
        info.key = key;
        info.create = create;
        info.frame = frame;
        info.scale.set(static_cast<float>(scale.x()), static_cast<float>(scale.y()), static_cast<float>(scale.z()));
        shapes.push_back(info);
    };

    // A capsule of radius r and length h (between the centers of the end caps) is a rounded cylinder
    auto capsule = [num_divs](float r, float h) {
        return [num_divs, r, h]() {
            ShapeBuilder::RoundedCylinderShapeData data(r, h + 2 * r, r, num_divs);
            return Geometry{data.vertices, data.normals, data.indices};
        };
    };
    auto rounded_box = [num_divs](const ChVector3d& lengths, float sradius) {
        return [num_divs, lengths, sradius]() {
            ShapeBuilder::RoundedBoxShapeData data(vsg::vec3CH(lengths), sradius, num_divs);
            return Geometry{data.vertices, data.normals, data.indices};
        };
    };
    auto rounded_cylinder = [num_divs](float radius, float height, float sradius) {
        return [num_divs, radius, height, sradius]() {
            ShapeBuilder::RoundedCylinderShapeData data(radius, height, sradius, num_divs);
            return Geometry{data.vertices, data.normals, data.indices};
        };
    };

    if (m_source == ShapeSource::COLLISION) {
        auto model = body.GetCollisionModel();
        if (!model || model->GetNumShapes() == 0)
            return false;

        for (const auto& si : model->GetShapeInstances()) {
            const auto& shape = si.shape;
            const auto& frame = si.frame;
            switch (shape->GetType()) {
                case ChCollisionShape::Type::SPHERE: {
                    double r = std::static_pointer_cast<ChCollisionShapeSphere>(shape)->GetRadius();
                    unit(SPHERE, frame, ChVector3d(r));
                    break;
                }
                case ChCollisionShape::Type::ELLIPSOID:
                    unit(SPHERE, frame, std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape)->GetSemiaxes());
                    break;
                case ChCollisionShape::Type::BOX:
                    unit(BOX, frame, std::static_pointer_cast<ChCollisionShapeBox>(shape)->GetHalflengths());
                    break;
                case ChCollisionShape::Type::CYLINDER: {
                    auto cyl = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                    unit(CYLINDER, frame, ChVector3d(cyl->GetRadius(), cyl->GetRadius(), cyl->GetHeight()));
                    break;
                }
                case ChCollisionShape::Type::CONE: {
                    auto cone = std::static_pointer_cast<ChCollisionShapeCone>(shape);
                    unit(CONE, frame, ChVector3d(cone->GetRadius(), cone->GetRadius(), cone->GetHeight()));
                    break;
                }
                case ChCollisionShape::Type::CAPSULE: {
                    auto cap = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                    float r = static_cast<float>(cap->GetRadius());
                    float h = static_cast<float>(cap->GetHeight());
                    distinct(CAPSULE, frame, {r, h}, nullptr, capsule(r, h));
                    break;
                }
                case ChCollisionShape::Type::ROUNDEDBOX: {
                    auto rbox = std::static_pointer_cast<ChCollisionShapeRoundedBox>(shape);
                    auto lengths = rbox->GetLengths();
                    float sradius = static_cast<float>(rbox->GetSweepRadius());
                    distinct(ROUNDED_BOX, frame, {lengths.x(), lengths.y(), lengths.z(), sradius}, nullptr, rounded_box(lengths, sradius));
                    break;
                }
                case ChCollisionShape::Type::ROUNDEDCYL: {
                    auto rcyl = std::static_pointer_cast<ChCollisionShapeRoundedCylinder>(shape);
                    float radius = static_cast<float>(rcyl->GetRadius());
                    float height = static_cast<float>(rcyl->GetHeight());
                    float sradius = static_cast<float>(rcyl->GetSweepRadius());
                    distinct(ROUNDED_CYLINDER, frame, {radius, height, sradius}, nullptr, rounded_cylinder(radius, height, sradius));
                    break;
                }
                case ChCollisionShape::Type::TRIANGLEMESH: {
                    auto trimesh = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(shape);
                    auto mesh = std::dynamic_pointer_cast<ChTriangleMeshConnected>(trimesh->GetMesh());
                    if (!mesh || trimesh->IsMutable())
                        return false;
                    distinct(MESH, frame, {}, mesh.get(), [mesh]() { return CreateMeshGeometry(*mesh); });
                    break;
                }
                case ChCollisionShape::Type::CONVEXHULL: {
                    auto hull = std::static_pointer_cast<ChCollisionShapeConvexHull>(shape);
                    if (hull->IsMutable())
                        return false;
                    distinct(MESH, frame, {}, hull.get(), [hull]() {
                        ChTriangleMeshConnected mesh;
                        bt_utils::ChConvexHullLibraryWrapper::ComputeHull(hull->GetPoints(), mesh);
                        return CreateMeshGeometry(mesh);
                    });
                    break;
                }
                default:
                    return false;
            }
        }
    } else {
        auto model = body.GetVisualModel();
        if (!model || model->GetNumShapes() == 0)
            return false;

        for (const auto& si : model->GetShapeInstances()) {
            const auto& shape = si.shape;
            const auto& frame = si.frame;
            if (!shape->IsVisible())
                continue;
            switch (shape->GetType()) {
                case ChVisualShape::Type::SPHERE: {
                    double r = std::static_pointer_cast<ChVisualShapeSphere>(shape)->GetRadius();
                    unit(SPHERE, frame, ChVector3d(r));
                    break;
                }
                case ChVisualShape::Type::ELLIPSOID:
                    unit(SPHERE, frame, std::static_pointer_cast<ChVisualShapeEllipsoid>(shape)->GetSemiaxes());
                    break;
                case ChVisualShape::Type::BOX:
                    unit(BOX, frame, std::static_pointer_cast<ChVisualShapeBox>(shape)->GetHalflengths());
                    break;
                case ChVisualShape::Type::CYLINDER: {
                    auto cyl = std::static_pointer_cast<ChVisualShapeCylinder>(shape);
                    unit(CYLINDER, frame, ChVector3d(cyl->GetRadius(), cyl->GetRadius(), cyl->GetHeight()));
                    break;
                }
                case ChVisualShape::Type::CONE: {
                    auto cone = std::static_pointer_cast<ChVisualShapeCone>(shape);
                    unit(CONE, frame, ChVector3d(cone->GetRadius(), cone->GetRadius(), cone->GetHeight()));
                    break;
                }
                case ChVisualShape::Type::CAPSULE: {
                    auto cap = std::static_pointer_cast<ChVisualShapeCapsule>(shape);
                    float r = static_cast<float>(cap->GetRadius());
                    float h = static_cast<float>(cap->GetHeight());
                    distinct(CAPSULE, frame, {r, h}, nullptr, capsule(r, h));
                    break;
                }
                case ChVisualShape::Type::ROUNDEDBOX: {
                    auto rbox = std::static_pointer_cast<ChVisualShapeRoundedBox>(shape);
                    auto lengths = rbox->GetLengths();
                    float sradius = static_cast<float>(rbox->GetSphereRadius());
                    distinct(ROUNDED_BOX, frame, {lengths.x(), lengths.y(), lengths.z(), sradius}, nullptr, rounded_box(lengths, sradius));
                    break;
                }
                case ChVisualShape::Type::ROUNDEDCYL: {
                    auto rcyl = std::static_pointer_cast<ChVisualShapeRoundedCylinder>(shape);
                    float radius = static_cast<float>(rcyl->GetRadius());
                    float height = static_cast<float>(rcyl->GetHeight());
                    float sradius = static_cast<float>(rcyl->GetSphereRadius());
                    distinct(ROUNDED_CYLINDER, frame, {radius, height, sradius}, nullptr, rounded_cylinder(radius, height, sradius));
                    break;
                }
                case ChVisualShape::Type::TRIANGLEMESH: {
                    auto trimesh = std::static_pointer_cast<ChVisualShapeTriangleMesh>(shape);
                    auto mesh = trimesh->GetMesh();
                    if (!mesh || trimesh->IsMutable())
                        return false;
                    distinct(MESH, frame, {}, mesh.get(), [mesh]() { return CreateMeshGeometry(*mesh); }, trimesh->GetScale());
                    break;
                }
                default:
                    return false;
            }
        }
    }

    return !shapes.empty();
}

// -----------------------------------------------------------------------------

void ChInstancedShapesVSG::Initialize(ChVisualSystemVSG& vsys, const std::vector<std::shared_ptr<ChBody>>& candidates) {
    if (m_root)
        return;

    if (vsys.IsInitialized()) {
        std::cerr << "ChInstancedShapesVSG: the visual system is already initialized; the instanced bodies cannot be added." << std::endl;
        return;
    }

    // Decompose the candidate bodies into shapes and find their sizes
    struct Candidate {
        std::shared_ptr<ChBody> body;
        std::vector<ShapeInfo> shapes;
        double size;
    };
    std::vector<Candidate> instanceable;
    for (const auto& body : candidates) {
        if (!body)
            continue;
        Candidate c;
        c.body = body;
        if (!GetShapes(*body, c.shapes))
            continue;
        c.size = GetBodySize(*body, m_source);
        instanceable.push_back(std::move(c));
    }

    // Median size of the bodies active at initialization (of all instanceable bodies, if none is active)
    std::vector<double> sizes;
    for (const auto& c : instanceable) {
        if (c.body->IsActive())
            sizes.push_back(c.size);
    }
    if (sizes.empty()) {
        for (const auto& c : instanceable)
            sizes.push_back(c.size);
    }
    if (!sizes.empty()) {
        auto mid = sizes.begin() + sizes.size() / 2;
        std::nth_element(sizes.begin(), mid, sizes.end());
        m_median_size = *mid;
    }

    // Select the bodies drawn as instances and assign their shapes to templates
    std::unordered_set<const ChBody*> selection;
    for (const auto& body : m_selection)
        selection.insert(body.get());
    double size_limit = m_size_factor * m_median_size;

    std::map<std::tuple<int, std::vector<double>, const void*>, size_t> template_map;
    std::vector<Geometry> geometries;

    for (auto& c : instanceable) {
        bool selected;
        if (!m_selection.empty())
            selected = selection.count(c.body.get()) > 0;
        else if (m_selector)
            selected = m_selector(*c.body);
        else
            selected = c.body->IsActive() && c.size <= size_limit;
        if (!selected)
            continue;

        auto index = static_cast<unsigned int>(m_bodies.size());
        m_bodies.push_back(c.body);
        m_body_set.insert(c.body.get());

        for (auto& shape : c.shapes) {
            auto key = std::make_tuple(shape.kind, shape.params, shape.key);
            auto it = template_map.find(key);
            size_t t;
            if (it == template_map.end()) {
                t = m_templates.size();
                template_map[key] = t;
                m_templates.push_back(Template());
                m_templates[t].double_faced = (shape.kind == MESH);  // meshes may not be consistently oriented
                geometries.push_back(shape.create());
            } else {
                t = it->second;
            }
            bool identity = shape.frame.GetPos() == VNULL && shape.frame.GetRot() == QUNIT;
            m_templates[t].instances.push_back({index, shape.frame, identity, shape.scale});
        }
    }

    if (m_selection.size() > m_bodies.size()) {
        std::cerr << "ChInstancedShapesVSG: " << m_selection.size() - m_bodies.size()
                  << " of the selected bodies cannot be drawn as instances (unsupported shapes, or not among the candidates)." << std::endl;
    }

    // Create the instance nodes (one per template, with capacity for all its instances)
    auto options = vsys.GetOptions();
    auto group = vsg::Group::create();
    m_num_instances = 0;
    for (size_t t = 0; t < m_templates.size(); t++) {
        auto& tmpl = m_templates[t];
        const auto& geometry = geometries[t];
        size_t size = tmpl.instances.size();
        m_num_instances += static_cast<unsigned int>(size);

        tmpl.translations = vsg::vec3Array::create(size);
        tmpl.rotations = vsg::quatArray::create(size);
        tmpl.scales = vsg::vec3Array::create(size);
        tmpl.colors = vsg::vec4Array::create(size);
        tmpl.translations->properties.dataVariance = vsg::DYNAMIC_DATA;
        tmpl.rotations->properties.dataVariance = vsg::DYNAMIC_DATA;
        tmpl.scales->properties.dataVariance = vsg::DYNAMIC_DATA;
        tmpl.colors->properties.dataVariance = vsg::DYNAMIC_DATA;

        auto draw = vsg::InstanceDrawIndexed::create();
        draw->assignArrays(vsg::DataList{geometry.vertices, geometry.normals});
        draw->assignIndices(geometry.indices);
        draw->indexCount = static_cast<uint32_t>(geometry.indices->valueCount());

        auto state = createPbrInstancedStateGroup(options, m_material, tmpl.double_faced);
        state->addChild(draw);

        tmpl.node = vsg::InstanceNode::create();
        tmpl.node->translations = vsg::BufferInfo::create(tmpl.translations);
        tmpl.node->rotations = vsg::BufferInfo::create(tmpl.rotations);
        tmpl.node->scales = vsg::BufferInfo::create(tmpl.scales);
        tmpl.node->colors = vsg::BufferInfo::create(tmpl.colors);
        tmpl.node->instanceCount = 0;
        tmpl.node->child = state;

        group->addChild(tmpl.node);
    }

    m_root = vsg::Switch::create();
    m_root->addChild(m_visible, group);
    vsys.GetVSGScene()->addChild(m_root);

    // Exclude the selected bodies from the default binding of the visual system
    for (const auto& body : m_bodies)
        vsys.ExcludeFromBinding(body);

    // Default body colors, from the first material in the body visual model
    m_body_colors.resize(m_bodies.size());
    for (size_t i = 0; i < m_bodies.size(); i++) {
        ChColor color = m_default_color;
        if (auto model = m_bodies[i]->GetVisualModel()) {
            for (const auto& si : model->GetShapeInstances()) {
                if (si.shape->GetNumMaterials() > 0) {
                    color = si.shape->GetMaterial(0)->GetDiffuseColor();
                    break;
                }
            }
        }
        m_body_colors[i] = vsg::vec4(color.R, color.G, color.B, 1.0f);
    }

    m_body_visible.resize(m_bodies.size());
    m_color_work.resize(m_bodies.size());
}

void ChInstancedShapesVSG::Update() {
    m_num_visible = 0;
    if (!m_root || !m_visible)
        return;

    // Visibility and color of each body
    for (size_t i = 0; i < m_bodies.size(); i++) {
        const auto& body = *m_bodies[i];
        bool visible = (m_show_inactive || body.IsActive()) && (!m_visibility_callback || m_visibility_callback->IsVisible(body));
        m_body_visible[i] = visible;
        if (!visible)
            continue;
        if (m_color_callback) {
            ChColor c = m_color_callback->GetColor(body);
            m_color_work[i] = vsg::vec4(c.R, c.G, c.B, 1.0f);
        } else {
            m_color_work[i] = m_body_colors[i];
        }
    }

    // Instances of visible bodies, packed at the start of the instance arrays of each template
    for (auto& tmpl : m_templates) {
        uint32_t k = 0;
        for (const auto& instance : tmpl.instances) {
            if (!m_body_visible[instance.body])
                continue;
            const auto& X_body = m_bodies[instance.body]->GetFrameRefToAbs();
            ChVector3d pos;
            ChQuaterniond rot;
            if (instance.identity) {
                pos = X_body.GetPos();
                rot = X_body.GetRot();
            } else {
                pos = X_body.TransformPointLocalToParent(instance.frame.GetPos());
                rot = X_body.GetRot() * instance.frame.GetRot();
            }
            (*tmpl.translations)[k].set(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z()));
            (*tmpl.rotations)[k].set(static_cast<float>(rot.e1()), static_cast<float>(rot.e2()), static_cast<float>(rot.e3()), static_cast<float>(rot.e0()));
            (*tmpl.scales)[k] = instance.scale;
            (*tmpl.colors)[k] = m_color_work[instance.body];
            k++;
        }

        tmpl.node->instanceCount = k;
        tmpl.translations->dirty();
        tmpl.rotations->dirty();
        tmpl.scales->dirty();
        tmpl.colors->dirty();
        m_num_visible += k;
    }
}

}  // namespace vsg3d
}  // namespace chrono
