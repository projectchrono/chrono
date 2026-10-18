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
// Authors: Kyle Sha
// =============================================================================
//
// Bridge from a Chrono ChSystem to a MetalRenderScene. See ChMetalSceneBuilder.h.
//
// =============================================================================

#include "chrono_sensor/metal/ChMetalSceneBuilder.h"
#include <fstream>
#include <cstdio>
#include <cmath>
#include <utility>
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
using namespace chrono;
namespace chrono {
namespace sensor {

// Shading parameters for a shape that carries no ChVisualMaterial of its own.
//
// Read from ChVisualMaterial::Default() rather than written out as literals, so this backend
// cannot drift from OptiX and Vulkan RT when that canonical default changes. OptiX shades a
// material-less shape with exactly this material (ChOptixPipeline), and Vulkan RT derives its
// implicit material the same way (ChVulkanRTScene).
struct DefaultMaterial {
    float kd[3], ks[3], ke[3];
    float opacity, roughness, metallic, emissive_power, use_specular_workflow;
    float tex_scale[2];
};
static const DefaultMaterial& defaultMaterial() {
    static const DefaultMaterial d = [] {
        const auto m = ChVisualMaterial::Default();
        const auto kd = m->GetDiffuseColor();
        const auto ks = m->GetSpecularColor();
        const auto ke = m->GetEmissiveColor();
        const auto ts = m->GetTextureScale();
        DefaultMaterial out{};
        out.kd[0] = kd.R;
        out.kd[1] = kd.G;
        out.kd[2] = kd.B;
        out.ks[0] = ks.R;
        out.ks[1] = ks.G;
        out.ks[2] = ks.B;
        out.ke[0] = ke.R;
        out.ke[1] = ke.G;
        out.ke[2] = ke.B;
        out.opacity = m->GetOpacity();
        out.roughness = m->GetRoughness();
        out.metallic = m->GetMetallic();
        out.emissive_power = m->GetEmissivePower();
        out.use_specular_workflow = m->GetUseSpecularWorkflow() ? 1.f : 0.f;
        out.tex_scale[0] = (float)ts.x();
        out.tex_scale[1] = (float)ts.y();
        return out;
    }();
    return d;
}

struct V3 {
    double x, y, z;
};
static V3 cross3(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static V3 sub3(V3 a, V3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
static double dot3(V3 a, V3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static V3 nrm3(V3 a) {
    double l = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (l > 0) {
        a.x /= l;
        a.y /= l;
        a.z /= l;
    }
    return a;
}
// orthonormalize tangent t against normal n (Gram-Schmidt); fall back to +X if degenerate
static V3 gramSchmidt(V3 t, V3 n) {
    double d = dot3(n, t);
    V3 r = {t.x - n.x * d, t.y - n.y * d, t.z - n.z * d};
    r = nrm3(r);
    if (r.x == 0 && r.y == 0 && r.z == 0)
        r = V3{1, 0, 0};
    return r;
}

// tan9 (optional): 3 precomputed per-corner tangents (9 floats). If null, a flat per-face tangent is used.
static void addTri(MetalGeometry& g, V3 a, V3 b, V3 c, V3 na, V3 nb, V3 nc, float* col, float* uvs, float* tan9 = nullptr) {
    g.verts.insert(g.verts.end(), {(float)a.x, (float)a.y, (float)a.z, (float)b.x, (float)b.y, (float)b.z, (float)c.x, (float)c.y, (float)c.z});
    g.normals.insert(g.normals.end(), {(float)na.x, (float)na.y, (float)na.z, (float)nb.x, (float)nb.y, (float)nb.z, (float)nc.x, (float)nc.y, (float)nc.z});
    g.colors.insert(g.colors.end(), {col[0], col[1], col[2]});
    if (uvs)
        g.uv.insert(g.uv.end(), {uvs[0], uvs[1], uvs[2], uvs[3], uvs[4], uvs[5]});
    else
        g.uv.insert(g.uv.end(), {0, 0, 0, 0, 0, 0});
    if (tan9) {
        g.tangents.insert(g.tangents.end(), tan9, tan9 + 9);
        return;
    }  // smooth per-vertex tangents (from meshToGeom)
    // fallback: flat per-face tangent from edges + UV deltas (primitives / no smoothing); default +X if degenerate
    V3 T{1, 0, 0};
    if (uvs) {
        V3 e1 = sub3(b, a), e2 = sub3(c, a);
        double du1 = uvs[2] - uvs[0], dv1 = uvs[3] - uvs[1], du2 = uvs[4] - uvs[0], dv2 = uvs[5] - uvs[1];
        double det = du1 * dv2 - du2 * dv1;
        if (std::fabs(det) > 1e-12) {
            double f = 1.0 / det;
            T = nrm3({f * (dv2 * e1.x - dv1 * e2.x), f * (dv2 * e1.y - dv1 * e2.y), f * (dv2 * e1.z - dv1 * e2.z)});
        }
    }
    g.tangents.insert(g.tangents.end(), {(float)T.x, (float)T.y, (float)T.z, (float)T.x, (float)T.y, (float)T.z, (float)T.x, (float)T.y, (float)T.z});
}

// fill a MetalGeometry (verts/normals/uv/colors) from a Chrono mesh; also emit per-face material index
static void meshToGeom(std::shared_ptr<ChTriangleMeshConnected> mesh,
                       MetalGeometry& g,
                       std::vector<int>& faceMat,
                       const std::vector<std::shared_ptr<ChVisualMaterial>>& mats,
                       float defR,
                       float defG,
                       float defB,
                       ChVector3d scale) {
    auto& Vs = mesh->GetCoordsVertices();
    auto& Is = mesh->GetIndicesVertices();
    size_t nv = Vs.size();
    auto& Ns = mesh->GetCoordsNormals();
    auto& INs = mesh->GetIndicesNormals();
    auto& MI = mesh->GetIndicesMaterials();
    auto& UVs = mesh->GetCoordsUV();
    auto& IUV = mesh->GetIndicesUV();
    bool haveN = (!Ns.empty() && INs.size() == Is.size());
    bool haveMI = (MI.size() == Is.size());
    bool haveM = !mats.empty();
    bool haveUV = (!UVs.empty() && IUV.size() == Is.size());
    g = {};
    faceMat.clear();
    // Pass 1: accumulate smooth per-vertex tangents (averaged across faces) so normal mapping is smooth,
    // not faceted. Faceted per-face tangents made the car body look bumpy near the windows.
    std::vector<V3> accTan(nv, V3{0, 0, 0});
    if (haveUV) {
        for (size_t ti = 0; ti < Is.size(); ++ti) {
            auto& f = Is[ti];
            int i0 = f.x(), i1 = f.y(), i2 = f.z();
            if (i0 < 0 || i1 < 0 || i2 < 0 || (size_t)i0 >= nv || (size_t)i1 >= nv || (size_t)i2 >= nv)
                continue;
            auto& fu = IUV[ti];
            size_t nu = UVs.size();
            int u0 = fu.x(), u1 = fu.y(), u2 = fu.z();
            if (!(u0 >= 0 && u1 >= 0 && u2 >= 0 && (size_t)u0 < nu && (size_t)u1 < nu && (size_t)u2 < nu))
                continue;
            V3 A{Vs[i0].x() * scale.x(), Vs[i0].y() * scale.y(), Vs[i0].z() * scale.z()};
            V3 B{Vs[i1].x() * scale.x(), Vs[i1].y() * scale.y(), Vs[i1].z() * scale.z()};
            V3 C{Vs[i2].x() * scale.x(), Vs[i2].y() * scale.y(), Vs[i2].z() * scale.z()};
            V3 e1 = sub3(B, A), e2 = sub3(C, A);
            double du1 = UVs[u1].x() - UVs[u0].x(), dv1 = UVs[u1].y() - UVs[u0].y(), du2 = UVs[u2].x() - UVs[u0].x(), dv2 = UVs[u2].y() - UVs[u0].y();
            double det = du1 * dv2 - du2 * dv1;
            if (std::fabs(det) < 1e-12)
                continue;
            double fdet = 1.0 / det;
            V3 T{fdet * (dv2 * e1.x - dv1 * e2.x), fdet * (dv2 * e1.y - dv1 * e2.y), fdet * (dv2 * e1.z - dv1 * e2.z)};
            for (int idx : {i0, i1, i2}) {
                accTan[idx].x += T.x;
                accTan[idx].y += T.y;
                accTan[idx].z += T.z;
            }
        }
    }
    for (size_t ti = 0; ti < Is.size(); ++ti) {
        auto& f = Is[ti];
        int i0 = f.x(), i1 = f.y(), i2 = f.z();
        if (i0 < 0 || i1 < 0 || i2 < 0 || (size_t)i0 >= nv || (size_t)i1 >= nv || (size_t)i2 >= nv)
            continue;
        V3 A{Vs[i0].x() * scale.x(), Vs[i0].y() * scale.y(), Vs[i0].z() * scale.z()};
        V3 B{Vs[i1].x() * scale.x(), Vs[i1].y() * scale.y(), Vs[i1].z() * scale.z()};
        V3 C{Vs[i2].x() * scale.x(), Vs[i2].y() * scale.y(), Vs[i2].z() * scale.z()};
        V3 nA, nB, nC;
        if (haveN) {
            auto& fn = INs[ti];
            size_t nn = Ns.size();
            int j0 = fn.x(), j1 = fn.y(), j2 = fn.z();
            if (j0 >= 0 && j1 >= 0 && j2 >= 0 && (size_t)j0 < nn && (size_t)j1 < nn && (size_t)j2 < nn) {
                ChVector3d is(1.0 / scale.x(), 1.0 / scale.y(), 1.0 / scale.z());  // inverse-transpose for non-uniform scale
                nA = nrm3({Ns[j0].x() * is.x(), Ns[j0].y() * is.y(), Ns[j0].z() * is.z()});
                nB = nrm3({Ns[j1].x() * is.x(), Ns[j1].y() * is.y(), Ns[j1].z() * is.z()});
                nC = nrm3({Ns[j2].x() * is.x(), Ns[j2].y() * is.y(), Ns[j2].z() * is.z()});
            } else {
                nA = nrm3(cross3(sub3(B, A), sub3(C, A)));
                nB = nA;
                nC = nA;
            }
        } else {
            nA = nrm3(cross3(sub3(B, A), sub3(C, A)));
            nB = nA;
            nC = nA;
        }
        int mi = haveMI ? MI[ti] : 0;
        float col[3] = {defR, defG, defB};
        if (haveM && mi >= 0 && mi < (int)mats.size()) {
            auto kd = mats[mi]->GetDiffuseColor();
            col[0] = kd.R;
            col[1] = kd.G;
            col[2] = kd.B;
        }
        float uvs[6] = {0, 0, 0, 0, 0, 0};
        if (haveUV) {
            auto& fu = IUV[ti];
            size_t nu = UVs.size();
            int u0 = fu.x(), u1 = fu.y(), u2 = fu.z();
            if (u0 >= 0 && u1 >= 0 && u2 >= 0 && (size_t)u0 < nu && (size_t)u1 < nu && (size_t)u2 < nu) {
                uvs[0] = UVs[u0].x();
                uvs[1] = UVs[u0].y();
                uvs[2] = UVs[u1].x();
                uvs[3] = UVs[u1].y();
                uvs[4] = UVs[u2].x();
                uvs[5] = UVs[u2].y();
            }
        }
        float tan9[9];
        float* tptr = nullptr;
        if (haveUV) {
            V3 g0 = gramSchmidt(accTan[i0], nA), g1 = gramSchmidt(accTan[i1], nB), g2 = gramSchmidt(accTan[i2], nC);
            tan9[0] = g0.x;
            tan9[1] = g0.y;
            tan9[2] = g0.z;
            tan9[3] = g1.x;
            tan9[4] = g1.y;
            tan9[5] = g1.z;
            tan9[6] = g2.x;
            tan9[7] = g2.y;
            tan9[8] = g2.z;
            tptr = tan9;
        }
        addTri(g, A, B, C, nA, nB, nC, col, uvs, tptr);
        faceMat.push_back(haveM ? mi : -1);
    }
}
// primitive generators (object space; white base color; no UV)
static void genBox(double lx, double ly, double lz, MetalGeometry& g) {
    g = {};
    double x = lx * .5, y = ly * .5, z = lz * .5;
    float w[3] = {1, 1, 1};
    // Per-vertex UVs computed exactly as the OptiX procedural box does (box.cu), rather than a
    // generic 0..1 quad layout. The two agree on +X, -Y and +Z but are mirrored in BOTH axes -- a 180
    // degree rotation -- on -X, +Y and -Z, because box.cu carries the sign of the face normal into V.
    // The rotation corrupts every map on those faces at once (albedo, normal, roughness, metallic,
    // opacity) and, because addTri derives the face tangent from the UV deltas, it flipped the
    // tangent as well; matching the UVs fixes the tangent with it.
    auto uvOf = [&](V3 v, V3 nn) {
        double pnx = (x > 0) ? v.x / (2 * x) : 0.0;
        double pny = (y > 0) ? v.y / (2 * y) : 0.0;
        double pnz = (z > 0) ? v.z / (2 * z) : 0.0;
        if (std::abs(nn.x) > 0.5)
            return std::pair<float, float>{(float)(pny + 0.5), (float)((pnz + 0.5) * nn.x)};
        if (std::abs(nn.y) > 0.5)
            return std::pair<float, float>{(float)(pnx + 0.5), (float)(-(pnz + 0.5) * nn.y)};
        return std::pair<float, float>{(float)(pnx + 0.5), (float)((pny + 0.5) * nn.z)};
    };
    auto q = [&](V3 a, V3 b, V3 c, V3 d, V3 nn) {
        auto ua = uvOf(a, nn), ub = uvOf(b, nn), uc = uvOf(c, nn), ud = uvOf(d, nn);
        float u1[6] = {ua.first, ua.second, ub.first, ub.second, uc.first, uc.second};
        float u2[6] = {ua.first, ua.second, uc.first, uc.second, ud.first, ud.second};
        addTri(g, a, b, c, nn, nn, nn, w, u1);
        addTri(g, a, c, d, nn, nn, nn, w, u2);
    };
    q({-x, -y, z}, {x, -y, z}, {x, y, z}, {-x, y, z}, {0, 0, 1});
    q({x, -y, -z}, {-x, -y, -z}, {-x, y, -z}, {x, y, -z}, {0, 0, -1});
    q({-x, -y, -z}, {x, -y, -z}, {x, -y, z}, {-x, -y, z}, {0, -1, 0});
    q({x, y, -z}, {-x, y, -z}, {-x, y, z}, {x, y, z}, {0, 1, 0});
    q({x, -y, -z}, {x, y, -z}, {x, y, z}, {x, -y, z}, {1, 0, 0});
    q({-x, y, -z}, {-x, -y, -z}, {-x, -y, z}, {-x, y, z}, {-1, 0, 0});
}
static void genSphere(double sx, double sy, double sz, int st, int sl, MetalGeometry& g) {
    g = {};
    float w[3] = {1, 1, 1};
    auto P = [&](int i, int j) {
        double th = M_PI * i / st, ph = 2 * M_PI * j / sl;
        return V3{sin(th) * cos(ph), sin(th) * sin(ph), cos(th)};
    };
    auto vN = [&](V3 d) { return nrm3(V3{d.x / sx, d.y / sy, d.z / sz}); };
    auto sc = [&](V3 d) { return V3{d.x * sx, d.y * sy, d.z * sz}; };
    for (int i = 0; i < st; i++)
        for (int j = 0; j < sl; j++) {
            V3 a = P(i, j), b = P(i, j + 1), c = P(i + 1, j), d = P(i + 1, j + 1);
            float uj = (float)j / sl, uj1 = (float)(j + 1) / sl, vi = (float)i / st, vi1 = (float)(i + 1) / st;  // equirectangular UVs
            float t1[6] = {uj, vi, uj1, vi, uj, vi1}, t2[6] = {uj1, vi, uj1, vi1, uj, vi1};
            addTri(g, sc(a), sc(b), sc(c), vN(a), vN(b), vN(c), w, t1);
            addTri(g, sc(b), sc(d), sc(c), vN(b), vN(d), vN(c), w, t2);
        }
}
static void genCyl(double r, double h, int seg, MetalGeometry& g) {
    g = {};
    double hz = h * .5;
    float w[3] = {1, 1, 1};
    for (int i = 0; i < seg; i++) {
        double a0 = 2 * M_PI * i / seg, a1 = 2 * M_PI * (i + 1) / seg;
        V3 d0{cos(a0), sin(a0), 0}, d1{cos(a1), sin(a1), 0};
        V3 p00{d0.x * r, d0.y * r, -hz}, p01{d1.x * r, d1.y * r, -hz}, p10{d0.x * r, d0.y * r, hz}, p11{d1.x * r, d1.y * r, hz};
        float su0 = (float)i / seg, su1 = (float)(i + 1) / seg;  // side: u=angle, v=height
        float s1[6] = {su0, 0, su1, 0, su0, 1}, s2[6] = {su1, 0, su1, 1, su0, 1};
        addTri(g, p00, p01, p10, d0, d1, d0, w, s1);
        addTri(g, p01, p11, p10, d1, d1, d0, w, s2);
        float ct[6] = {0.5f, 0.5f, (float)(d0.x * .5 + .5), (float)(d0.y * .5 + .5), (float)(d1.x * .5 + .5), (float)(d1.y * .5 + .5)};  // caps: planar
        float cb[6] = {0.5f, 0.5f, (float)(d1.x * .5 + .5), (float)(d1.y * .5 + .5), (float)(d0.x * .5 + .5), (float)(d0.y * .5 + .5)};
        addTri(g, {0, 0, hz}, p10, p11, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, w, ct);
        addTri(g, {0, 0, -hz}, p01, p00, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, w, cb);
    }
}
static void genCone(double r, double h, int seg, MetalGeometry& g) {
    g = {};
    double hz = h * .5;
    float w[3] = {1, 1, 1};
    V3 ap{0, 0, hz};
    for (int i = 0; i < seg; i++) {
        double a0 = 2 * M_PI * i / seg, a1 = 2 * M_PI * (i + 1) / seg;
        V3 b0{r * cos(a0), r * sin(a0), -hz}, b1{r * cos(a1), r * sin(a1), -hz};
        V3 n0 = nrm3({cos(a0), sin(a0), r / h}), n1 = nrm3({cos(a1), sin(a1), r / h});
        float u0 = (float)i / seg, u1 = (float)(i + 1) / seg;
        float side[6] = {u0, 0, u1, 0, 0.5f, 1};  // side: u=angle, apex at v=1
        addTri(g, b0, b1, ap, n0, n1, nrm3({(n0.x + n1.x) / 2, (n0.y + n1.y) / 2, (n0.z + n1.z) / 2}), w, side);
        float cb[6] = {0.5f, 0.5f, (float)(cos(a1) * .5 + .5), (float)(sin(a1) * .5 + .5), (float)(cos(a0) * .5 + .5), (float)(sin(a0) * .5 + .5)};  // base cap: planar
        addTri(g, {0, 0, -hz}, b1, b0, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, w, cb);
    }
}
static void genCapsule(double r, double h, int seg, MetalGeometry& g) {
    genCyl(r, h, seg, g);
    double hz = h * .5;
    float w[3] = {1, 1, 1};
    int rings = 6;
    auto hemi = [&](double zc, double sgn) {
        for (int i = 0; i < rings; i++)
            for (int j = 0; j < seg; j++) {
                double t0 = (M_PI / 2) * i / rings, t1 = (M_PI / 2) * (i + 1) / rings, p0 = 2 * M_PI * j / seg, p1 = 2 * M_PI * (j + 1) / seg;
                auto pt = [&](double t, double p) {
                    V3 d{sin(t) * cos(p), sin(t) * sin(p), sgn * cos(t)};
                    return std::pair<V3, V3>({d.x * r, d.y * r, zc + d.z * r}, d);
                };
                auto A = pt(t0, p0), B = pt(t0, p1), C = pt(t1, p0), D = pt(t1, p1);
                float u0 = (float)j / seg, u1 = (float)(j + 1) / seg, v0 = (float)i / rings, v1 = (float)(i + 1) / rings;
                float h1[6] = {u0, v0, u1, v0, u0, v1}, h2[6] = {u1, v0, u1, v1, u0, v1};
                addTri(g, A.first, B.first, C.first, A.second, B.second, C.second, w, h1);
                addTri(g, B.first, D.first, C.first, B.second, D.second, C.second, w, h2);
            }
    };
    hemi(hz, 1.0);
    hemi(-hz, -1.0);
}

int ChMetalSceneBuilder::CountShapes() const {
    int c = 0;
    for (auto& it : m_sys->GetOtherPhysicsItems()) {
        auto vm = it->GetVisualModel();
        if (vm)
            c += (int)vm->GetShapeInstances().size();
    }
    for (auto& b : m_sys->GetBodies()) {
        auto vm = b->GetVisualModel();
        if (vm)
            c += (int)vm->GetShapeInstances().size();
    }
    return c;
}
bool ChMetalSceneBuilder::TopologyChanged() const {
    return CountShapes() != m_last_shape_count;
}

static void fillFrame(MetalInstance& in, const ChFramed& F) {
    auto R = F.GetRotMat();
    ChVector3d p = F.GetPos();
    in.xform[0] = R(0, 0);
    in.xform[1] = R(1, 0);
    in.xform[2] = R(2, 0);
    in.xform[3] = R(0, 1);
    in.xform[4] = R(1, 1);
    in.xform[5] = R(2, 1);
    in.xform[6] = R(0, 2);
    in.xform[7] = R(1, 2);
    in.xform[8] = R(2, 2);
    in.xform[9] = p.x();
    in.xform[10] = p.y();
    in.xform[11] = p.z();
    for (int k = 0; k < 9; k++)
        in.rot[k] = in.xform[k];
}

void ChMetalSceneBuilder::Build(MetalRenderScene& scene) {
    scene.geometries.clear();
    scene.instances.clear();
    scene.texturePaths.clear();
    m_srcs.clear();
    m_geom_cache.clear();
    std::map<std::string, int> texIdx;
    std::string curBaseDir;              // directory of the current mesh file (resolve relative map_Kd)
    uint32_t curClass = 0, curInst = 0;  // semantic class/instance ids of the current shape
    auto fileOk = [](const std::string& p) {
        std::ifstream f(p);
        return f.good();
    };
    auto texturePathIndex = [&](const std::string& raw) -> int {
        if (raw.empty())
            return -1;
        std::string p = raw;
        if (!fileOk(p)) {
            if (!curBaseDir.empty() && fileOk(curBaseDir + "/" + raw))
                p = curBaseDir + "/" + raw;  // relative to the mesh's own dir
            else if (fileOk(GetChronoDataPath() + raw))
                p = GetChronoDataPath() + raw;  // relative to the Chrono data path
        }
        auto it = texIdx.find(p);
        if (it != texIdx.end())
            return it->second;
        if ((int)scene.texturePaths.size() >= 64)
            return -1;  // shader texs[] array is fixed at 64; refuse beyond it (avoids OOB reads)
        int idx = (int)scene.texturePaths.size();
        scene.texturePaths.push_back(p);
        texIdx[p] = idx;
        return idx;
    };
    std::vector<int> faceMat;
    // Every material property that toTexIds bakes into the geometry below is baked into the SHARED,
    // cached geometry, so all of it has to appear in the cache key. Keying on shape dimensions and the
    // Kd texture alone let two shapes of the same size but different roughness, metallic, specular or
    // opacity share one entry, and the second shape silently rendered with the first one's material --
    // only the per-instance tint escapes the cache, so the objects still had the right colour.
    auto materialKey = [](const std::vector<std::shared_ptr<ChVisualMaterial>>& mats) {
        std::string key;
        char buf[256];
        for (const auto& m : mats) {
            if (!m) {
                key += "|~";
                continue;
            }
            const ChColor& ks = m->GetSpecularColor();
            const ChColor& ke = m->GetEmissiveColor();
            const ChVector2f& ts = m->GetTextureScale();
            snprintf(buf, sizeof(buf), "|%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%d,%g,%g", m->GetOpacity(), m->GetRoughness(), m->GetMetallic(), ks.R, ks.G, ks.B, ke.R, ke.G, ke.B,
                     m->GetEmissivePower(), m->GetUseSpecularWorkflow() ? 1 : 0, ts.x(), ts.y());
            key += buf;
            key += "|" + m->GetKdTexture() + "|" + m->GetRoughnessTexture() + "|" + m->GetMetallicTexture() + "|" + m->GetOpacityTexture() + "|" + m->GetNormalMapTexture() + "|" +
                   m->GetKsTexture() + "|" + m->GetKeTexture() + "|" + m->GetWeightTexture();
        }
        return key;
    };

    auto toTexIds = [&](MetalGeometry& g, std::vector<int>& fm, const std::vector<std::shared_ptr<ChVisualMaterial>>& mats) {
        g.texId.clear();
        g.texId.reserve(fm.size());
        g.opacity.clear();
        g.opacity.reserve(fm.size());
        g.roughness.clear();
        g.roughness.reserve(fm.size());
        g.metallic.clear();
        g.metallic.reserve(fm.size());
        g.roughTexId.clear();
        g.roughTexId.reserve(fm.size());
        g.metalTexId.clear();
        g.metalTexId.reserve(fm.size());
        g.opacityTexId.clear();
        g.opacityTexId.reserve(fm.size());
        g.normalTexId.clear();
        g.normalTexId.reserve(fm.size());
        g.specular.clear();
        g.emissive.clear();
        g.texScale.clear();
        g.ksTexId.clear();
        g.keTexId.clear();
        g.blendKdTexId.clear();
        g.blendWeightTexId.clear();
        // Weight-blended materials (OptiX): if the material list carries weight textures, all faces blend the
        // materials per-pixel by those weights. We do the common 2-way blend: base = material 0, blend layer =
        // material 1 mixed by material 1's weight texture. Trigger matches OptiX (ChOptixPipeline): mats[0]
        // has a weight texture.
        bool blend = mats.size() >= 2 && !mats[0]->GetWeightTexture().empty();
        int bkt = -1, bwt = -1;
        if (blend) {
            bkt = texturePathIndex(mats[1]->GetKdTexture());
            bwt = texturePathIndex(mats[1]->GetWeightTexture());
        }
        const DefaultMaterial& D = defaultMaterial();
        for (int mi : fm) {
            if (blend)
                mi = 0;  // blend mode uses material 0 as the base for every face
            int t = -1, rtx = -1, mtx = -1, otx = -1, ntx = -1, ktx = -1, etx = -1;
            // A face with no material is shaded as ChVisualMaterial::Default(), matching OptiX.
            float op = D.opacity, rg = D.roughness, mt = D.metallic;
            float ks[3] = {D.ks[0], D.ks[1], D.ks[2]}, usp = D.use_specular_workflow;
            float ke[3] = {D.ke[0], D.ke[1], D.ke[2]}, ep = D.emissive_power;
            float ts[2] = {D.tex_scale[0], D.tex_scale[1]};
            if (mi >= 0 && mi < (int)mats.size()) {
                auto& M = *mats[mi];
                t = texturePathIndex(M.GetKdTexture());
                op = M.GetOpacity();
                rg = M.GetRoughness();
                mt = M.GetMetallic();
                rtx = texturePathIndex(M.GetRoughnessTexture());
                mtx = texturePathIndex(M.GetMetallicTexture());
                otx = texturePathIndex(M.GetOpacityTexture());
                ntx = texturePathIndex(M.GetNormalMapTexture());
                ktx = texturePathIndex(M.GetKsTexture());
                etx = texturePathIndex(M.GetKeTexture());
                auto ksc = M.GetSpecularColor();
                ks[0] = ksc.R;
                ks[1] = ksc.G;
                ks[2] = ksc.B;
                usp = M.GetUseSpecularWorkflow() ? 1.f : 0.f;
                auto kec = M.GetEmissiveColor();
                ke[0] = kec.R;
                ke[1] = kec.G;
                ke[2] = kec.B;
                ep = M.GetEmissivePower();
                auto tsc = M.GetTextureScale();
                ts[0] = tsc.x();
                ts[1] = tsc.y();
            }
            g.texId.push_back(t);
            g.opacity.push_back(op);
            g.roughness.push_back(rg);
            g.metallic.push_back(mt);
            g.roughTexId.push_back(rtx);
            g.metalTexId.push_back(mtx);
            g.opacityTexId.push_back(otx);
            g.normalTexId.push_back(ntx);
            g.specular.insert(g.specular.end(), {ks[0], ks[1], ks[2], usp});
            g.emissive.insert(g.emissive.end(), {ke[0], ke[1], ke[2], ep});
            g.texScale.insert(g.texScale.end(), {ts[0], ts[1]});
            g.ksTexId.push_back(ktx);
            g.keTexId.push_back(etx);
            g.blendKdTexId.push_back(bkt);
            g.blendWeightTexId.push_back(bwt);
        }
    };
    auto shapeColor = [&](std::shared_ptr<ChVisualShape> sh, float* out) {
        auto& m = sh->GetMaterials();
        if (!m.empty()) {
            auto kd = m[0]->GetDiffuseColor();
            out[0] = kd.R;
            out[1] = kd.G;
            out[2] = kd.B;
        }
    };

    auto storeSF = [&](InstSrc& s, const ChFramed& f) {
        auto q = f.GetRot();
        auto p = f.GetPos();
        s.sf[0] = q.e0();
        s.sf[1] = q.e1();
        s.sf[2] = q.e2();
        s.sf[3] = q.e3();
        s.sf[4] = p.x();
        s.sf[5] = p.y();
        s.sf[6] = p.z();
    };

    auto addInstance = [&](int geom, ChBody* body, const ChFramed& sf, float* tint, uint32_t mat, bool dynamic, std::shared_ptr<ChTriangleMeshConnected> dynMesh) {
        MetalInstance in;
        in.geom = geom;
        in.mat = mat;
        if (tint) {
            in.tint[0] = tint[0];
            in.tint[1] = tint[1];
            in.tint[2] = tint[2];
        }
        in.classId = curClass;
        in.instanceId = curInst;
        scene.instances.push_back(in);
        InstSrc s;
        s.body = body;
        s.geom = geom;
        s.dynamic = dynamic;
        s.mesh = dynMesh;
        storeSF(s, sf);
        m_srcs.push_back(s);
    };

    auto processItem = [&](ChObj* obj, ChBody* body) {
        auto vm = obj->GetVisualModel();
        if (!vm)
            return;
        // Fallback diffuse for a shape with no material: ChVisualMaterial::Default()'s Kd, which is
        // what OptiX and Vulkan RT shade such a shape with.
        const DefaultMaterial& DM = defaultMaterial();
        float vdef[3] = {DM.kd[0], DM.kd[1], DM.kd[2]};
        for (auto& si : vm->GetShapeInstances()) {
            auto sh = si.shape;
            ChFramed sf = si.frame;
            MetalGeometry g;
            {
                auto& mm = sh->GetMaterials();
                curClass = mm.empty() ? 0u : (uint32_t)mm[0]->GetClassID();
                curInst = mm.empty() ? 0u : (uint32_t)mm[0]->GetInstanceID();
            }
            curBaseDir.clear();
            if (auto tm = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(sh)) {
                auto mesh = tm->GetMesh();
                if (!mesh)
                    continue;
                // Split on IsMutable(), the same test OptiX uses to choose between
                // deformableMeshVisualization and rigidMeshVisualization. A mutable mesh is
                // re-extracted every frame and so cannot be shared through the geometry cache;
                // an immutable one can. Both take the shape's own materials: a mesh does not
                // lose its textures and shading parameters for being deformable.
                if (tm->IsMutable()) {
                    meshToGeom(mesh, g, faceMat, tm->GetMaterials(), vdef[0], vdef[1], vdef[2], ChVector3d(1, 1, 1));
                    if (g.verts.empty())
                        continue;
                    toTexIds(g, faceMat, tm->GetMaterials());
                    g.dynamic = true;
                    int gi = (int)scene.geometries.size();
                    scene.geometries.push_back(std::move(g));
                    float t[3] = {1, 1, 1};
                    addInstance(gi, body, sf, t, 0, true, mesh);
                } else {
                    char kb2[64];
                    snprintf(kb2, 64, "mesh:%p", (void*)mesh.get());
                    std::string k = std::string(kb2) + materialKey(tm->GetMaterials());
                    if (!m_geom_cache.count(k)) {
                        meshToGeom(mesh, g, faceMat, tm->GetMaterials(), vdef[0], vdef[1], vdef[2], ChVector3d(1, 1, 1));
                        if (g.verts.empty())
                            continue;
                        toTexIds(g, faceMat, tm->GetMaterials());
                        m_geom_cache[k] = (int)scene.geometries.size();
                        scene.geometries.push_back(std::move(g));
                    }
                    float t[3] = {1, 1, 1};
                    addInstance(m_geom_cache[k], body, sf, t, 0, false, nullptr);
                }
            } else if (auto mf = std::dynamic_pointer_cast<ChVisualShapeModelFile>(sh)) {
                std::string fn = mf->GetFilename();
                ChVector3d sc = mf->GetScale();
                char kb3[600];
                snprintf(kb3, 600, "file:%s:%g,%g,%g", fn.c_str(), sc.x(), sc.y(), sc.z());
                std::string k = std::string(kb3) + materialKey(mf->GetMaterials());
                {
                    auto sl = fn.find_last_of("/\\");
                    curBaseDir = (sl == std::string::npos) ? std::string() : fn.substr(0, sl);
                }
                if (!m_geom_cache.count(k)) {
                    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(fn, true, true);
                    if (!mesh)
                        continue;
                    float col[3] = {vdef[0], vdef[1], vdef[2]};
                    shapeColor(sh, col);
                    meshToGeom(mesh, g, faceMat, mf->GetMaterials(), col[0], col[1], col[2], sc);
                    if (g.verts.empty())
                        continue;
                    toTexIds(g, faceMat, mf->GetMaterials());
                    m_geom_cache[k] = (int)scene.geometries.size();
                    scene.geometries.push_back(std::move(g));
                }
                float t[3] = {1, 1, 1};
                addInstance(m_geom_cache[k], body, sf, t, 0, false, nullptr);
            } else {
                char kb[96] = {0};
                bool ok = true;
                float col[3] = {vdef[0], vdef[1], vdef[2]};
                shapeColor(sh, col);
                if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeBox>(sh)) {
                    auto L = s2->GetLengths();
                    snprintf(kb, 96, "box:%.4f,%.4f,%.4f", L.x(), L.y(), L.z());
                } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeSphere>(sh)) {
                    double r = s2->GetRadius();
                    snprintf(kb, 96, "sph:%.4f", r);
                } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCylinder>(sh)) {
                    double r = s2->GetRadius(), h = s2->GetHeight();
                    snprintf(kb, 96, "cyl:%.4f,%.4f", r, h);
                } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCapsule>(sh)) {
                    double r = s2->GetRadius(), h = s2->GetHeight();
                    snprintf(kb, 96, "cap:%.4f,%.4f", r, h);
                } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCone>(sh)) {
                    double r = s2->GetRadius(), h = s2->GetHeight();
                    snprintf(kb, 96, "cone:%.4f,%.4f", r, h);
                } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(sh)) {
                    auto S = s2->GetSemiaxes();
                    snprintf(kb, 96, "ell:%.4f,%.4f,%.4f", S.x(), S.y(), S.z());
                } else
                    ok = false;
                if (!ok)
                    continue;
                // texture-aware cache key: same-size primitives with different textures must not share geometry
                auto& pmats = sh->GetMaterials();
                std::string ptex = pmats.empty() ? std::string() : pmats[0]->GetKdTexture();
                std::string k = std::string(kb) + materialKey(pmats);
                if (!m_geom_cache.count(k)) {
                    if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeBox>(sh)) {
                        auto L = s2->GetLengths();
                        genBox(L.x(), L.y(), L.z(), g);
                    } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeSphere>(sh)) {
                        double r = s2->GetRadius();
                        genSphere(r, r, r, 16, 28, g);
                    } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCylinder>(sh)) {
                        double r = s2->GetRadius(), h = s2->GetHeight();
                        genCyl(r, h, 24, g);
                    } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCapsule>(sh)) {
                        double r = s2->GetRadius(), h = s2->GetHeight();
                        genCapsule(r, h, 24, g);
                    } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeCone>(sh)) {
                        double r = s2->GetRadius(), h = s2->GetHeight();
                        genCone(r, h, 24, g);
                    } else if (auto s2 = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(sh)) {
                        auto S = s2->GetSemiaxes();
                        genSphere(S.x(), S.y(), S.z(), 16, 28, g);
                    }
                    if (g.verts.empty())
                        continue;
                    // primitives carry a single material (index 0); UVs are generated by gen*(), so textures tile
                    std::vector<int> fm(g.triCount(), pmats.empty() ? -1 : 0);
                    toTexIds(g, fm, pmats);
                    m_geom_cache[k] = (int)scene.geometries.size();
                    scene.geometries.push_back(std::move(g));
                }
                // textured primitive -> white tint so the texture shows true; untextured -> flat diffuse color
                float tintv[3] = {col[0], col[1], col[2]};
                if (!ptex.empty()) {
                    tintv[0] = tintv[1] = tintv[2] = 1.f;
                }
                addInstance(m_geom_cache[k], body, sf, tintv, 0, false, nullptr);
            }
        }
    };
    // Other physics items are walked exactly like bodies, as ChOptixEngine::ConstructScene does.
    // They carry no transform of their own, hence the null body.
    for (auto& it : m_sys->GetOtherPhysicsItems())
        processItem(it.get(), nullptr);
    for (auto& b : m_sys->GetBodies())
        processItem(b.get(), b.get());

    m_last_shape_count = CountShapes();
    Refresh(scene);
}

void ChMetalSceneBuilder::Refresh(MetalRenderScene& scene) {
    std::vector<int> fmTmp;
    MetalGeometry gTmp;
    const DefaultMaterial& DMr = defaultMaterial();
    const float vdefRefresh[3] = {DMr.kd[0], DMr.kd[1], DMr.kd[2]};
    for (size_t i = 0; i < m_srcs.size() && i < scene.instances.size(); ++i) {
        InstSrc& s = m_srcs[i];
        MetalInstance& in = scene.instances[i];
        ChFramed shapeF(ChVector3d(s.sf[4], s.sf[5], s.sf[6]), ChQuaterniond(s.sf[0], s.sf[1], s.sf[2], s.sf[3]));
        // Compose against the body's VISUAL-MODEL frame, not GetPos()/GetRot(): for a
        // ChBodyAuxRef (e.g. a vehicle chassis) visuals attach at the reference frame, while
        // GetPos() is the center of mass. Using GetPos() renders the body offset by the COM
        // gap (the Audi COM is 0.55 m above its ref -> body floats above the wheels).
        ChFramed world = s.body ? (shapeF >> s.body->GetVisualModelFrame()) : shapeF;
        fillFrame(in, world);
        if (s.body) {
            auto v = s.body->GetPosDt();
            in.vel[0] = (float)v.x();
            in.vel[1] = (float)v.y();
            in.vel[2] = (float)v.z();
        } else {
            in.vel[0] = in.vel[1] = in.vel[2] = 0.f;
        }
        if (s.dynamic && s.mesh) {  // re-extract deforming mesh in place
            MetalGeometry& g = scene.geometries[s.geom];
            // Only positions and normals are taken; the shading arrays built by Build() stay in
            // place, so the material arguments here are immaterial and are left at the defaults.
            meshToGeom(s.mesh, gTmp, fmTmp, {}, vdefRefresh[0], vdefRefresh[1], vdefRefresh[2], ChVector3d(1, 1, 1));
            if (gTmp.triCount() == g.triCount()) {
                g.verts.swap(gTmp.verts);
                g.normals.swap(gTmp.normals);
            }
        }
    }
}

}  // namespace sensor
}  // namespace chrono
