// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include <cmath>

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChColor.h"
#include "chrono/utils/ChProfiler.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

namespace irr {
namespace core {

vector3dfCH::vector3dfCH(const chrono::ChVector3d& mch) {
    X = ((f32)mch.x());
    Y = ((f32)mch.y());
    Z = ((f32)mch.z());
}

matrix4CH::matrix4CH(const chrono::ChCoordsys<>& csys) : matrix4CH(chrono::ChFrame<>(csys)) {}

matrix4CH::matrix4CH(const chrono::ChFrame<>& frame) {
    const auto& v = frame.GetPos();
    const auto& A = frame.GetRotMat();

    //// RADU
    //// Is this actually correct?   ChMatrix33 is also stored row-major (even pre-Eigen)!
    //// Or is Irrlicht actually using column-major?

    // Fill the upper 3x3 submatrix with the [A] matrix transposed, since Irrlicht uses the row-major style as in D3D
    (*this)[0] = (irr::f32)A(0);
    (*this)[1] = (irr::f32)A(3);
    (*this)[2] = (irr::f32)A(6);

    (*this)[4] = (irr::f32)A(1);
    (*this)[5] = (irr::f32)A(4);
    (*this)[6] = (irr::f32)A(7);

    (*this)[8] = (irr::f32)A(2);
    (*this)[9] = (irr::f32)A(5);
    (*this)[10] = (irr::f32)A(8);

    (*this)[12] = (irr::f32)v.x();
    (*this)[13] = (irr::f32)v.y();
    (*this)[14] = (irr::f32)v.z();

    // Clear the last column to 0 and set low-right corner to 1
    // as in Denavitt-Hartemberg matrices, transposed.
    (*this)[3] = (*this)[7] = (*this)[11] = 0.0f;
    (*this)[15] = 1.0f;
}

}  // namespace core
}  // namespace irr

namespace chrono {
namespace irrlicht {
namespace tools {

using namespace irr;
using namespace irr::core;

// -----------------------------------------------------------------------------

video::SColorf ToIrrlichtSColorf(const ChColor& col) {
    return video::SColorf(col.R, col.G, col.B, 1.0);
}

video::SColor ToIrrlichtSColor(const ChColor& col, float alpha) {
    return video::SColor((u32)(alpha * 255), (u32)(col.R * 255), (u32)(col.G * 255), (u32)(col.B * 255));
}

// -----------------------------------------------------------------------------

video::SMaterial ToIrrlichtMaterial(std::shared_ptr<ChVisualMaterial> mat, video::IVideoDriver* driver) {
    video::SMaterial irr_mat;

    irr_mat.AmbientColor = ToIrrlichtSColor(mat->GetAmbientColor());
    irr_mat.DiffuseColor = ToIrrlichtSColor(mat->GetDiffuseColor());
    irr_mat.SpecularColor = ToIrrlichtSColor(mat->GetSpecularColor());
    irr_mat.EmissiveColor = ToIrrlichtSColor(mat->GetEmissiveColor());

    float dval = mat->GetOpacity();  // in [0,1]
    irr_mat.DiffuseColor.setAlpha((s32)(dval * 255));
    if (dval < 1.0f)
        irr_mat.MaterialType = video::EMT_TRANSPARENT_VERTEX_ALPHA;

    float ns_val = mat->GetSpecularExponent();  // in [0, 1000]
    irr_mat.Shininess = ns_val * 0.128f;

    auto scale = mat->GetTextureScale();

    auto kd_texture_name = mat->GetKdTexture();
    if (!kd_texture_name.empty()) {
        auto kd_texture = driver->getTexture(kd_texture_name.c_str());
        irr_mat.setTexture(0, kd_texture);
        irr_mat.getTextureMatrix(0).setTextureScale(scale.x(), scale.y());

        // Same as when Irrlicht loads the OBJ+MTL.  Is this really needed?
        irr_mat.DiffuseColor.setRed(255);
        irr_mat.DiffuseColor.setGreen(255);
        irr_mat.DiffuseColor.setBlue(255);
    }

    auto normal_texture_name = mat->GetNormalMapTexture();
    if (!normal_texture_name.empty()) {
        auto normal_texture = driver->getTexture(normal_texture_name.c_str());
        irr_mat.setTexture(1, normal_texture);
        irr_mat.getTextureMatrix(1).setTextureScale(scale.x(), scale.y());

        // Same as when Irrlicht loads the OBJ+MTL.  Is this really needed?
        irr_mat.DiffuseColor.setRed(255);
        irr_mat.DiffuseColor.setGreen(255);
        irr_mat.DiffuseColor.setBlue(255);
    }

    auto opacity_texture_name = mat->GetOpacityTexture();
    if (!opacity_texture_name.empty()) {
        auto opacity_texture = driver->getTexture(opacity_texture_name.c_str());
        irr_mat.setTexture(2, opacity_texture);
        irr_mat.getTextureMatrix(2).setTextureScale(scale.x(), scale.y());

        irr_mat.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL;

        // Same as when Irrlicht loads the OBJ+MTL.  Is this really needed?
        ////irr_mat.DiffuseColor.setRed(255);
        ////irr_mat.DiffuseColor.setGreen(255);
        ////irr_mat.DiffuseColor.setBlue(255);
    }

    return irr_mat;
}

// -----------------------------------------------------------------------------
// Align an Irrlicht object to a the specified coordinate system.
// -----------------------------------------------------------------------------
void alignIrrlichtNode(scene::ISceneNode* mnode, const ChCoordsys<>& mcoords) {
    // Construct the equivalent 4x4 Irrlicht matrix
    irr::core::matrix4CH irrMat(mcoords);

    // Set position and rotation of node using the Irrlicht matrix
    mnode->setPosition(irrMat.getTranslation());
    mnode->setRotation(irrMat.getRotationDegrees());
}

// -----------------------------------------------------------------------------
// Draw contact points.
// Uses the _draw_reporter_class callback class.
// -----------------------------------------------------------------------------
class _draw_reporter_class : public ChContactContainer::ReportContactCallback {
  public:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_Radius,
                                 const ChVector3d& react_forces,
                                 const ChVector3d& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB,
                                 int constraint_offset) override {
        ChMatrix33<>& mplanecoord = const_cast<ChMatrix33<>&>(plane_coord);
        ChVector3d v1 = pA;
        ChVector3d v2;
        ChVector3d vn = mplanecoord.GetAxisX();

        irr::video::SColor mcol = irr::video::SColor(200, 255, 0, 0);

        switch (drawtype) {
            case ContactsDrawMode::CONTACT_DISTANCES:
                v2 = pB;
                if (distance > 0.0)
                    mcol = irr::video::SColor(200, 20, 255, 0);  // green: non penetration
                else
                    mcol = irr::video::SColor(200, 255, 60, 60);  // red: penetration
                break;
            case ContactsDrawMode::CONTACT_NORMALS:
                v2 = pA + vn * clen;

                v1 = pB;
                v2 = pB - vn * clen;

                mcol = irr::video::SColor(200, 0, 100, 255);
                break;
            case ContactsDrawMode::CONTACT_FORCES_N:
                v2 = pA + vn * clen * react_forces.x();
                break;
            case ContactsDrawMode::CONTACT_FORCES:
                v2 = pA + (mplanecoord * (react_forces * clen));
                break;
            default:
                break;
        }

        this->cdriver->draw3DLine(irr::core::vector3dfCH(v1), irr::core::vector3dfCH(v2), mcol);
        return true;  // to continue scanning contacts
    }

    irr::video::IVideoDriver* cdriver;
    ContactsDrawMode drawtype;
    double clen;
};

int drawAllContactPoints(ChVisualSystemIrrlicht* vis, double mlen, ContactsDrawMode drawtype) {
    if (drawtype == ContactsDrawMode::CONTACT_NONE)
        return 0;

    // if (sys.GetNumContacts() == 0)
    //    return 0;

    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    auto my_drawer = chrono_types::make_shared<_draw_reporter_class>();

    my_drawer->cdriver = vis->GetVideoDriver();
    my_drawer->clen = mlen;
    my_drawer->drawtype = drawtype;

    // scan all contacts
    vis->GetSystem(0).GetContactContainer()->ReportAllContacts(my_drawer);

    return 0;
}

// -----------------------------------------------------------------------------
// Draw contact information as labels at the contact point.
// Uses the _label_reporter_class callback class.
// -----------------------------------------------------------------------------
class _label_reporter_class : public ChContactContainer::ReportContactCallback {
  public:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const ChVector3d& react_forces,
                                 const ChVector3d& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB,
                                 int constraint_offset) override {
        char buffer[25];
        irr::core::vector3df mpos((irr::f32)pA.x(), (irr::f32)pA.y(), (irr::f32)pA.z());
        irr::core::position2d<s32> spos =
            this->cdevice->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
        gui::IGUIFont* font = this->cdevice->getGUIEnvironment()->getBuiltInFont();

        switch (labeltype) {
            case ContactsLabelMode::CONTACT_DISTANCES_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", distance);
                break;
            case ContactsLabelMode::CONTACT_FORCES_N_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", react_forces.x());
                break;
            case ContactsLabelMode::CONTACT_FORCES_T_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", ChVector3d(0, react_forces.y(), react_forces.z()).Length());
                break;
            case ContactsLabelMode::CONTACT_FORCES_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", ChVector3d(react_forces).Length());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", ChVector3d(react_torques).Length());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_S_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", react_torques.x());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_R_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g",
                         ChVector3d(0, react_torques.y(), react_torques.z()).Length());
                break;
            default:
                break;
        }

        font->draw(irr::core::stringw(buffer).c_str(),
                   irr::core::rect<s32>(spos.X - 15, spos.Y, spos.X + 15, spos.Y + 10), ccol);

        return true;  // to continue scanning contacts
    }

    irr::IrrlichtDevice* cdevice;
    ContactsLabelMode labeltype;
    irr::video::SColor ccol;
};

int drawAllContactLabels(ChVisualSystemIrrlicht* vis, ContactsLabelMode labeltype, ChColor col) {
    if (labeltype == ContactsLabelMode::CONTACT_NONE_VAL)
        return 0;

    // if (sys.GetNumContacts() == 0)
    //   return 0;

    auto my_label_rep = chrono_types::make_shared<_label_reporter_class>();

    my_label_rep->cdevice = vis->GetDevice();
    my_label_rep->ccol = tools::ToIrrlichtSColor(col);
    my_label_rep->labeltype = labeltype;

    // scan all contacts
    vis->GetSystem(0).GetContactContainer()->ReportAllContacts(my_label_rep);

    return 0;
}

// -----------------------------------------------------------------------------
// Draw links as glyps.
// ---------------------------------------------------------------------------
int drawAllLinks(ChVisualSystemIrrlicht* vis, double mlen, LinkDrawMode drawtype) {
    if (drawtype == LinkDrawMode::LINK_NONE)
        return 0;

    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (auto& link : vis->GetSystem(0).GetLinks()) {
        ChFrame<> frame = link->GetFrame2Abs();
        ChWrenchd react = link->GetReaction2();
        ChVector3d v1abs = frame.GetCoordsys().pos;
        ChVector3d v2;
        switch (drawtype) {
            case LinkDrawMode::LINK_REACT_FORCE:
                v2 = react.force;
                break;
            case LinkDrawMode::LINK_REACT_TORQUE:
                v2 = react.torque;
                break;
            default:
                break;
        }
        v2 *= mlen;
        ChVector3d v2abs = v2 >> frame;

        video::SColor mcol(200, 250, 250, 0);  // yellow vectors

        vis->GetVideoDriver()->draw3DLine(vector3dfCH(v1abs), vector3dfCH(v2abs), mcol);
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw links as labels
// ---------------------------------------------------------------------------
int drawAllLinkLabels(ChVisualSystemIrrlicht* vis, LinkLabelMode labeltype, ChColor col) {
    if (labeltype == LinkLabelMode::LINK_NONE_VAL)
        return 0;

    auto device = vis->GetDevice();
    char buffer[25];

    for (auto& link : vis->GetSystem(0).GetLinks()) {
        const auto& frame = link->GetFrame2Abs();
        const auto& react = link->GetReaction2();
        const auto& force = react.force;
        const auto& torque = react.torque;

        position2d<s32> spos =
            device->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(
                vector3dfCH(frame.GetPos()));
        gui::IGUIFont* font = device->getGUIEnvironment()->getBuiltInFont();

        switch (labeltype) {
            case LinkLabelMode::LINK_REACT_FORCE_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", force.Length());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_X:
                snprintf(buffer, sizeof(buffer), "% 6.3g", force.x());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_Y:
                snprintf(buffer, sizeof(buffer), "% 6.3g", force.y());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_Z:
                snprintf(buffer, sizeof(buffer), "% 6.3g", force.z());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_VAL:
                snprintf(buffer, sizeof(buffer), "% 6.3g", torque.Length());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_X:
                snprintf(buffer, sizeof(buffer), "% 6.3g", torque.x());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_Y:
                snprintf(buffer, sizeof(buffer), "% 6.3g", torque.y());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_Z:
                snprintf(buffer, sizeof(buffer), "% 6.3g", torque.z());
                break;
            default:
                break;
        }

        font->draw(stringw(buffer).c_str(), rect<s32>(spos.X - 15, spos.Y, spos.X + 15, spos.Y + 10),
                   tools::ToIrrlichtSColor(col));
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw collision objects bounding boxes for rigid bodies.
// -----------------------------------------------------------------------------
int drawAllBoundingBoxes(ChVisualSystemIrrlicht* vis) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (auto& body : vis->GetSystem(0).GetBodies()) {
        irr::video::SColor mcol;

        if (body->IsSleeping())
            mcol = irr::video::SColor(70, 0, 50, 255);  // blue: sleeping
        else
            mcol = irr::video::SColor(70, 30, 200, 200);  // cyan: not sleeping

        auto bbox = body->GetTotalAABB();
        if (bbox.IsInverted())
            continue;

        ChVector3d lo = bbox.min;
        ChVector3d hi = bbox.max;

        ChVector3d p1(hi.x(), lo.y(), lo.z());
        ChVector3d p2(lo.x(), hi.y(), lo.z());
        ChVector3d p3(lo.x(), lo.y(), hi.z());
        ChVector3d p4(hi.x(), hi.y(), lo.z());
        ChVector3d p5(lo.x(), hi.y(), hi.z());
        ChVector3d p6(hi.x(), lo.y(), hi.z());
        ChVector3d p7(lo.x(), lo.y(), hi.z());
        ChVector3d p8(lo.x(), lo.y(), hi.z());
        ChVector3d p9(lo.x(), hi.y(), lo.z());
        ChVector3d p10(lo.x(), hi.y(), lo.z());
        ChVector3d p11(hi.x(), lo.y(), lo.z());
        ChVector3d p12(hi.x(), lo.y(), lo.z());
        ChVector3d p14(hi.x(), lo.y(), hi.z());
        ChVector3d p15(lo.x(), hi.y(), hi.z());
        ChVector3d p16(lo.x(), hi.y(), hi.z());
        ChVector3d p17(hi.x(), hi.y(), lo.z());
        ChVector3d p18(hi.x(), lo.y(), hi.z());
        ChVector3d p19(hi.x(), hi.y(), lo.z());
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p1), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p2), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p3), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p4), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p5), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p6), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p7), irr::core::vector3dfCH(p14), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p8), irr::core::vector3dfCH(p15), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p9), irr::core::vector3dfCH(p16), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p10), irr::core::vector3dfCH(p17), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p11), irr::core::vector3dfCH(p18), mcol);
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p12), irr::core::vector3dfCH(p19), mcol);
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw coordinate systems of ChBody objects bodies.
// -----------------------------------------------------------------------------
int drawAllCOGs(ChVisualSystemIrrlicht* vis, double scale) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (auto& body : vis->GetSystem(0).GetBodies()) {
        irr::video::SColor mcol;
        const ChFrame<>& mframe_cog = body->GetFrameCOMToAbs();
        const ChFrame<>& mframe_ref = body->GetFrameRefToAbs();

        ChVector3d p0 = mframe_cog.GetPos();
        ChVector3d px = p0 + mframe_cog.GetRotMat().GetAxisX() * 0.5 * scale;
        ChVector3d py = p0 + mframe_cog.GetRotMat().GetAxisY() * 0.5 * scale;
        ChVector3d pz = p0 + mframe_cog.GetRotMat().GetAxisZ() * 0.5 * scale;

        mcol = irr::video::SColor(70, 125, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
        mcol = irr::video::SColor(70, 0, 125, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
        mcol = irr::video::SColor(70, 0, 0, 125);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);

        p0 = mframe_ref.GetPos();
        px = p0 + mframe_ref.GetRotMat().GetAxisX() * scale;
        py = p0 + mframe_ref.GetRotMat().GetAxisY() * scale;
        pz = p0 + mframe_ref.GetRotMat().GetAxisZ() * scale;

        mcol = irr::video::SColor(70, 255, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
        mcol = irr::video::SColor(70, 0, 255, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
        mcol = irr::video::SColor(70, 0, 0, 255);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw coordinate systems of frames used by links.
// -----------------------------------------------------------------------------
int drawAllLinkframes(ChVisualSystemIrrlicht* vis, double scale) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (auto& link : vis->GetSystem(0).GetLinks()) {
        ChFrame<> frame1 = link->GetFrame1Abs();
        ChFrame<> frame2 = link->GetFrame2Abs();

        irr::video::SColor col;

        ChVector3d p0 = frame1.GetPos();
        ChVector3d px = p0 + frame1.GetRotMat().GetAxisX() * 0.7 * scale;
        ChVector3d py = p0 + frame1.GetRotMat().GetAxisY() * 0.7 * scale;
        ChVector3d pz = p0 + frame1.GetRotMat().GetAxisZ() * 0.7 * scale;

        col = irr::video::SColor(70, 125, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), col);
        col = irr::video::SColor(70, 0, 125, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), col);
        col = irr::video::SColor(70, 0, 0, 125);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), col);

        p0 = frame2.GetPos();
        px = p0 + frame2.GetRotMat().GetAxisX() * scale;
        py = p0 + frame2.GetRotMat().GetAxisY() * scale;
        pz = p0 + frame2.GetRotMat().GetAxisZ() * scale;

        col = irr::video::SColor(70, 255, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), col);
        col = irr::video::SColor(70, 0, 255, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), col);
        col = irr::video::SColor(70, 0, 0, 255);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), col);
    }

    return 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void drawHUDviolation(ChVisualSystemIrrlicht* vis, int pos_x, int pos_y, int width, int height) {
    auto solver = std::dynamic_pointer_cast<ChIterativeSolver>(vis->GetSystem(0).GetSolver());
    if (!solver || solver->GetIterations() == 0)
        return;

    auto solver_vi = std::dynamic_pointer_cast<ChIterativeSolverVI>(solver);

    if (solver_vi)
        solver_vi->SetRecordViolation(true);

    double solver_tol = solver->GetTolerance();
    int tolerance_line_y = (int)(height * 0.66);
    // the iterative solvers have a default tolerance of 0, so we need to have a guard against division by zero
    if (solver->GetTolerance() == 0) {
        tolerance_line_y = height - 1;
        solver_tol = 1e-6;
    }

    double deltalambda_max =
        solver_vi ? *std::max_element(solver_vi->GetDeltalambdaHistory().begin(),
                                      solver_vi->GetDeltalambdaHistory().begin() + solver->GetIterations())
                  : 1.0;

    // WARNING: here it is assumed that non-VI solvers have no error history
    int num_bars = solver_vi ? std::min((int)solver->GetMaxIterations(), 50) : 1;
    double num_it_per_bar = solver_vi ? (double)solver->GetMaxIterations() / (double)num_bars : 1;
    // width = num_spacings*spacing_width + num_bars*viol_bar_width
    // num_spacings = num_bars + 1
    // spacing_width = 1/4*viol_bar_width;
    double viol_bar_width_D = 4.0 * width / (5. * num_bars + 1.0);
    int viol_bar_width = (int)viol_bar_width_D;
    int spacing_width = std::max((int)(viol_bar_width_D / 4.), 1);
    int lamb_bar_width = std::min((int)(3.0 / 4.0 * viol_bar_width_D), viol_bar_width - 1);
    int actual_width = num_bars * viol_bar_width + (num_bars + 1) * spacing_width;

    irr::core::rect<s32> mrect(pos_x, pos_y, pos_x + actual_width, pos_y + height);
    vis->GetVideoDriver()->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);
    int cur_bar = -1;
    double cur_error = 0;
    double cur_deltalambda = 0;

    std::function<double(int)> getErrorI;
    if (solver_vi)
        getErrorI = [solver_vi](int i) { return solver_vi->GetViolationHistory().at(i); };
    else
        getErrorI = [solver](int i) { return solver->GetError(); };

    for (auto i = 0; i < solver->GetIterations(); i++) {
        cur_deltalambda = solver_vi ? std::max(cur_deltalambda, solver_vi->GetDeltalambdaHistory().at(i)) : 0;

        if (i >= num_it_per_bar * (cur_bar + 1) || i == solver->GetIterations() - 1) {
            cur_error = ChClamp(getErrorI(i), solver_tol / 10.0, solver_tol * 100.0);
            cur_bar++;
            int pos_rect_x_start = spacing_width + cur_bar * (viol_bar_width + spacing_width);
            int bar_height = (int)(height * ((1.0 - std::log10(solver_tol) + std::log10(cur_error)) / 3.0));

            // draw the error bar (red)
            vis->GetVideoDriver()->draw2DRectangle(
                irr::video::SColor(90, 255, 0, 0),
                irr::core::rect<s32>(pos_x + pos_rect_x_start, pos_y + height - std::min(height, bar_height),
                                     pos_x + pos_rect_x_start + viol_bar_width, pos_y + height),
                &mrect);

            // draw the tolerance line
            vis->GetVideoDriver()->draw2DLine(
                irr::core::position2d<s32>(pos_x, pos_y + tolerance_line_y),
                irr::core::position2d<s32>(pos_x + actual_width, pos_y + tolerance_line_y),
                irr::video::SColor(90, 255, 0, 20));

            if (solver_vi) {
                // draw the delta lambda bar (yellow)
                vis->GetVideoDriver()->draw2DRectangle(
                    irr::video::SColor(100, 255, 255, 0),
                    irr::core::rect<s32>(
                        pos_x + pos_rect_x_start,
                        pos_y + height - std::min(height, (int)((double)height * (cur_deltalambda / deltalambda_max))),
                        pos_x + pos_rect_x_start + lamb_bar_width, pos_y + height),
                    &mrect);
            }

            // reset counters
            cur_deltalambda = 0;
        }
    }

    if (vis->GetDevice()->getGUIEnvironment()) {
        gui::IGUIFont* font = vis->GetDevice()->getGUIEnvironment()->getBuiltInFont();
        if (font) {
            // print solver iterations at last solve
            char buffer0[100];
            snprintf(buffer0, sizeof(buffer0), "Iters: %d", solver->GetIterations());
            font->draw(irr::core::stringw(buffer0).c_str(),
                       irr::core::rect<s32>(pos_x, pos_y, pos_x + actual_width, pos_y + 10),
                       irr::video::SColor(200, 100, 0, 0));

            // print solver error
            char buffer1[100];
            snprintf(buffer1, sizeof(buffer1), "Error %g", solver->GetError());
            font->draw(irr::core::stringw(buffer1).c_str(),
                       irr::core::rect<s32>(pos_x, pos_y + 10, pos_x + actual_width, pos_y + 20),
                       irr::video::SColor(200, 100, 0, 0));

            // print solver tolerance
            char buffer2[100];
            snprintf(buffer2, sizeof(buffer2), "Tolerance: %g", solver->GetTolerance());
            font->draw(irr::core::stringw(buffer2).c_str(),
                       irr::core::rect<s32>(pos_x, pos_y + tolerance_line_y - 12, pos_x + actual_width,
                                            pos_y + tolerance_line_y + 2),
                       irr::video::SColor(200, 100, 0, 0));

            // print solver delta lambda
            if (solver_vi) {
                char buffer3[100];
                snprintf(buffer3, sizeof(buffer3), "DLambda %g",
                         solver_vi->GetDeltalambdaHistory().at(solver->GetIterations() - 1));
                font->draw(irr::core::stringw(buffer3).c_str(),
                           irr::core::rect<s32>(pos_x, pos_y + 20, pos_x + actual_width, pos_y + 30),
                           irr::video::SColor(200, 100, 0, 0));
            }
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void drawChFunction(ChVisualSystemIrrlicht* vis,
                    std::shared_ptr<chrono::ChFunction> fx,
                    double xmin,
                    double xmax,
                    double ymin,
                    double ymax,
                    int pos_x,
                    int pos_y,
                    int width,
                    int height,
                    chrono::ChColor col,
                    std::string title) {
    irr::video::IVideoDriver* driver = vis->GetDevice()->getVideoDriver();

    if (!fx)
        return;

    irr::core::rect<s32> mrect(pos_x, pos_y, pos_x + width, pos_y + height);
    driver->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);

    if (vis->GetDevice()->getGUIEnvironment()) {
        gui::IGUIFont* font = vis->GetDevice()->getGUIEnvironment()->getBuiltInFont();
        if (font) {
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "%g", ymax);
            font->draw(irr::core::stringw(buffer).c_str(),
                       irr::core::rect<s32>(pos_x, pos_y, pos_x + width, pos_y + 10),
                       irr::video::SColor(200, 100, 0, 0));
            snprintf(buffer, sizeof(buffer), "%g", ymin);
            font->draw(irr::core::stringw(buffer).c_str(),
                       irr::core::rect<s32>(pos_x, pos_y + height, pos_x + width, pos_y + height + 10),
                       irr::video::SColor(200, 100, 0, 0));

            if ((ymin < 0) && (ymax > 0)) {
                int yzero = pos_y + height - (int)(((-ymin) / (ymax - ymin)) * (double)height);
                driver->draw2DLine(irr::core::position2d<s32>(pos_x, yzero),
                                   irr::core::position2d<s32>(pos_x + width, yzero),
                                   irr::video::SColor(90, 255, 255, 255));
                font->draw(irr::core::stringw(buffer).c_str(),
                           irr::core::rect<s32>(pos_x, pos_y + yzero, pos_x + width, pos_y + yzero + 10),
                           irr::video::SColor(200, 100, 0, 0));
            }
        }
        vis->GetDevice()->getGUIEnvironment()->addStaticText(
            irr::core::stringw(title.c_str()).c_str(), irr::core::rect<s32>(pos_x, pos_y - 15, pos_x + width, pos_y));
    }

    int prevx = 0;
    int prevy = 0;

    for (int ix = 0; ix < width; ix++) {
        double x = xmin + (xmax - xmin) * ((double)(ix)) / (double)(width);
        double y = fx->GetVal(x);
        int py = pos_y + height - (int)(((y - ymin) / (ymax - ymin)) * (double)height);
        int px = pos_x + ix;
        if (ix > 0)
            driver->draw2DLine(irr::core::position2d<s32>(px, py), irr::core::position2d<s32>(prevx, prevy),
                               ToIrrlichtSColor(col));
        prevx = px;
        prevy = py;
    }
}

// -----------------------------------------------------------------------------
// Draw segment lines in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawSegment(ChVisualSystemIrrlicht* vis, ChVector3d start, ChVector3d end, chrono::ChColor col, bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = use_Zbuffer;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);
    vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(start), irr::core::vector3dfCH(end),
                                      tools::ToIrrlichtSColor(col));
}

// -----------------------------------------------------------------------------
// Draw a polyline in 3D space, given the array of points as a std::vector.
// -----------------------------------------------------------------------------
void drawPolyline(ChVisualSystemIrrlicht* vis, std::vector<ChVector3d>& points, chrono::ChColor col, bool use_Zbuffer) {
    // not very efficient, but enough as an example..
    if (points.size() < 2)
        return;

    for (unsigned int i = 0; i < points.size() - 1; i++)
        drawSegment(vis, points[i], points[i + 1], col, use_Zbuffer);
}

// -----------------------------------------------------------------------------
// Draw a circle line in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawCircle(ChVisualSystemIrrlicht* vis,
                double radius,
                ChCoordsys<> pos,
                chrono::ChColor col,
                int resolution,
                bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    double phaseA = 0;
    double phaseB = 0;

    for (int iu = 1; iu <= resolution; iu++) {
        phaseB = CH_2PI * (double)iu / (double)resolution;
        ChVector3d V1(radius * std::cos(phaseA), radius * std::sin(phaseA), 0);
        ChVector3d V2(radius * std::cos(phaseB), radius * std::sin(phaseB), 0);
        drawSegment(vis, pos.TransformPointLocalToParent(V1), pos.TransformPointLocalToParent(V2), col, use_Zbuffer);
        phaseA = phaseB;
    }
}

// -----------------------------------------------------------------------------
// Draw a spring in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawSpring(ChVisualSystemIrrlicht* vis,
                double radius,
                ChVector3d start,
                ChVector3d end,
                chrono::ChColor col,
                int resolution,
                double turns,
                bool use_Zbuffer) {
    ChVector3d dist = end - start;
    ChVector3d Vx, Vy, Vz;
    double length = dist.Length();
    ChMatrix33<> rel_matrix;
    rel_matrix.SetFromAxisX(dist, VECT_Y);
    ChQuaternion<> Q12 = rel_matrix.GetQuaternion();
    ChCoordsys<> pos(start, Q12);

    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    double phaseA = 0;
    double phaseB = 0;
    double heightA = 0;
    double heightB = 0;

    for (int iu = 1; iu <= resolution; iu++) {
        phaseB = turns * CH_2PI * (double)iu / (double)resolution;
        heightB = length * ((double)iu / (double)resolution);
        ChVector3d V1(heightA, radius * std::cos(phaseA), radius * std::sin(phaseA));
        ChVector3d V2(heightB, radius * std::cos(phaseB), radius * std::sin(phaseB));
        drawSegment(vis, pos.TransformPointLocalToParent(V1), pos.TransformPointLocalToParent(V2), col, use_Zbuffer);
        phaseA = phaseB;
        heightA = heightB;
    }
}

// -----------------------------------------------------------------------------
// Draw a rotational spring in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawRotSpring(ChVisualSystemIrrlicht* vis,
                   ChCoordsys<> pos,
                   double radius,
                   double start_angle,
                   double end_angle,
                   chrono::ChColor col,
                   int resolution,
                   bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    double del_angle = (end_angle - start_angle) / resolution;
    ChVector3d V1(radius * std::cos(start_angle), radius * std::sin(start_angle), 0);

    for (int iu = 1; iu <= resolution; iu++) {
        double crt_angle = start_angle + iu * del_angle;
        double crt_radius = radius - (iu * del_angle / CH_2PI) * (radius / 10);
        ChVector3d V2(crt_radius * std::cos(crt_angle), crt_radius * std::sin(crt_angle), 0);
        drawSegment(vis, pos.TransformPointLocalToParent(V1), pos.TransformPointLocalToParent(V2), col, use_Zbuffer);
        V1 = V2;
    }
}

// -----------------------------------------------------------------------------
// Draw grids in 3D space, with given orientation, colour and spacing.
// -----------------------------------------------------------------------------
void drawGrid(ChVisualSystemIrrlicht* vis,
              double ustep,
              double vstep,
              int nu,
              int nv,
              ChCoordsys<> pos,
              chrono::ChColor col,
              bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (int iu = -nu / 2; iu <= nu / 2; iu++) {
        ChVector3d V1(iu * ustep, vstep * (nv / 2), 0);
        ChVector3d V2(iu * ustep, -vstep * (nv / 2), 0);
        drawSegment(vis, pos.TransformPointLocalToParent(V1), pos.TransformPointLocalToParent(V2), col, use_Zbuffer);
    }

    for (int iv = -nv / 2; iv <= nv / 2; iv++) {
        ChVector3d V1(ustep * (nu / 2), iv * vstep, 0);
        ChVector3d V2(-ustep * (nu / 2), iv * vstep, 0);
        drawSegment(vis, pos.TransformPointLocalToParent(V1), pos.TransformPointLocalToParent(V2), col, use_Zbuffer);
    }
}

// Easy-to-use function to draw color map 2D legend
void drawColorbar(ChVisualSystemIrrlicht* vis,
                  const ChColormap& colormap,
                  double vmin,
                  double vmax,
                  const std::string& label,
                  int x,
                  int y,
                  int sx,
                  int sy) {
    irr::video::IVideoDriver* driver = vis->GetVideoDriver();

    gui::IGUIFont* font = 0;
    if (vis->GetGUIEnvironment())
        font = vis->GetGUIEnvironment()->getSkin()->getFont();

    int steps = 10;
    double ystep = ((double)sy / (double)steps);
    for (int i = 0; i < steps; ++i) {
        double v_up = vmax - (vmax - vmin) * ((double)(i) / (double)steps);
        double v_dw = vmax - (vmax - vmin) * ((double)(i + 1) / (double)steps);
        core::rect<s32> rect(x, y + (s32)(i * ystep), x + sx, y + (s32)((i + 1) * ystep));
        ChColor c_up = colormap.Get(v_up, vmin, vmax);
        ChColor c_dw = colormap.Get(v_dw, vmin, vmax);
        video::SColor col_up(255, u32(255 * c_up.R), u32(255 * c_up.G), u32(255 * c_up.B));
        video::SColor col_dw(255, u32(255 * c_dw.R), u32(255 * c_dw.G), u32(255 * c_dw.B));
        driver->draw2DRectangle(rect, col_up, col_up, col_dw, col_dw);

        if (font) {
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "%g", v_up);
            font->draw(irr::core::stringw(buffer).c_str(),
                       core::rect<s32>(rect.UpperLeftCorner.X + sx + 6, rect.UpperLeftCorner.Y - 5,
                                       rect.LowerRightCorner.X + sx + 6, rect.LowerRightCorner.Y - 5),
                       irr::video::SColor(255, 0, 0, 0));
            driver->draw2DLine(irr::core::position2d<s32>(rect.UpperLeftCorner.X + sx - 4, rect.UpperLeftCorner.Y),
                               irr::core::position2d<s32>(rect.UpperLeftCorner.X + sx, rect.UpperLeftCorner.Y),
                               irr::video::SColor(255, 100, 100, 100));
        }
    }
    font->draw(irr::core::stringw(label.c_str()).c_str(), core::rect<s32>(x, y + sy + 5, x + 100, y + sy + 20),
               irr::video::SColor(255, 0, 0, 0));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void drawPlot3D(ChVisualSystemIrrlicht* vis,
                ChMatrixConstRef X,
                ChMatrixConstRef Y,
                ChMatrixConstRef Z,
                ChCoordsys<> pos,
                chrono::ChColor col,
                bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    if ((X.cols() != Y.cols()) || (X.cols() != Z.cols()) || (X.rows() != Y.rows()) || (X.rows() != Z.rows())) {
        std::cerr << "drawPlot3D: X Y Z matrices must have the same size, as n.rows and n.columns" << std::endl;
        return;
    }

    for (int iy = 0; iy < X.cols(); ++iy) {
        for (int ix = 0; ix < X.rows(); ++ix) {
            if (ix > 0) {
                ChVector3d Vx1(X(ix - 1, iy), Y(ix - 1, iy), Z(ix - 1, iy));
                ChVector3d Vx2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(vis, pos.TransformPointLocalToParent(Vx1), pos.TransformPointLocalToParent(Vx2), col,
                            use_Zbuffer);
            }

            if (iy > 0) {
                ChVector3d Vy1(X(ix, iy - 1), Y(ix, iy - 1), Z(ix, iy - 1));
                ChVector3d Vy2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(vis, pos.TransformPointLocalToParent(Vy1), pos.TransformPointLocalToParent(Vy2), col,
                            use_Zbuffer);
            }
        }
    }
}

void drawProfilerRecursive(utils::ChProfileIterator* profileIterator,
                           irr::IrrlichtDevice* device,
                           int mx,
                           int my,
                           int sx,
                           int sy,
                           int xspacing,
                           int& ypos) {
    profileIterator->First();
    if (profileIterator->Is_Done())
        return;

    irr::gui::IGUIFont* font = device->getGUIEnvironment()->getSkin()->getFont();  // getBuiltInFont();

    float accumulated_time = 0, parent_time = profileIterator->Is_Root()
                                                  ? utils::ChProfileManager::Get_Time_Since_Reset()
                                                  : profileIterator->Get_Current_Parent_Total_Time();
    int i;
    int frames_since_reset = utils::ChProfileManager::Get_Frame_Count_Since_Reset();

    float tot_frametime = utils::ChProfileManager::Get_Time_Since_Reset();

    char buffer[300];
    irr::video::SColor mcol(255, 255, 255, 90);

    int numChildren = 0;

    for (i = 0; !profileIterator->Is_Done(); i++, profileIterator->Next()) {
        numChildren++;
        float current_total_time = profileIterator->Get_Current_Total_Time();
        accumulated_time += current_total_time;
        float fraction = parent_time > FLT_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
        float fraction_tot = tot_frametime > FLT_EPSILON ? (current_total_time / tot_frametime) * 100 : 0.f;

        irr::core::rect<s32> mrect(mx, my + ypos, mx + (int)(sx * (current_total_time / tot_frametime)),
                                   my + ypos + 18);
        device->getVideoDriver()->draw2DRectangle(
            irr::video::SColor(100, ((xspacing * 200) % 255), ((-xspacing * 151 + 200) % 255), 230), mrect);

        snprintf(buffer, sizeof(buffer), "%d -- %s (%.2f %% parent, %.2f %% tot.) :: %.3f ms / frame (%d calls)\n", i,
                 profileIterator->Get_Current_Name(), fraction, fraction_tot,
                 (current_total_time / (double)frames_since_reset), profileIterator->Get_Current_Total_Calls());
        irr::core::stringw mstring(buffer);
        font->draw(mstring, irr::core::rect<irr::s32>(mx + xspacing, my + ypos, mx + sx, my + ypos + 20), mcol);
        ypos += 20;

        profileIterator->Enter_Child(i);
        drawProfilerRecursive(profileIterator, device, mx, my, sx, sy, xspacing + 30, ypos);
        profileIterator->Enter_Parent();
        for (int j = 0; j < i; ++j)
            profileIterator->Next();
    }

    float fraction = parent_time > FLT_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f;
    float fraction_tot = tot_frametime > FLT_EPSILON ? ((parent_time - accumulated_time) / tot_frametime) * 100 : 0.f;

    irr::core::rect<s32> mrect(mx, my + ypos, mx + (int)(sx * ((parent_time - accumulated_time) / tot_frametime)),
                               my + ypos + 18);
    device->getVideoDriver()->draw2DRectangle(
        irr::video::SColor(100, ((xspacing * 200) % 255), ((-xspacing * 151 + 200) % 255), 230), mrect);

    snprintf(buffer, sizeof(buffer), "n -- %s (%.2f %% parent, %.2f %% tot.) :: %.3f ms\n", "Unaccounted:", fraction,
             fraction_tot, parent_time - accumulated_time);
    irr::core::stringw mstringu(buffer);
    font->draw(mstringu, irr::core::rect<irr::s32>(mx + xspacing, my + ypos, mx + sx, my + ypos + 20), mcol);
    ypos += 20;
}

// Draw run-time profiler infos
void drawProfiler(ChVisualSystemIrrlicht* vis) {
    int pos_x = 230;
    int pos_y = 30;
    int width = 500;
    int height = 400;

    int ypos = 0;
    utils::ChProfileIterator* profileIterator = 0;
    profileIterator = chrono::utils::ChProfileManager::Get_Iterator();

    drawProfilerRecursive(profileIterator, vis->GetDevice(), pos_x, pos_y, width, height, 0, ypos);

    utils::ChProfileManager::Release_Iterator(profileIterator);
}

// Draw RGB coordinate system
void drawCoordsys(ChVisualSystemIrrlicht* vis, const ChCoordsys<>& coord, double scale, bool use_Zbuffer) {
    ChVector3d pos = coord.pos;
    ChQuaternion<> rot = coord.rot;
    // X axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_X) * scale, ChColor(1.f, 0.f, 0.f), use_Zbuffer);
    // Y axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_Y) * scale, ChColor(0.f, 1.f, 0.f), use_Zbuffer);
    // Z axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_Z) * scale, ChColor(0.f, 0.f, 1.f), use_Zbuffer);
}

// -----------------------------------------------------------------------------
// Draw a line arrow in 3D space with given color.
// -----------------------------------------------------------------------------
void drawArrow(ChVisualSystemIrrlicht* vis,
               const ChVector3d& start,
               const ChVector3d& end,
               const ChVector3d& plane_normal,
               bool sharp,
               const ChColor& col,
               bool use_Zbuffer) {
    drawSegment(vis, start, end, col, use_Zbuffer);  // main segment
    ChVector3d dir = (end - start).GetNormalized();
    ChVector3d u, v, w;
    dir.GetDirectionAxesAsX(u, v, w, plane_normal);
    ChVector3d p1, p2;
    if (!sharp) {
        p1 = end + 0.25 * (w - dir);
        p2 = end + 0.25 * (-w - dir);
    } else {
        p1 = end + 0.1 * w - 0.5 * dir;
        p2 = end + 0.1 * -w - 0.5 * dir;
    }
    drawSegment(vis, end, p1, col, use_Zbuffer);  // arrow segment 1
    drawSegment(vis, end, p2, col, use_Zbuffer);  // arrow segment 2
}

// -----------------------------------------------------------------------------
// Draw a label in 3D scene at given position.
// -----------------------------------------------------------------------------
void drawLabel3D(ChVisualSystemIrrlicht* vis,
                 const std::string& text,
                 const ChVector3d& position,
                 const ChColor& color,
                 bool use_Zbuffer) {
    irr::core::position2di spos =
        vis->GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(
            irr::core::vector3dfCH(position));
    auto font = vis->GetGUIEnvironment()->getFont(GetChronoDataFile("fonts/arial8.xml").c_str());
    if (font) {
        font->draw(text.c_str(), irr::core::rect<irr::s32>(spos.X - 15, spos.Y, spos.X + 15, spos.Y + 10),
                   tools::ToIrrlichtSColor(color));
    }
}

}  // end namespace tools
}  // end namespace irrlicht
}  // end namespace chrono
