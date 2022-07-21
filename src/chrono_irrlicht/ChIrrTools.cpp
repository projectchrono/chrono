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

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChColor.h"
#include "chrono/utils/ChProfiler.h"

#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

namespace irr {
namespace core {

vector3dfCH::vector3dfCH(const chrono::ChVector<>& mch) {
    X = ((f32)mch.x());
    Y = ((f32)mch.y());
    Z = ((f32)mch.z());
}

matrix4CH::matrix4CH(const chrono::ChCoordsys<>& csys) : matrix4CH(chrono::ChFrame<>(csys)) {}

matrix4CH::matrix4CH(const chrono::ChFrame<>& frame) {
    const auto& v = frame.GetPos();
    const auto& A = frame.GetA();

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

    auto kd_texture_name = mat->GetKdTexture();
    if (!kd_texture_name.empty()) {
        auto scale = mat->GetKdTextureScale();
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
        auto scale = mat->GetNormalMapTextureScale();
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
        auto scale = mat->GetOpacityTextureScale();
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
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_Radius,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        ChMatrix33<>& mplanecoord = const_cast<ChMatrix33<>&>(plane_coord);
        ChVector<> v1 = pA;
        ChVector<> v2;
        ChVector<> vn = mplanecoord.Get_A_Xaxis();

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

    // if (sys.GetNcontacts() == 0)
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
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        char buffer[25];
        irr::core::vector3df mpos((irr::f32)pA.x(), (irr::f32)pA.y(), (irr::f32)pA.z());
        irr::core::position2d<s32> spos =
            this->cdevice->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
        gui::IGUIFont* font = this->cdevice->getGUIEnvironment()->getBuiltInFont();

        switch (labeltype) {
            case ContactsLabelMode::CONTACT_DISTANCES_VAL:
                sprintf(buffer, "% 6.3g", distance);
                break;
            case ContactsLabelMode::CONTACT_FORCES_N_VAL:
                sprintf(buffer, "% 6.3g", react_forces.x());
                break;
            case ContactsLabelMode::CONTACT_FORCES_T_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(0, react_forces.y(), react_forces.z()).Length());
                break;
            case ContactsLabelMode::CONTACT_FORCES_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(react_forces).Length());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(react_torques).Length());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_S_VAL:
                sprintf(buffer, "% 6.3g", react_torques.x());
                break;
            case ContactsLabelMode::CONTACT_TORQUES_R_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(0, react_torques.y(), react_torques.z()).Length());
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

    // if (sys.GetNcontacts() == 0)
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

    for (auto& link : vis->GetSystem(0).Get_linklist()) {
        ChCoordsys<> mlinkframe = link->GetLinkAbsoluteCoords();
        ChVector<> v1abs = mlinkframe.pos;
        ChVector<> v2;
        switch (drawtype) {
            case LinkDrawMode::LINK_REACT_FORCE:
                v2 = link->Get_react_force();
                break;
            case LinkDrawMode::LINK_REACT_TORQUE:
                v2 = link->Get_react_torque();
                break;
            default:
                break;
        }
        v2 *= mlen;
        ChVector<> v2abs = v2 >> mlinkframe;

        irr::video::SColor mcol(200, 250, 250, 0);  // yellow vectors

        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(v1abs), irr::core::vector3dfCH(v2abs), mcol);
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw links as labels
// ---------------------------------------------------------------------------
int drawAllLinkLabels(ChVisualSystemIrrlicht* vis, LinkLabelMode labeltype, ChColor col) {
    if (labeltype == LinkLabelMode::LINK_NONE_VAL)
        return 0;

    for (auto& link : vis->GetSystem(0).Get_linklist()) {
        ChCoordsys<> mlinkframe = link->GetLinkAbsoluteCoords();

        char buffer[25];
        irr::core::vector3df mpos((irr::f32)mlinkframe.pos.x(), (irr::f32)mlinkframe.pos.y(),
                                  (irr::f32)mlinkframe.pos.z());
        irr::core::position2d<s32> spos =
            vis->GetDevice()->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
        gui::IGUIFont* font = vis->GetDevice()->getGUIEnvironment()->getBuiltInFont();

        switch (labeltype) {
            case LinkLabelMode::LINK_REACT_FORCE_VAL:
                sprintf(buffer, "% 6.3g", link->Get_react_force().Length());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_X:
                sprintf(buffer, "% 6.3g", link->Get_react_force().x());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_Y:
                sprintf(buffer, "% 6.3g", link->Get_react_force().y());
                break;
            case LinkLabelMode::LINK_REACT_FORCE_Z:
                sprintf(buffer, "% 6.3g", link->Get_react_force().z());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_VAL:
                sprintf(buffer, "% 6.3g", link->Get_react_torque().Length());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_X:
                sprintf(buffer, "% 6.3g", link->Get_react_torque().x());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_Y:
                sprintf(buffer, "% 6.3g", link->Get_react_torque().y());
                break;
            case LinkLabelMode::LINK_REACT_TORQUE_Z:
                sprintf(buffer, "% 6.3g", link->Get_react_torque().z());
                break;
            default:
                break;
        }

        font->draw(irr::core::stringw(buffer).c_str(),
                   irr::core::rect<s32>(spos.X - 15, spos.Y, spos.X + 15, spos.Y + 10), tools::ToIrrlichtSColor(col));
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

    for (auto& body : vis->GetSystem(0).Get_bodylist()) {
        irr::video::SColor mcol;

        if (body->GetSleeping())
            mcol = irr::video::SColor(70, 0, 50, 255);  // blue: sleeping
        else
            mcol = irr::video::SColor(70, 30, 200, 200);  // cyan: not sleeping

        ChVector<> hi = VNULL;
        ChVector<> lo = VNULL;
        body->GetTotalAABB(lo, hi);
        ChVector<> p1(hi.x(), lo.y(), lo.z());
        ChVector<> p2(lo.x(), hi.y(), lo.z());
        ChVector<> p3(lo.x(), lo.y(), hi.z());
        ChVector<> p4(hi.x(), hi.y(), lo.z());
        ChVector<> p5(lo.x(), hi.y(), hi.z());
        ChVector<> p6(hi.x(), lo.y(), hi.z());
        ChVector<> p7(lo.x(), lo.y(), hi.z());
        ChVector<> p8(lo.x(), lo.y(), hi.z());
        ChVector<> p9(lo.x(), hi.y(), lo.z());
        ChVector<> p10(lo.x(), hi.y(), lo.z());
        ChVector<> p11(hi.x(), lo.y(), lo.z());
        ChVector<> p12(hi.x(), lo.y(), lo.z());
        ChVector<> p14(hi.x(), lo.y(), hi.z());
        ChVector<> p15(lo.x(), hi.y(), hi.z());
        ChVector<> p16(lo.x(), hi.y(), hi.z());
        ChVector<> p17(hi.x(), hi.y(), lo.z());
        ChVector<> p18(hi.x(), lo.y(), hi.z());
        ChVector<> p19(hi.x(), hi.y(), lo.z());
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

    for (auto& body : vis->GetSystem(0).Get_bodylist()) {
        irr::video::SColor mcol;
        const ChFrame<>& mframe_cog = body->GetFrame_COG_to_abs();
        const ChFrame<>& mframe_ref = body->GetFrame_REF_to_abs();

        ChVector<> p0 = mframe_cog.GetPos();
        ChVector<> px = p0 + mframe_cog.GetA().Get_A_Xaxis() * 0.5 * scale;
        ChVector<> py = p0 + mframe_cog.GetA().Get_A_Yaxis() * 0.5 * scale;
        ChVector<> pz = p0 + mframe_cog.GetA().Get_A_Zaxis() * 0.5 * scale;

        mcol = irr::video::SColor(70, 125, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
        mcol = irr::video::SColor(70, 0, 125, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
        mcol = irr::video::SColor(70, 0, 0, 125);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);

        p0 = mframe_ref.GetPos();
        px = p0 + mframe_ref.GetA().Get_A_Xaxis() * scale;
        py = p0 + mframe_ref.GetA().Get_A_Yaxis() * scale;
        pz = p0 + mframe_ref.GetA().Get_A_Zaxis() * scale;

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

    for (auto& link : vis->GetSystem(0).Get_linklist()) {
        ChFrame<> frAabs;
        ChFrame<> frBabs;

        // default frame alignment:

        frAabs = link->GetVisualModelFrame();
        frBabs = frAabs;

        // special cases:

        if (auto mylink = std::dynamic_pointer_cast<ChLinkMarkers>(link)) {
            frAabs = *mylink->GetMarker1() >> *mylink->GetBody1();
            frBabs = *mylink->GetMarker2() >> *mylink->GetBody2();
        }

        if (auto mylink = std::dynamic_pointer_cast<ChLinkMateGeneric>(link)) {
            frAabs = mylink->GetFrame1() >> *mylink->GetBody1();
            frBabs = mylink->GetFrame2() >> *mylink->GetBody2();
        }

        irr::video::SColor mcol;

        ChVector<> p0 = frAabs.GetPos();
        ChVector<> px = p0 + frAabs.GetA().Get_A_Xaxis() * 0.7 * scale;
        ChVector<> py = p0 + frAabs.GetA().Get_A_Yaxis() * 0.7 * scale;
        ChVector<> pz = p0 + frAabs.GetA().Get_A_Zaxis() * 0.7 * scale;

        mcol = irr::video::SColor(70, 125, 0, 0);  // X red
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
        mcol = irr::video::SColor(70, 0, 125, 0);  // Y green
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
        mcol = irr::video::SColor(70, 0, 0, 125);  // Z blue
        vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);

        p0 = frBabs.GetPos();
        px = p0 + frBabs.GetA().Get_A_Xaxis() * scale;
        py = p0 + frBabs.GetA().Get_A_Yaxis() * scale;
        pz = p0 + frBabs.GetA().Get_A_Zaxis() * scale;

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
// -----------------------------------------------------------------------------
void drawHUDviolation(ChVisualSystemIrrlicht* vis, int mx, int my, int sx, int sy, double spfact) {
    auto msolver_speed = std::dynamic_pointer_cast<ChIterativeSolverVI>(vis->GetSystem(0).GetSolver());
    if (!msolver_speed)
        return;

    msolver_speed->SetRecordViolation(true);

    irr::core::rect<s32> mrect(mx, my, mx + sx, my + sy);
    vis->GetVideoDriver()->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);
    for (unsigned int i = 0; i < msolver_speed->GetViolationHistory().size(); i++) {
        vis->GetVideoDriver()->draw2DRectangle(
            irr::video::SColor(90, 255, 0, 0),
            irr::core::rect<s32>(mx + i * 4, sy + my - (int)(spfact * msolver_speed->GetViolationHistory()[i]),
                                 mx + (i + 1) * 4 - 1, sy + my),
            &mrect);
    }
    for (unsigned int i = 0; i < msolver_speed->GetDeltalambdaHistory().size(); i++) {
        vis->GetVideoDriver()->draw2DRectangle(
            irr::video::SColor(100, 255, 255, 0),
            irr::core::rect<s32>(mx + i * 4, sy + my - (int)(spfact * msolver_speed->GetDeltalambdaHistory()[i]),
                                 mx + (i + 1) * 4 - 2, sy + my),
            &mrect);
    }

    if (vis->GetDevice()->getGUIEnvironment()) {
        gui::IGUIFont* font = vis->GetDevice()->getGUIEnvironment()->getBuiltInFont();
        if (font) {
            char buffer[100];
            font->draw(L"Solver speed violation", irr::core::rect<s32>(mx + sx / 2 - 100, my, mx + sx, my + 10),
                       irr::video::SColor(200, 100, 0, 0));
            sprintf(buffer, "%g", sy / spfact);
            font->draw(irr::core::stringw(buffer).c_str(), irr::core::rect<s32>(mx, my, mx + 30, my + 10),
                       irr::video::SColor(200, 100, 0, 0));
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void drawChFunction(ChVisualSystemIrrlicht* vis,
                    ChFunction* fx,
                    double xmin,
                    double xmax,
                    double ymin,
                    double ymax,
                    int mx,
                    int my,
                    int sx,
                    int sy) {
    irr::video::IVideoDriver* driver = vis->GetDevice()->getVideoDriver();

    if (!fx)
        return;

    irr::core::rect<s32> mrect(mx, my, mx + sx, my + sy);
    driver->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);

    if (vis->GetDevice()->getGUIEnvironment()) {
        gui::IGUIFont* font = vis->GetDevice()->getGUIEnvironment()->getBuiltInFont();
        if (font) {
            char buffer[100];
            sprintf(buffer, "%g", ymax);
            font->draw(irr::core::stringw(buffer).c_str(), irr::core::rect<s32>(mx, my, mx + sx, my + 10),
                       irr::video::SColor(200, 100, 0, 0));
            sprintf(buffer, "%g", ymin);
            font->draw(irr::core::stringw(buffer).c_str(), irr::core::rect<s32>(mx, my + sy, mx + sx, my + sy + 10),
                       irr::video::SColor(200, 100, 0, 0));

            if ((ymin < 0) && (ymax > 0)) {
                int yzero = my + sy - (int)(((-ymin) / (ymax - ymin)) * (double)sy);
                driver->draw2DLine(irr::core::position2d<s32>(mx, yzero), irr::core::position2d<s32>(mx + sx, yzero),
                                   irr::video::SColor(90, 255, 255, 255));
                font->draw(irr::core::stringw(buffer).c_str(),
                           irr::core::rect<s32>(mx, my + yzero, mx + sx, my + yzero + 10),
                           irr::video::SColor(200, 100, 0, 0));
            }
        }
    }

    int prevx = 0;
    int prevy = 0;

    for (int ix = 0; ix < sx; ix++) {
        double x = xmin + (xmax - xmin) * ((double)(ix)) / (double)(sx);
        double y = fx->Get_y(x);
        int py = my + sy - (int)(((y - ymin) / (ymax - ymin)) * (double)sy);
        int px = mx + ix;
        if (ix > 0)
            driver->draw2DLine(irr::core::position2d<s32>(px, py), irr::core::position2d<s32>(prevx, prevy),
                               irr::video::SColor(200, 255, 0, 0));
        prevx = px;
        prevy = py;
    }
}

// -----------------------------------------------------------------------------
// Draw segment lines in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawSegment(ChVisualSystemIrrlicht* vis, ChVector<> start, ChVector<> end, ChColor col, bool use_Zbuffer) {
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
void drawPolyline(ChVisualSystemIrrlicht* vis, std::vector<ChVector<> >& points, ChColor col, bool use_Zbuffer) {
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
                ChColor col,
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
        phaseB = CH_C_2PI * (double)iu / (double)resolution;
        ChVector<> V1(radius * cos(phaseA), radius * sin(phaseA), 0);
        ChVector<> V2(radius * cos(phaseB), radius * sin(phaseB), 0);
        drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
        phaseA = phaseB;
    }
}

// -----------------------------------------------------------------------------
// Draw a spring in 3D space, with given color.
// -----------------------------------------------------------------------------
void drawSpring(ChVisualSystemIrrlicht* vis,
                double radius,
                ChVector<> start,
                ChVector<> end,
                ChColor col,
                int resolution,
                double turns,
                bool use_Zbuffer) {
    ChMatrix33<> rel_matrix;
    ChVector<> dist = end - start;
    ChVector<> Vx, Vy, Vz;
    double length = dist.Length();
    ChVector<> dir = Vnorm(dist);
    XdirToDxDyDz(dir, VECT_Y, Vx, Vy, Vz);
    rel_matrix.Set_A_axis(Vx, Vy, Vz);
    ChQuaternion<> Q12 = rel_matrix.Get_A_quaternion();
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
        phaseB = turns * CH_C_2PI * (double)iu / (double)resolution;
        heightB = length * ((double)iu / (double)resolution);
        ChVector<> V1(heightA, radius * cos(phaseA), radius * sin(phaseA));
        ChVector<> V2(heightB, radius * cos(phaseB), radius * sin(phaseB));
        drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
        phaseA = phaseB;
        heightA = heightB;
    }
}

// -----------------------------------------------------------------------------
// Draw a rotational spring in 3D space, with given color.
// -----------------------------------------------------------------------------
ChApiIrr void drawRotSpring(ChVisualSystemIrrlicht* vis,
                            ChCoordsys<> pos,
                            double radius,
                            double start_angle,
                            double end_angle,
                            ChColor col,
                            int resolution,
                            bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    double del_angle = (end_angle - start_angle) / resolution;
    ChVector<> V1(radius * std::cos(start_angle), radius * std::sin(start_angle), 0);

    for (int iu = 1; iu <= resolution; iu++) {
        double crt_angle = start_angle + iu * del_angle;
        double crt_radius = radius - (iu * del_angle / CH_C_2PI) * (radius / 10);
        ChVector<> V2(crt_radius * std::cos(crt_angle), crt_radius * std::sin(crt_angle), 0);
        drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
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
              ChColor col,
              bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    for (int iu = -nu / 2; iu <= nu / 2; iu++) {
        ChVector<> V1(iu * ustep, vstep * (nv / 2), 0);
        ChVector<> V2(iu * ustep, -vstep * (nv / 2), 0);
        drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
    }

    for (int iv = -nv / 2; iv <= nv / 2; iv++) {
        ChVector<> V1(ustep * (nu / 2), iv * vstep, 0);
        ChVector<> V2(-ustep * (nu / 2), iv * vstep, 0);
        drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
    }
}

/// Easy-to-use function to draw color map 2D legend
void drawColorbar(ChVisualSystemIrrlicht* vis,
                  double vmin,
                  double vmax,
                  const std::string& label,
                  int mx,
                  int my,
                  int sx,
                  int sy) {
    irr::video::IVideoDriver* driver = vis->GetVideoDriver();

    gui::IGUIFont* font = 0;
    if (vis->GetGUIEnvironment())
        font = vis->GetGUIEnvironment()->getSkin()->getFont();

    int steps = 10;
    double ystep = ((double)sy / (double)steps);
    for (int i = 0; i < steps; ++i) {
        double mv_up = vmax - (vmax - vmin) * ((double)(i) / (double)steps);
        double mv_dw = vmax - (vmax - vmin) * ((double)(i + 1) / (double)steps);
        core::rect<s32> mrect(mx, my + (s32)(i * ystep), mx + sx, my + (s32)((i + 1) * ystep));
        ChColor c_up = ChColor::ComputeFalseColor(mv_up, vmin, vmax, false);
        ChColor c_dw = ChColor::ComputeFalseColor(mv_dw, vmin, vmax, false);
        video::SColor col_up(255, u32(255 * c_up.R), u32(255 * c_up.G), u32(255 * c_up.B));
        video::SColor col_dw(255, u32(255 * c_dw.R), u32(255 * c_dw.G), u32(255 * c_dw.B));
        driver->draw2DRectangle(mrect, col_up, col_up, col_dw, col_dw);

        if (font) {
            char buffer[100];
            sprintf(buffer, "%g", mv_up);
            font->draw(irr::core::stringw(buffer).c_str(),
                       core::rect<s32>(mrect.UpperLeftCorner.X + sx + 6, mrect.UpperLeftCorner.Y - 5,
                                       mrect.LowerRightCorner.X + sx + 6, mrect.LowerRightCorner.Y - 5),
                       irr::video::SColor(255, 0, 0, 0));
            driver->draw2DLine(irr::core::position2d<s32>(mrect.UpperLeftCorner.X + sx - 4, mrect.UpperLeftCorner.Y),
                               irr::core::position2d<s32>(mrect.UpperLeftCorner.X + sx, mrect.UpperLeftCorner.Y),
                               irr::video::SColor(255, 100, 100, 100));
        }
    }
    font->draw(irr::core::stringw(label.c_str()).c_str(), core::rect<s32>(mx, my + sy + 5, mx + 100, my + sy + 20),
               irr::video::SColor(255, 0, 0, 0));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void drawPlot3D(ChVisualSystemIrrlicht* vis,
                ChMatrixConstRef X,
                ChMatrixConstRef Y,
                ChMatrixConstRef Z,
                ChCoordsys<> pos,
                ChColor col,
                bool use_Zbuffer) {
    vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    vis->GetVideoDriver()->setMaterial(mattransp);

    if ((X.cols() != Y.cols()) || (X.cols() != Z.cols()) || (X.rows() != Y.rows()) || (X.rows() != Z.rows())) {
        GetLog() << "drawPlot3D: X Y Z matrices must have the same size, as n.rows and n.columns \n";
        return;
    }

    for (int iy = 0; iy < X.cols(); ++iy) {
        for (int ix = 0; ix < X.rows(); ++ix) {
            if (ix > 0) {
                ChVector<> Vx1(X(ix - 1, iy), Y(ix - 1, iy), Z(ix - 1, iy));
                ChVector<> Vx2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(vis, pos.TransformLocalToParent(Vx1), pos.TransformLocalToParent(Vx2), col, use_Zbuffer);
            }

            if (iy > 0) {
                ChVector<> Vy1(X(ix, iy - 1), Y(ix, iy - 1), Z(ix, iy - 1));
                ChVector<> Vy2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(vis, pos.TransformLocalToParent(Vy1), pos.TransformLocalToParent(Vy2), col, use_Zbuffer);
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

        sprintf(buffer, "%d -- %s (%.2f %% parent, %.2f %% tot.) :: %.3f ms / frame (%d calls)\n", i,
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

    sprintf(buffer, "n -- %s (%.2f %% parent, %.2f %% tot.) :: %.3f ms\n", "Unaccounted:", fraction, fraction_tot,
            parent_time - accumulated_time);
    irr::core::stringw mstringu(buffer);
    font->draw(mstringu, irr::core::rect<irr::s32>(mx + xspacing, my + ypos, mx + sx, my + ypos + 20), mcol);
    ypos += 20;
}

// Draw run-time profiler infos
void drawProfiler(ChVisualSystemIrrlicht* vis) {
    int mx = 230;
    int my = 30;
    int sx = 500;
    int sy = 400;

    int ypos = 0;
    utils::ChProfileIterator* profileIterator = 0;
    profileIterator = chrono::utils::ChProfileManager::Get_Iterator();

    drawProfilerRecursive(profileIterator, vis->GetDevice(), mx, my, sx, sy, 0, ypos);

    utils::ChProfileManager::Release_Iterator(profileIterator);
}

// Draw RGB coordinate system
void drawCoordsys(ChVisualSystemIrrlicht* vis, const ChCoordsys<>& coord, double scale) {
    ChVector<> pos = coord.pos;
    ChQuaternion<> rot = coord.rot;
    // X axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_X) * scale, ChColor(1, 0, 0));
    // Y axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_Y) * scale, ChColor(0, 1, 0));
    // Z axis
    drawSegment(vis, pos, pos + rot.Rotate(VECT_Z) * scale, ChColor(0, 0, 1));
}

}  // end namespace tools
}  // end namespace irrlicht
}  // end namespace chrono
