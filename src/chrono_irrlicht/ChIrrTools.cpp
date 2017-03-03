// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChColor.h"
#include "chrono_irrlicht/ChIrrTools.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

// -----------------------------------------------------------------------------
// Function to align an Irrlicht object to a Chrono::Engine coordsys.
// -----------------------------------------------------------------------------
void ChIrrTools::alignIrrlichtNodeToChronoCsys(scene::ISceneNode* mnode, const ChCoordsys<>& mcoords) {
    // Output: will be an Irrlicht 4x4 matrix
    irr::core::matrix4 irrMat;

    // Get the rigid body actual rotation, as a 3x3 matrix [A]
    ChMatrix33<> chMat(mcoords.rot);

    // Fill the upper 3x3 submatrix with the [A] matrix
    // transposed, since Irrlicht uses the row-major style as in D3D
    irrMat[0] = (irr::f32)chMat.GetElementN(0);
    irrMat[1] = (irr::f32)chMat.GetElementN(3);
    irrMat[2] = (irr::f32)chMat.GetElementN(6);

    irrMat[4] = (irr::f32)chMat.GetElementN(1);
    irrMat[5] = (irr::f32)chMat.GetElementN(4);
    irrMat[6] = (irr::f32)chMat.GetElementN(7);

    irrMat[8] = (irr::f32)chMat.GetElementN(2);
    irrMat[9] = (irr::f32)chMat.GetElementN(5);
    irrMat[10] = (irr::f32)chMat.GetElementN(8);

    irrMat[12] = (irr::f32)mcoords.pos.x();
    irrMat[13] = (irr::f32)mcoords.pos.y();
    irrMat[14] = (irr::f32)mcoords.pos.z();

    // Clear the last column to 0 and set low-right corner to 1
    // as in Denavitt-Hartemberg matrices, transposed.
    irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
    irrMat[15] = 1.0f;

    // Set position and rotation of node using the 4x4 Irrlicht matrix.
    mnode->setPosition(irrMat.getTranslation());
    mnode->setRotation(irrMat.getRotationDegrees());
}

// -----------------------------------------------------------------------------
// Draw contact points.
// Uses the _draw_reporter_class callback class.
// -----------------------------------------------------------------------------
class _draw_reporter_class : public ChReportContactCallback {
  public:
    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
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
            case ChIrrTools::CONTACT_DISTANCES:
                v2 = pB;
                if (distance > 0.0)
                    mcol = irr::video::SColor(200, 20, 255, 0);  // green: non penetration
                else
                    mcol = irr::video::SColor(200, 255, 60, 60);  // red: penetration
                break;
            case ChIrrTools::CONTACT_NORMALS:
                v2 = pA + vn * clen;
                mcol = irr::video::SColor(200, 0, 100, 255);
                break;
            case ChIrrTools::CONTACT_FORCES_N:
                v2 = pA + vn * clen * react_forces.x();
                break;
            case ChIrrTools::CONTACT_FORCES:
                v2 = pA + (mplanecoord * (react_forces * clen));
                break;
            default:
                break;
        }

        this->cdriver->draw3DLine(irr::core::vector3dfCH(v1), irr::core::vector3dfCH(v2), mcol);
        return true;  // to continue scanning contacts
    }

    irr::video::IVideoDriver* cdriver;
    ChIrrTools::eCh_ContactsDrawMode drawtype;
    double clen;
};

int ChIrrTools::drawAllContactPoints(ChSystem& mphysicalSystem,
                                     irr::video::IVideoDriver* driver,
                                     double mlen,
                                     eCh_ContactsDrawMode drawtype) {
    if (drawtype == CONTACT_NONE)
        return 0;

    if (mphysicalSystem.GetNcontacts() == 0)
        return 0;

    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    _draw_reporter_class my_drawer;

    my_drawer.cdriver = driver;
    my_drawer.clen = mlen;
    my_drawer.drawtype = drawtype;

    // scan all contacts
    mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_drawer);

    return 0;
}

// -----------------------------------------------------------------------------
// Draw contact information as labels at the contact point.
// Uses the _label_reporter_class callback class.
// -----------------------------------------------------------------------------
class _label_reporter_class : public ChReportContactCallback {
  public:
    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
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
            case ChIrrTools::CONTACT_DISTANCES_VAL:
                sprintf(buffer, "% 6.3g", distance);
                break;
            case ChIrrTools::CONTACT_FORCES_N_VAL:
                sprintf(buffer, "% 6.3g", react_forces.x());
                break;
            case ChIrrTools::CONTACT_FORCES_T_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(0, react_forces.y(), react_forces.z()).Length());
                break;
            case ChIrrTools::CONTACT_FORCES_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(react_forces).Length());
                break;
            case ChIrrTools::CONTACT_TORQUES_VAL:
                sprintf(buffer, "% 6.3g", ChVector<>(react_torques).Length());
                break;
            case ChIrrTools::CONTACT_TORQUES_S_VAL:
                sprintf(buffer, "% 6.3g", react_torques.x());
                break;
            case ChIrrTools::CONTACT_TORQUES_R_VAL:
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
    ChIrrTools::eCh_ContactsLabelMode labeltype;
    irr::video::SColor ccol;
};

int ChIrrTools::drawAllContactLabels(ChSystem& mphysicalSystem,
                                     irr::IrrlichtDevice* device,
                                     eCh_ContactsLabelMode labeltype,
                                     irr::video::SColor mcol) {
    if (labeltype == CONTACT_NONE_VAL)
        return 0;

    if (mphysicalSystem.GetNcontacts() == 0)
        return 0;

    _label_reporter_class my_label_rep;

    my_label_rep.cdevice = device;
    my_label_rep.ccol = mcol;
    my_label_rep.labeltype = labeltype;

    // scan all contacts
    mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_label_rep);

    return 0;
}

// -----------------------------------------------------------------------------
// Draw links as glyps.
// ---------------------------------------------------------------------------
int ChIrrTools::drawAllLinks(ChSystem& mphysicalSystem,
                             irr::video::IVideoDriver* driver,
                             double mlen,
                             eCh_LinkDrawMode drawtype) {
    if (drawtype == LINK_NONE)
        return 0;

    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        if (auto mylink = std::dynamic_pointer_cast<ChLinkBase>(*myiter)) {
            ChCoordsys<> mlinkframe = mylink->GetLinkAbsoluteCoords();
            ChVector<> v1abs = mlinkframe.pos;
            ChVector<> v2;
            ChVector<> v2abs;
            switch (drawtype) {
                case ChIrrTools::LINK_REACT_FORCE:
                    v2 = mylink->Get_react_force();
                    break;
                case ChIrrTools::LINK_REACT_TORQUE:
                    v2 = mylink->Get_react_torque();
                    break;
                default:
                    break;
            }

            v2 *= mlen;
            v2abs = v2 >> mlinkframe;

            irr::video::SColor mcol(200, 250, 250, 0);  // yellow vectors

            driver->draw3DLine(irr::core::vector3dfCH(v1abs), irr::core::vector3dfCH(v2abs), mcol);
        }
        ++myiter;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// Draw links as labels
// ---------------------------------------------------------------------------
int ChIrrTools::drawAllLinkLabels(ChSystem& mphysicalSystem,
                                  irr::IrrlichtDevice* device,
                                  eCh_LinkLabelMode labeltype,
                                  irr::video::SColor mcol) {
    if (labeltype == LINK_NONE_VAL)
        return 0;

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        if (auto mylink = std::dynamic_pointer_cast<ChLinkBase>(*myiter)) {
            ChCoordsys<> mlinkframe = mylink->GetLinkAbsoluteCoords();  // GetAssetsFrame();

            char buffer[25];
            irr::core::vector3df mpos((irr::f32)mlinkframe.pos.x(), (irr::f32)mlinkframe.pos.y(),
                                      (irr::f32)mlinkframe.pos.z());
            irr::core::position2d<s32> spos =
                device->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
            gui::IGUIFont* font = device->getGUIEnvironment()->getBuiltInFont();

            switch (labeltype) {
                case ChIrrTools::LINK_REACT_FORCE_VAL:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_force().Length());
                    break;
                case ChIrrTools::LINK_REACT_FORCE_X:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_force().x());
                    break;
                case ChIrrTools::LINK_REACT_FORCE_Y:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_force().y());
                    break;
                case ChIrrTools::LINK_REACT_FORCE_Z:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_force().z());
                    break;
                case ChIrrTools::LINK_REACT_TORQUE_VAL:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_torque().Length());
                    break;
                case ChIrrTools::LINK_REACT_TORQUE_X:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_torque().x());
                    break;
                case ChIrrTools::LINK_REACT_TORQUE_Y:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_torque().y());
                    break;
                case ChIrrTools::LINK_REACT_TORQUE_Z:
                    sprintf(buffer, "% 6.3g", mylink->Get_react_torque().z());
                    break;
                default:
                    break;
            }

            font->draw(irr::core::stringw(buffer).c_str(),
                       irr::core::rect<s32>(spos.X - 15, spos.Y, spos.X + 15, spos.Y + 10), mcol);
        }
        ++myiter;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// Draw collision objects bounding boxes for rigid bodies.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllBoundingBoxes(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        if (auto abody = std::dynamic_pointer_cast<ChBody>(*myiter)) {
            irr::video::SColor mcol;

            if (abody->GetSleeping())
                mcol = irr::video::SColor(70, 0, 50, 255);  // blue: sleeping
            else
                mcol = irr::video::SColor(70, 30, 200, 200);  // cyan: not sleeping

            ChVector<> hi = VNULL;
            ChVector<> lo = VNULL;
            abody->GetTotalAABB(lo, hi);
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
            driver->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p1), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p2), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(lo), irr::core::vector3dfCH(p3), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p4), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p5), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(hi), irr::core::vector3dfCH(p6), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p7), irr::core::vector3dfCH(p14), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p8), irr::core::vector3dfCH(p15), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p9), irr::core::vector3dfCH(p16), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p10), irr::core::vector3dfCH(p17), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p11), irr::core::vector3dfCH(p18), mcol);
            driver->draw3DLine(irr::core::vector3dfCH(p12), irr::core::vector3dfCH(p19), mcol);
        }

        ++myiter;
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw coordinate systems of ChBody objects bodies.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllCOGs(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver, double scale) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        if (auto abody = std::dynamic_pointer_cast<ChBody>(*myiter)) {
            irr::video::SColor mcol;
            const ChFrame<>& mframe_cog = abody->GetFrame_COG_to_abs();
            const ChFrame<>& mframe_ref = abody->GetFrame_REF_to_abs();

            ChVector<> p0 = mframe_cog.GetPos();
            ChVector<> px = p0 + mframe_cog.GetA().Get_A_Xaxis() * 0.5 * scale;
            ChVector<> py = p0 + mframe_cog.GetA().Get_A_Yaxis() * 0.5 * scale;
            ChVector<> pz = p0 + mframe_cog.GetA().Get_A_Zaxis() * 0.5 * scale;

            mcol = irr::video::SColor(70, 125, 0, 0);  // X red
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
            mcol = irr::video::SColor(70, 0, 125, 0);  // Y green
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
            mcol = irr::video::SColor(70, 0, 0, 125);  // Z blue
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);

            p0 = mframe_ref.GetPos();
            px = p0 + mframe_ref.GetA().Get_A_Xaxis() * scale;
            py = p0 + mframe_ref.GetA().Get_A_Yaxis() * scale;
            pz = p0 + mframe_ref.GetA().Get_A_Zaxis() * scale;

            mcol = irr::video::SColor(70, 255, 0, 0);  // X red
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
            mcol = irr::video::SColor(70, 0, 255, 0);  // Y green
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
            mcol = irr::video::SColor(70, 0, 0, 255);  // Z blue
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);
        }

        ++myiter;
    }

    return 0;
}

// -----------------------------------------------------------------------------
// Draw coordinate systems of frames used by links.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllLinkframes(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver, double scale) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        if (auto mylinkbase = std::dynamic_pointer_cast<ChLinkBase>(*myiter)) {
            ChFrame<> frAabs;
            ChFrame<> frBabs;

            // default frame alignment:

            frAabs = mylinkbase->GetAssetsFrame();
            frBabs = frAabs;

            // special cases:

            if (auto mylink = std::dynamic_pointer_cast<ChLinkMarkers>(*myiter)) {
                frAabs = *mylink->GetMarker1() >> *mylink->GetBody1();
                frBabs = *mylink->GetMarker2() >> *mylink->GetBody2();
            }

            if (auto mylink = std::dynamic_pointer_cast<ChLinkMateGeneric>(*myiter)) {
                frAabs = mylink->GetFrame1() >> *mylink->GetBody1();
                frBabs = mylink->GetFrame2() >> *mylink->GetBody2();
            }

            irr::video::SColor mcol;

            ChVector<> p0 = frAabs.GetPos();
            ChVector<> px = p0 + frAabs.GetA().Get_A_Xaxis() * 0.7 * scale;
            ChVector<> py = p0 + frAabs.GetA().Get_A_Yaxis() * 0.7 * scale;
            ChVector<> pz = p0 + frAabs.GetA().Get_A_Zaxis() * 0.7 * scale;

            mcol = irr::video::SColor(70, 125, 0, 0);  // X red
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
            mcol = irr::video::SColor(70, 0, 125, 0);  // Y green
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
            mcol = irr::video::SColor(70, 0, 0, 125);  // Z blue
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);

            p0 = frBabs.GetPos();
            px = p0 + frBabs.GetA().Get_A_Xaxis() * scale;
            py = p0 + frBabs.GetA().Get_A_Yaxis() * scale;
            pz = p0 + frBabs.GetA().Get_A_Zaxis() * scale;

            mcol = irr::video::SColor(70, 255, 0, 0);  // X red
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(px), mcol);
            mcol = irr::video::SColor(70, 0, 255, 0);  // Y green
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(py), mcol);
            mcol = irr::video::SColor(70, 0, 0, 255);  // Z blue
            driver->draw3DLine(irr::core::vector3dfCH(p0), irr::core::vector3dfCH(pz), mcol);
        }

        ++myiter;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrTools::drawHUDviolation(irr::video::IVideoDriver* driver,
                                  IrrlichtDevice* mdevice,
                                  ChSystem& asystem,
                                  int mx,
                                  int my,
                                  int sx,
                                  int sy,
                                  double spfact,
                                  double posfact) {
    if (!std::dynamic_pointer_cast<ChIterativeSolver>(asystem.GetSolver()))
        return;

    auto msolver_speed = std::static_pointer_cast<ChIterativeSolver>(asystem.GetSolver());
    auto msolver_stab = std::static_pointer_cast<ChIterativeSolver>(asystem.GetStabSolver());
    msolver_speed->SetRecordViolation(true);
    msolver_stab->SetRecordViolation(true);

    irr::core::rect<s32> mrect(mx, my, mx + sx, my + sy);
    driver->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);
    for (unsigned int i = 0; i < msolver_speed->GetViolationHistory().size(); i++) {
        driver->draw2DRectangle(
            irr::video::SColor(90, 255, 0, 0),
            irr::core::rect<s32>(mx + i * 4, sy + my - (int)(spfact * msolver_speed->GetViolationHistory()[i]),
                                 mx + (i + 1) * 4 - 1, sy + my),
            &mrect);
    }
    for (unsigned int i = 0; i < msolver_speed->GetDeltalambdaHistory().size(); i++) {
        driver->draw2DRectangle(
            irr::video::SColor(100, 255, 255, 0),
            irr::core::rect<s32>(mx + i * 4, sy + my - (int)(spfact * msolver_speed->GetDeltalambdaHistory()[i]),
                                 mx + (i + 1) * 4 - 2, sy + my),
            &mrect);
    }
    for (unsigned int i = 0; i < msolver_stab->GetViolationHistory().size(); i++) {
        driver->draw2DRectangle(
            irr::video::SColor(90, 0, 255, 0),
            irr::core::rect<s32>(mx + sx / 2 + i * 4, sy + my - (int)(posfact * msolver_stab->GetViolationHistory()[i]),
                                 mx + sx / 2 + (i + 1) * 4 - 1, sy + my),
            &mrect);
    }
    for (unsigned int i = 0; i < msolver_stab->GetDeltalambdaHistory().size(); i++) {
        driver->draw2DRectangle(
            irr::video::SColor(100, 0, 255, 255),
            irr::core::rect<s32>(mx + sx / 2 + i * 4,
                                 sy + my - (int)(posfact * msolver_stab->GetDeltalambdaHistory()[i]),
                                 mx + sx / 2 + (i + 1) * 4 - 2, sy + my),
            &mrect);
    }

    if (mdevice->getGUIEnvironment()) {
        gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
        if (font) {
            char buffer[100];
            font->draw(L"Solver speed violation", irr::core::rect<s32>(mx + sx / 2 - 100, my, mx + sx, my + 10),
                       irr::video::SColor(200, 100, 0, 0));
            sprintf(buffer, "%g", sy / spfact);
            font->draw(irr::core::stringw(buffer).c_str(), irr::core::rect<s32>(mx, my, mx + 30, my + 10),
                       irr::video::SColor(200, 100, 0, 0));
            font->draw(L"Solver position violation", irr::core::rect<s32>(mx + sx - 100, my, mx + sx, my + 10),
                       irr::video::SColor(200, 0, 100, 0));
            sprintf(buffer, "%g", sy / posfact);
            font->draw(irr::core::stringw(buffer).c_str(),
                       irr::core::rect<s32>(mx + sx / 2, my, mx + sx / 2 + 10, my + 10),
                       irr::video::SColor(200, 0, 100, 0));
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrTools::drawChFunction(IrrlichtDevice* mdevice,
                                ChFunction* fx,
                                double xmin,
                                double xmax,
                                double ymin,
                                double ymax,
                                int mx,
                                int my,
                                int sx,
                                int sy) {
    irr::video::IVideoDriver* driver = mdevice->getVideoDriver();

    if (!fx)
        return;

    irr::core::rect<s32> mrect(mx, my, mx + sx, my + sy);
    driver->draw2DRectangle(irr::video::SColor(100, 200, 200, 230), mrect);

    if (mdevice->getGUIEnvironment()) {
        gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
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
void ChIrrTools::drawSegment(irr::video::IVideoDriver* driver,
                             ChVector<> mstart,
                             ChVector<> mend,
                             irr::video::SColor mcol,
                             bool use_Zbuffer) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = use_Zbuffer;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);
    driver->draw3DLine(irr::core::vector3dfCH(mstart), irr::core::vector3dfCH(mend), mcol);
}

// -----------------------------------------------------------------------------
// Draw a polyline in 3D space, given the array of points as a std::vector.
// -----------------------------------------------------------------------------
void ChIrrTools::drawPolyline(irr::video::IVideoDriver* driver,
                              std::vector<ChVector<> >& mpoints,
                              irr::video::SColor mcol,
                              bool use_Zbuffer) {
    // not very efficient, but enough as an example..
    if (mpoints.size() < 2)
        return;

    for (unsigned int i = 0; i < mpoints.size() - 1; i++)
        drawSegment(driver, mpoints[i], mpoints[i + 1], mcol, use_Zbuffer);
}

// -----------------------------------------------------------------------------
// Draw a circle line in 3D space, with given color.
// -----------------------------------------------------------------------------
void ChIrrTools::drawCircle(irr::video::IVideoDriver* driver,
                            double radius,
                            ChCoordsys<> mpos,
                            irr::video::SColor mcol,
                            int mresolution,
                            bool use_Zbuffer) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    double phaseA = 0;
    double phaseB = 0;

    for (int iu = 1; iu <= mresolution; iu++) {
        phaseB = CH_C_2PI * (double)iu / (double)mresolution;
        ChVector<> V1(radius * cos(phaseA), radius * sin(phaseA), 0);
        ChVector<> V2(radius * cos(phaseB), radius * sin(phaseB), 0);
        drawSegment(driver, mpos.TransformLocalToParent(V1), mpos.TransformLocalToParent(V2), mcol, use_Zbuffer);
        phaseA = phaseB;
    }
}

// -----------------------------------------------------------------------------
// Draw a spring in 3D space, with given color.
// -----------------------------------------------------------------------------
void ChIrrTools::drawSpring(irr::video::IVideoDriver* driver,
                            double radius,
                            ChVector<> start,
                            ChVector<> end,
                            irr::video::SColor mcol,
                            int mresolution,
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
    ChCoordsys<> mpos(start, Q12);

    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = false;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    double phaseA = 0;
    double phaseB = 0;
    double heightA = 0;
    double heightB = 0;

    for (int iu = 1; iu <= mresolution; iu++) {
        phaseB = turns * CH_C_2PI * (double)iu / (double)mresolution;
        heightB = length * ((double)iu / (double)mresolution);
        ChVector<> V1(heightA, radius * cos(phaseA), radius * sin(phaseA));
        ChVector<> V2(heightB, radius * cos(phaseB), radius * sin(phaseB));
        drawSegment(driver, mpos.TransformLocalToParent(V1), mpos.TransformLocalToParent(V2), mcol, use_Zbuffer);
        phaseA = phaseB;
        heightA = heightB;
    }
}

// -----------------------------------------------------------------------------
// Draw grids in 3D space, with given orientation, colour and spacing.
// -----------------------------------------------------------------------------
void ChIrrTools::drawGrid(irr::video::IVideoDriver* driver,
                          double ustep,
                          double vstep,
                          int nu,
                          int nv,
                          ChCoordsys<> mpos,
                          irr::video::SColor mcol,
                          bool use_Zbuffer) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    for (int iu = -nu / 2; iu <= nu / 2; iu++) {
        ChVector<> V1(iu * ustep, vstep * (nv / 2), 0);
        ChVector<> V2(iu * ustep, -vstep * (nv / 2), 0);
        drawSegment(driver, mpos.TransformLocalToParent(V1), mpos.TransformLocalToParent(V2), mcol, use_Zbuffer);
    }

    for (int iv = -nv / 2; iv <= nv / 2; iv++) {
        ChVector<> V1(ustep * (nu / 2), iv * vstep, 0);
        ChVector<> V2(-ustep * (nu / 2), iv * vstep, 0);
        drawSegment(driver, mpos.TransformLocalToParent(V1), mpos.TransformLocalToParent(V2), mcol, use_Zbuffer);
    }
}

/// Easy-to-use function to draw color map 2D legend
void ChIrrTools::drawColorbar(double vmin,
                              double vmax,
                              const std::string& label,
                              IrrlichtDevice* mdevice,
                              int mx,
                              int my,
                              int sx,
                              int sy) {
    irr::video::IVideoDriver* driver = mdevice->getVideoDriver();

    gui::IGUIFont* font = 0;
    if (mdevice->getGUIEnvironment())
        font = mdevice->getGUIEnvironment()->getSkin()->getFont();

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
void ChIrrTools::drawPlot3D(irr::video::IVideoDriver* driver,
                            ChMatrix<> X,
                            ChMatrix<> Y,
                            ChMatrix<> Z,
                            ChCoordsys<> mpos,
                            irr::video::SColor mcol,
                            bool use_Zbuffer) {
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    if ((X.GetColumns() != Y.GetColumns()) || (X.GetColumns() != Z.GetColumns()) || (X.GetRows() != Y.GetRows()) ||
        (X.GetRows() != Z.GetRows())) {
        GetLog() << "drawPlot3D: X Y Z matrices must have the same size, as n.rows and n.columns \n";
        return;
    }

    for (int iy = 0; iy < X.GetColumns(); ++iy) {
        for (int ix = 0; ix < X.GetRows(); ++ix) {
            if (ix > 0) {
                ChVector<> Vx1(X(ix - 1, iy), Y(ix - 1, iy), Z(ix - 1, iy));
                ChVector<> Vx2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(driver, mpos.TransformLocalToParent(Vx1), mpos.TransformLocalToParent(Vx2), mcol,
                            use_Zbuffer);
            }

            if (iy > 0) {
                ChVector<> Vy1(X(ix, iy - 1), Y(ix, iy - 1), Z(ix, iy - 1));
                ChVector<> Vy2(X(ix, iy), Y(ix, iy), Z(ix, iy));
                drawSegment(driver, mpos.TransformLocalToParent(Vy1), mpos.TransformLocalToParent(Vy2), mcol,
                            use_Zbuffer);
            }
        }
    }
}

}  // end namespace irrlicht
}  // end namespace chrono
