//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "physics/ChLinkMate.h"
#include "physics/ChContactContainer.h"
#include "lcp/ChLcpIterativeSolver.h"

#include "unit_IRRLICHT/ChIrrTools.h"


namespace irr
{


// -----------------------------------------------------------------------------
// Function to align an Irrlicht object to a Chrono::Engine coordsys.
// -----------------------------------------------------------------------------
void ChIrrTools::alignIrrlichtNodeToChronoCsys(scene::ISceneNode*           mnode,
                                               const chrono::ChCoordsys<>&  mcoords)
{
  // Output: will be an Irrlicht 4x4 matrix
  core::matrix4 irrMat;

  // Get the rigid body actual rotation, as a 3x3 matrix [A]
  chrono::ChMatrix33<> chMat(mcoords.rot);

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
  irrMat[10]= (irr::f32)chMat.GetElementN(8);

  irrMat[12]= (irr::f32)mcoords.pos.x;
  irrMat[13]= (irr::f32)mcoords.pos.y;
  irrMat[14]= (irr::f32)mcoords.pos.z;

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
class _draw_reporter_class : public chrono::ChReportContactCallback
{
public:
  virtual bool ReportContactCallback(const chrono::ChVector<>& pA,
                                     const chrono::ChVector<>& pB,
                                     const chrono::ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const float& mfriction,
                                     const chrono::ChVector<>& react_forces,
                                     const chrono::ChVector<>& react_torques,
                                     chrono::collision::ChCollisionModel* modA,
                                     chrono::collision::ChCollisionModel* modB)
  {
    chrono::ChMatrix33<>& mplanecoord = const_cast<chrono::ChMatrix33<>&>(plane_coord);
    chrono::ChVector<> v1=pA;
    chrono::ChVector<> v2;
    chrono::ChVector<> vn = mplanecoord.Get_A_Xaxis();

    switch (drawtype) {
    case ChIrrTools::CONTACT_DISTANCES:
      v2=pB;
      break;
    case ChIrrTools::CONTACT_NORMALS:
      v2=pA + vn * clen;
      break;
    case ChIrrTools::CONTACT_FORCES_N:
      v2=pA + vn * clen * react_forces.x;
      break;
    case ChIrrTools::CONTACT_FORCES:
      v2=pA + (mplanecoord*(react_forces*clen));
      break;
    }

    video::SColor mcol;
    if (distance > 0.0) 
      mcol=video::SColor(200,20,255,0);  // green: non penetration
    else
      mcol=video::SColor(200,255,60,60); // red: penetration

    this->cdriver->draw3DLine(core::vector3dfCH(v1), core::vector3dfCH(v2), mcol);
    return true; // to continue scanning contacts
  }

  video::IVideoDriver*              cdriver;
  ChIrrTools::eCh_ContactsDrawMode  drawtype;
  double                            clen;
};


int ChIrrTools::drawAllContactPoints(chrono::ChSystem&    mphysicalSystem,
                                     video::IVideoDriver* driver,
                                     double               mlen,
                                     eCh_ContactsDrawMode drawtype)
{
  if (drawtype == CONTACT_NONE)
    return 0;

  if (mphysicalSystem.GetNcontacts() == 0)
    return 0;

  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer= false;
  mattransp.Lighting=false;
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
class _label_reporter_class : public chrono::ChReportContactCallback 
{
public:
  virtual bool ReportContactCallback(const chrono::ChVector<>& pA,
                                     const chrono::ChVector<>& pB,
                                     const chrono::ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const float& mfriction,
                                     const chrono::ChVector<>& react_forces,
                                     const chrono::ChVector<>& react_torques,
                                     chrono::collision::ChCollisionModel* modA,
                                     chrono::collision::ChCollisionModel* modB)
  {
    char buffer[25];
    core::vector3df mpos((irr::f32)pA.x, (irr::f32)pA.y, (irr::f32)pA.z);
    core::position2d<s32> spos = this->cdevice->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
    gui::IGUIFont* font = this->cdevice->getGUIEnvironment()->getBuiltInFont();

    switch (labeltype) {
    case ChIrrTools::CONTACT_DISTANCES_VAL:
      sprintf(buffer,"% 6.3g", distance );
      break;
    case ChIrrTools::CONTACT_FORCES_N_VAL:
      sprintf(buffer,"% 6.3g", react_forces.x);
      break;
    case ChIrrTools::CONTACT_FORCES_T_VAL:
      sprintf(buffer,"% 6.3g", chrono::ChVector<>(0,react_forces.y,react_forces.z).Length() );
      break;
    case ChIrrTools::CONTACT_FORCES_VAL:
      sprintf(buffer,"% 6.3g", chrono::ChVector<>(react_forces).Length() );
      break;
    case ChIrrTools::CONTACT_TORQUES_VAL:
      sprintf(buffer,"% 6.3g", chrono::ChVector<>(react_torques).Length() );
      break;
    case ChIrrTools::CONTACT_TORQUES_S_VAL:
      sprintf(buffer,"% 6.3g", react_torques.x);
      break;
    case ChIrrTools::CONTACT_TORQUES_R_VAL:
      sprintf(buffer,"% 6.3g",  chrono::ChVector<>(0,react_torques.y,react_torques.z).Length() ); 
      break;
    }

    font->draw(core::stringw(buffer).c_str(),
               core::rect<s32>(spos.X-15,spos.Y, spos.X+15, spos.Y+10),
               ccol);

    return true; // to continue scanning contacts
  }

  irr::IrrlichtDevice*              cdevice;
  ChIrrTools::eCh_ContactsLabelMode labeltype;
  video::SColor                     ccol;
};

int ChIrrTools::drawAllContactLabels(chrono::ChSystem&      mphysicalSystem,
                                     irr::IrrlichtDevice*   device,
                                     eCh_ContactsLabelMode  labeltype,
                                     video::SColor          mcol)
{ 
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
// Draw collision objects bounding boxes for rigid bodies.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllBoundingBoxes(chrono::ChSystem&    mphysicalSystem,
                                     video::IVideoDriver* driver)
{
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer = true;
  mattransp.Lighting = false;
  driver->setMaterial(mattransp);

  chrono::ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
  while (myiter.HasItem())
  {
    if ((*myiter).IsType<chrono::ChBody>()) //item is inherited from ChBody?
    {
      chrono::ChSharedPtr<chrono::ChBody> abody ((*myiter).DynamicCastTo<chrono::ChBody>());

      video::SColor mcol;

      if (abody->GetSleeping()) 
        mcol=video::SColor(70,0,50,255);  // blue: sleeping
      else
        mcol=video::SColor(70,30,200,200);  // cyan: not sleeping

      chrono::ChVector<> hi = chrono::VNULL;
      chrono::ChVector<> lo = chrono::VNULL;
      abody->GetTotalAABB(lo,hi);
      chrono::ChVector<> p1(hi.x,lo.y,lo.z);
      chrono::ChVector<> p2(lo.x,hi.y,lo.z);
      chrono::ChVector<> p3 (lo.x,lo.y,hi.z);
      chrono::ChVector<> p4 (hi.x,hi.y,lo.z);
      chrono::ChVector<> p5 (lo.x,hi.y,hi.z);
      chrono::ChVector<> p6 (hi.x,lo.y,hi.z);
      chrono::ChVector<> p7 (lo.x,lo.y,hi.z);
      chrono::ChVector<> p8 (lo.x,lo.y,hi.z);
      chrono::ChVector<> p9 (lo.x,hi.y,lo.z);
      chrono::ChVector<> p10(lo.x,hi.y,lo.z);
      chrono::ChVector<> p11(hi.x,lo.y,lo.z);
      chrono::ChVector<> p12(hi.x,lo.y,lo.z);
      chrono::ChVector<> p14(hi.x,lo.y,hi.z);
      chrono::ChVector<> p15(lo.x,hi.y,hi.z);
      chrono::ChVector<> p16(lo.x,hi.y,hi.z);
      chrono::ChVector<> p17(hi.x,hi.y,lo.z);
      chrono::ChVector<> p18(hi.x,lo.y,hi.z);
      chrono::ChVector<> p19(hi.x,hi.y,lo.z);
      driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(p1), mcol );
      driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(p2), mcol );
      driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(p3), mcol );
      driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(p4), mcol );
      driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(p5), mcol );
      driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(p6), mcol );
      driver->draw3DLine(core::vector3dfCH(p7), core::vector3dfCH(p14), mcol );
      driver->draw3DLine(core::vector3dfCH(p8), core::vector3dfCH(p15), mcol );
      driver->draw3DLine(core::vector3dfCH(p9), core::vector3dfCH(p16), mcol );
      driver->draw3DLine(core::vector3dfCH(p10), core::vector3dfCH(p17), mcol );
      driver->draw3DLine(core::vector3dfCH(p11), core::vector3dfCH(p18), mcol );
      driver->draw3DLine(core::vector3dfCH(p12), core::vector3dfCH(p19), mcol );
    }

    ++myiter;
  }

  return 0;
}


// -----------------------------------------------------------------------------
// Draw coordinate systems of ChBody objects bodies.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllCOGs(chrono::ChSystem&     mphysicalSystem,
                            video::IVideoDriver*  driver,
                            double                scale)
{ 
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer = true;
  mattransp.Lighting = false;
  driver->setMaterial(mattransp);

  chrono::ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
  while (myiter.HasItem())
  {
    if ((*myiter).IsType<chrono::ChBody>()) //item is inherited from ChBody?
    {
      chrono::ChSharedPtr<chrono::ChBody> abody ((*myiter).DynamicCastTo<chrono::ChBody>());

      video::SColor mcol;
      const chrono::ChFrame<>& mframe_cog = abody->GetFrame_COG_to_abs();
      const chrono::ChFrame<>& mframe_ref = abody->GetFrame_REF_to_abs();

      chrono::ChVector<> p0 = mframe_cog.GetPos();
      chrono::ChVector<> px = p0 + mframe_cog.GetA().Get_A_Xaxis()*0.5*scale;
      chrono::ChVector<> py = p0 + mframe_cog.GetA().Get_A_Yaxis()*0.5*scale;
      chrono::ChVector<> pz = p0 + mframe_cog.GetA().Get_A_Zaxis()*0.5*scale;

      mcol=video::SColor(70,125,0,0);  // X red
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(px), mcol );
      mcol=video::SColor(70,0,125,0);  // Y green
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(py), mcol );
      mcol=video::SColor(70,0,0,125);  // Z blue
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(pz), mcol );

      p0 = mframe_ref.GetPos();
      px = p0 + mframe_ref.GetA().Get_A_Xaxis()*scale;
      py = p0 + mframe_ref.GetA().Get_A_Yaxis()*scale;
      pz = p0 + mframe_ref.GetA().Get_A_Zaxis()*scale;

      mcol=video::SColor(70,255,0,0);  // X red
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(px), mcol );
      mcol=video::SColor(70,0,255,0);  // Y green
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(py), mcol );
      mcol=video::SColor(70,0,0,255);  // Z blue
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(pz), mcol );
    }

    ++myiter;
  }

  return 0;
}


// -----------------------------------------------------------------------------
// Draw coordinate systems of frames used by links.
// -----------------------------------------------------------------------------
int ChIrrTools::drawAllLinkframes(chrono::ChSystem&     mphysicalSystem,
                                  video::IVideoDriver*  driver,
                                  double                scale)
{
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer= true;
  mattransp.Lighting=false;
  driver->setMaterial(mattransp);

  chrono::ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
  while (myiter.HasItem())
  {
    if ((*myiter).IsType<chrono::ChLinkMarkers>() ||
        (*myiter).IsType<chrono::ChLinkMateGeneric>() ) //item is inherited from ChBody?
    {
      chrono::ChFrame<> frAabs;
      chrono::ChFrame<> frBabs;

      if ((*myiter).IsType<chrono::ChLinkMarkers>()) {
        chrono::ChSharedPtr<chrono::ChLinkMarkers> mylink ((*myiter).DynamicCastTo<chrono::ChLinkMarkers>());
        frAabs = *mylink->GetMarker1() >> *mylink->GetBody1();
        frBabs = *mylink->GetMarker2() >> *mylink->GetBody2();
      }

      if ((*myiter).IsType<chrono::ChLinkMateGeneric>()) {
        chrono::ChSharedPtr<chrono::ChLinkMateGeneric> mylink ((*myiter).DynamicCastTo<chrono::ChLinkMateGeneric>());
        frAabs = mylink->GetFrame1() >> *mylink->GetBody1();
        frBabs = mylink->GetFrame2() >> *mylink->GetBody2();
      }

      video::SColor mcol;

      chrono::ChVector<> p0 = frAabs.GetPos();
      chrono::ChVector<> px = p0 + frAabs.GetA()->Get_A_Xaxis()*0.7*scale;
      chrono::ChVector<> py = p0 + frAabs.GetA()->Get_A_Yaxis()*0.7*scale;
      chrono::ChVector<> pz = p0 + frAabs.GetA()->Get_A_Zaxis()*0.7*scale;

      mcol=video::SColor(70,125,0,0);  // X red
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(px), mcol );
      mcol=video::SColor(70,0,125,0);  // Y green
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(py), mcol );
      mcol=video::SColor(70,0,0,125);  // Z blue
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(pz), mcol );

      p0 = frBabs.GetPos();
      px = p0 + frBabs.GetA()->Get_A_Xaxis()*scale;
      py = p0 + frBabs.GetA()->Get_A_Yaxis()*scale;
      pz = p0 + frBabs.GetA()->Get_A_Zaxis()*scale;

      mcol=video::SColor(70,255,0,0);  // X red
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(px), mcol );
      mcol=video::SColor(70,0,255,0);  // Y green
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(py), mcol );
      mcol=video::SColor(70,0,0,255);  // Z blue
      driver->draw3DLine(core::vector3dfCH(p0), core::vector3dfCH(pz), mcol );
    }

    ++myiter;
  }
  return 0;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrTools::drawHUDviolation(video::IVideoDriver*  driver,
                                  IrrlichtDevice*       mdevice,
                                  chrono::ChSystem&     asystem,
                                  int                   mx,
                                  int                   my,
                                  int                   sx,
                                  int                   sy,
                                  double                spfact,
                                  double                posfact)
{
  if (asystem.GetLcpSolverType() == chrono::ChSystem::LCP_SIMPLEX)
    return;

  chrono::ChLcpIterativeSolver* msolver_speed = (chrono::ChLcpIterativeSolver*)asystem.GetLcpSolverSpeed();
  chrono::ChLcpIterativeSolver* msolver_stab  = (chrono::ChLcpIterativeSolver*)asystem.GetLcpSolverStab();
  msolver_speed->SetRecordViolation(true);
  msolver_stab->SetRecordViolation(true);

  core::rect<s32> mrect(mx, my, mx+sx, my+sy);
  driver->draw2DRectangle(video::SColor(100,200,200,230), mrect);
  for (unsigned int i=0; i<msolver_speed->GetViolationHistory().size(); i++) {
    driver->draw2DRectangle(video::SColor(90,255,0,0),
      core::rect<s32>(mx+i*4,       sy+my -(int)(spfact*msolver_speed->GetViolationHistory()[i]), 
      mx+(i+1)*4-1, sy+my),
      &mrect);
  }
  for (unsigned int i=0; i<msolver_speed->GetDeltalambdaHistory().size(); i++) {
    driver->draw2DRectangle(video::SColor(100,255,255,0),
      core::rect<s32>(mx+i*4,       sy+my -(int)(spfact*msolver_speed->GetDeltalambdaHistory()[i]), 
      mx+(i+1)*4-2, sy+my),
      &mrect);
  }
  for (unsigned int i=0; i<msolver_stab->GetViolationHistory().size(); i++) {
    driver->draw2DRectangle(video::SColor(90,0,255,0),
      core::rect<s32>(mx+sx/2+i*4,  sy+my -(int)(posfact*msolver_stab->GetViolationHistory()[i]), 
      mx+sx/2+(i+1)*4-1, sy+my),
      &mrect);
  }
  for (unsigned int i=0; i<msolver_stab->GetDeltalambdaHistory().size(); i++) {
    driver->draw2DRectangle(video::SColor(100,0,255,255),
      core::rect<s32>(mx+sx/2+i*4,   sy+my -(int)(posfact*msolver_stab->GetDeltalambdaHistory()[i]), 
      mx+sx/2+(i+1)*4-2, sy+my),
      &mrect);
  }

  if (mdevice->getGUIEnvironment()) {
    gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
    if (font) {
      char buffer[100];
      font->draw(L"LCP speed violation", core::rect<s32>(mx+sx/2-100,my, mx+sx, my+10), video::SColor(200,100,0,0));
      sprintf(buffer,"%g", sy/spfact); 
      font->draw(core::stringw(buffer).c_str(), core::rect<s32>(mx,my, mx+30, my+10), video::SColor(200,100,0,0));
      font->draw(L"LCP position violation", core::rect<s32>(mx+sx-100,my, mx+sx, my+10), video::SColor(200,0,100,0));
      sprintf(buffer,"%g", sy/posfact); 
      font->draw(core::stringw(buffer).c_str(), core::rect<s32>(mx+sx/2,my, mx+sx/2+10, my+10), video::SColor(200,0,100,0));
    }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrTools::drawChFunction(IrrlichtDevice*     mdevice,
                                chrono::ChFunction* fx,
                                double              xmin,
                                double              xmax,
                                double              ymin,
                                double              ymax,
                                int                 mx,
                                int                 my,
                                int                 sx,
                                int                 sy)
{
  irr::video::IVideoDriver* driver = mdevice->getVideoDriver();

  if (!fx) return;

  core::rect<s32> mrect(mx, my, mx+sx, my+sy);
  driver->draw2DRectangle(video::SColor(100,200,200,230), mrect);

  if (mdevice->getGUIEnvironment()) {
    gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
    if (font) {
      char buffer[100];
      sprintf(buffer,"%g",ymax);
      font->draw(core::stringw(buffer).c_str(), 
                 core::rect<s32>(mx,my, mx+sx, my+10),
                 video::SColor(200,100,0,0));
      sprintf(buffer,"%g",ymin);
      font->draw(core::stringw(buffer).c_str(), 
                 core::rect<s32>(mx,my+sy, mx+sx, my+sy+10),
                 video::SColor(200,100,0,0));

      if ((ymin <0 ) && (ymax>0)) {
        int yzero =  my + sy - (int) ((( -ymin)/(ymax-ymin)) * (double) sy );
        driver->draw2DLine(irr::core::position2d<s32>(mx,yzero), 
                           irr::core::position2d<s32>(mx+sx, yzero), 
                           video::SColor(90,255,255,255));
        font->draw(core::stringw(buffer).c_str(), 
                   core::rect<s32>(mx,my+yzero, mx+sx, my+yzero+10),
                   video::SColor(200,100,0,0));
      }
    }
  }

  int prevx = 0;
  int prevy = 0;

  for (int ix = 0; ix < sx; ix++) {
    double x = xmin + (xmax-xmin)*((double)(ix))/(double)(sx);
    double y = fx->Get_y(x);
    int py = my + sy - (int) (((y -ymin)/(ymax-ymin)) * (double) sy )  ;
    int px = mx+ix;
    if(ix>0)
      driver->draw2DLine(irr::core::position2d<s32>(px,py), 
                         irr::core::position2d<s32>(prevx, prevy), 
                         video::SColor(200,255,0,0));
    prevx = px;
    prevy=py;
  }

}


// -----------------------------------------------------------------------------
// Draw segment lines in 3D space, with given color.
// -----------------------------------------------------------------------------
void ChIrrTools::drawSegment(video::IVideoDriver* driver,
                             chrono::ChVector<>   mstart,
                             chrono::ChVector<>   mend,
                             video::SColor        mcol,
                             bool                 use_Zbuffer)
{ 
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer= use_Zbuffer;
  mattransp.Lighting=false;
  driver->setMaterial(mattransp);
  driver->draw3DLine(core::vector3dfCH(mstart), core::vector3dfCH(mend), mcol);
}


// -----------------------------------------------------------------------------
// Draw a polyline in 3D space, given the array of points as a std::vector.
// -----------------------------------------------------------------------------
void ChIrrTools::drawPolyline(video::IVideoDriver*              driver,
                              std::vector< chrono::ChVector<> > mpoints,
                              video::SColor                     mcol,
                              bool                              use_Zbuffer)
{ 
  // not very efficient, but enough as an example..
  if (mpoints.size()<2) return;

  for (unsigned int i =0; i<mpoints.size()-1; i++)
    drawSegment(driver, mpoints[i], mpoints[i+1], mcol, use_Zbuffer);
}


// -----------------------------------------------------------------------------
// Draw a circle line in 3D space, with given color.
// -----------------------------------------------------------------------------
void ChIrrTools::drawCircle(video::IVideoDriver*  driver,
                            double                radius,
                            chrono::ChCoordsys<>  mpos,
                            video::SColor         mcol,
                            int                   mresolution,
                            bool                  use_Zbuffer)
{ 
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer= false;
  mattransp.Lighting=false;
  driver->setMaterial(mattransp);

  double phaseA = 0;
  double phaseB = 0;

  for (int iu=1; iu<=mresolution; iu++) {
    phaseB= chrono::CH_C_2PI*(double)iu/(double)mresolution;
    chrono::ChVector<> V1(radius*cos(phaseA), radius*sin(phaseA), 0);
    chrono::ChVector<> V2(radius*cos(phaseB), radius*sin(phaseB), 0);
    drawSegment(driver,mpos.TrasformLocalToParent(V1), mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer );
    phaseA = phaseB;
  }
}


// -----------------------------------------------------------------------------
// Draw a spring in 3D space, with given color.
// -----------------------------------------------------------------------------
void ChIrrTools::drawSpring(video::IVideoDriver*  driver,
                            double                radius,
                            chrono::ChVector<>    start,
                            chrono::ChVector<>    end,
                            video::SColor         mcol,
                            int                   mresolution,
                            double                turns,
                            bool                  use_Zbuffer)
{ 
  chrono::ChMatrix33<> rel_matrix;
  chrono::ChVector<> dist = end-start;
  chrono::ChVector<> Vx,Vy,Vz;
  double length = dist.Length();
  chrono::ChVector<> dir = chrono::Vnorm(dist);
  chrono::ChVector<> singul(chrono::VECT_Y);
  chrono::XdirToDxDyDz(&dir, &singul, &Vx,  &Vy, &Vz);
  rel_matrix.Set_A_axis(Vx,Vy,Vz);
  chrono::ChQuaternion<> Q12 = rel_matrix.Get_A_quaternion();
  chrono::ChCoordsys<> mpos(start,Q12);

  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer= false;
  mattransp.Lighting=false;
  driver->setMaterial(mattransp);

  double phaseA  = 0;
  double phaseB  = 0;
  double heightA = 0;
  double heightB = 0;

  for (int iu=1; iu<=mresolution; iu++) {
    phaseB= turns*chrono::CH_C_2PI*(double)iu/(double)mresolution;
    heightB= length*((double)iu/(double)mresolution);
    chrono::ChVector<> V1(heightA, radius*cos(phaseA), radius*sin(phaseA));
    chrono::ChVector<> V2(heightB, radius*cos(phaseB), radius*sin(phaseB));
    drawSegment(driver,mpos.TrasformLocalToParent(V1), mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer );
    phaseA = phaseB;
    heightA = heightB;
  }
}


// -----------------------------------------------------------------------------
// Draw grids in 3D space, with given orientation, colour and spacing.
// -----------------------------------------------------------------------------
void ChIrrTools::drawGrid(video::IVideoDriver*  driver,
                          double                ustep,
                          double                vstep,
                          int                   nu,
                          int                   nv,
                          chrono::ChCoordsys<>  mpos,
                          video::SColor         mcol,
                          bool                  use_Zbuffer)
{ 
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer = true;
  mattransp.Lighting = false;
  driver->setMaterial(mattransp);

  for (int iu=-nu/2; iu<=nu/2; iu++) {
    chrono::ChVector<> V1(iu*ustep, vstep*(nv/2), 0);
    chrono::ChVector<> V2(iu*ustep,-vstep*(nv/2), 0);
    drawSegment(driver, mpos.TrasformLocalToParent(V1),mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer);
  }

  for (int iv=-nv/2; iv<=nv/2; iv++) {
    chrono::ChVector<> V1(ustep*(nu/2), iv*vstep, 0);
    chrono::ChVector<> V2(-ustep*(nu/2),iv*vstep, 0);
    drawSegment(driver, mpos.TrasformLocalToParent(V1),mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer);
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrTools::drawPlot3D(video::IVideoDriver*  driver,
                            chrono::ChMatrix<>    X,
                            chrono::ChMatrix<>    Y,
                            chrono::ChMatrix<>    Z,
                            chrono::ChCoordsys<>  mpos,
                            video::SColor         mcol,
                            bool                  use_Zbuffer)
{ 
  driver->setTransform(video::ETS_WORLD, core::matrix4());
  video::SMaterial mattransp;
  mattransp.ZBuffer = true;
  mattransp.Lighting = false;
  driver->setMaterial(mattransp);

  if ((X.GetColumns() != Y.GetColumns()) ||
      (X.GetColumns() != Z.GetColumns()) ||
      (X.GetRows() != Y.GetRows()) ||
      (X.GetRows() != Z.GetRows()) )
  { 
    chrono::GetLog() << "drawPlot3D: X Y Z matrices must have the same size, as n.rows and n.columns \n";
    return;
  }

  for (int iy = 0; iy < X.GetColumns(); ++iy) {
    for (int ix = 0; ix < X.GetRows(); ++ix) {
      if (ix >0) {
        chrono::ChVector<> Vx1(X(ix-1,iy),Y(ix-1,iy),Z(ix-1,iy));
        chrono::ChVector<> Vx2(X(ix  ,iy),Y(ix  ,iy),Z(ix  ,iy));
        drawSegment(driver, mpos.TrasformLocalToParent(Vx1),mpos.TrasformLocalToParent(Vx2), mcol, use_Zbuffer);
      }

      if (iy >0) {
        chrono::ChVector<> Vy1(X(ix,iy-1),Y(ix,iy-1),Z(ix,iy-1));
        chrono::ChVector<> Vy2(X(ix,iy  ),Y(ix,iy  ),Z(ix,iy  ));
        drawSegment(driver, mpos.TrasformLocalToParent(Vy1),mpos.TrasformLocalToParent(Vy2), mcol, use_Zbuffer);
      }
    }
  }
}



} // END_OF_NAMESPACE____


