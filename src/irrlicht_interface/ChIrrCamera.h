//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef __RTSCamera__ 
#define __RTSCamera__ 


//////////////////////////////////////////////////
//
//   ChIrrCamera.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Class to create an interactive videocamera
//   in Irrlicht, that is similar to the Maya camera
//   but hasn't the problems that the Maya camera 
//   has in Irrlicht 1.5.
//   The following code is based on a work by 
//   "CmdKewin" (from the Irrlicht forum).
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////




#include <irrlicht.h> 


namespace irr
{
namespace scene
{

class RTSCamera : public ICameraSceneNode 
{ 
   public: 
      RTSCamera(IrrlichtDevice* devicepointer,ISceneNode* parent,ISceneManager* smgr,s32 id, 
         f32 rotateSpeed = -1000.0f,f32 zoomSpeed = 100.0f, f32 translationSpeed = 300.0f); 

      virtual ~RTSCamera(); 

      //Events 
      virtual void render(); 
      virtual bool OnEvent(const SEvent& event); 
      virtual void OnRegisterSceneNode(); //old virtual void OnPreRender(); 
      virtual void OnAnimate(u32 timeMs);   //old virtual void OnPostRender(u32 timeMs); 

      //Setup 
      virtual void setInputReceiverEnabled(bool enabled); 
      virtual bool isInputReceiverEnabled() const; 

      //Gets 
	  virtual const core::aabbox3d<f32>& getBoundingBox() const; 
	  virtual const core::matrix4& getProjectionMatrix() const; 
	  virtual const SViewFrustum* getViewFrustum() const; 
	  virtual const core::matrix4& getViewMatrix() const; 
	  virtual void  setViewMatrixAffector(const core::matrix4& affector); //**ALEX
	  virtual const core::matrix4& getViewMatrixAffector() const ; //**ALEX
	  virtual const core::vector3df& getUpVector() const {return UpVector;}; 
	  virtual const core::vector3df& getTarget() const {return Target;} 
      virtual f32 getNearValue() const; 
      virtual f32 getFarValue() const ; 
      virtual f32 getAspectRatio() const; 
      virtual f32 getFOV() const; 

      //Sets 
      virtual void setNearValue(f32 zn); 
      virtual void setFarValue(f32 zf); 
      virtual void setAspectRatio(f32 aspect); 
      virtual void setFOV(f32 fovy); 
	  virtual void setUpVector(const core::vector3df& pos); 
	  virtual void setProjectionMatrix(const core::matrix4& projection, bool isOrthogonal = false); 
	  virtual void setPosition(const core::vector3df& newpos); 
	  virtual void setTarget(const core::vector3df& newpos); 
	   virtual void setRotation(const core::vector3df& rotation) {};

      virtual void setZoomSpeed(f32 value); 
      virtual void setTranslateSpeed(f32 value); 
      virtual void setRotationSpeed(f32 value); 

	  virtual void bindTargetAndRotation(bool bound) {};
	  virtual bool getTargetAndRotationBinding(void) const {return false;};

      //Helper Functions 
	  void pointCameraAtNode(ISceneNode* selectednode); 
      void setMinZoom(f32 amount); 
      void setMaxZoom(f32 amount); 

      //Type Return 
      virtual ESCENE_NODE_TYPE getType() const { return ESNT_CAMERA; } 

      //Public Attributes 
      bool atMinDistance; 
      bool atMaxDistance; 
      ISceneNode* selectednode; 
   protected: 
      //Properties 
	  core::vector3df Target; 
	  core::vector3df UpVector; 
	  core::matrix4 Projection; 
	  core::matrix4 View; 
	  core::matrix4 Affector; //**ALEX
      SViewFrustum ViewArea; 
	  core::aabbox3d<f32> BBox; 
      bool InputReceiverEnabled; 
	  core::dimension2d<f32> screenDim; 
      f32 Fovy;      //Field of view, in radians. 
      f32 Aspect;      //Aspect ratio. 
      f32 ZNear;      //Value of the near view-plane. 
      f32 ZFar;      //Z-value of the far view-plane. 

      void recalculateProjectionMatrix(); 
      void recalculateViewArea(); 

   private: 
      IrrlichtDevice* device; 
	  core::vector3df Pos; 
      bool zooming,rotating,moving,translating; 
      f32 zoomSpeed; 
      f32 translateSpeed; 
      f32 rotateSpeed; 
      f32 rotateStartX, rotateStartY; 
      f32 zoomStartX, zoomStartY; 
      f32 translateStartX, translateStartY; 
      f32 currentZoom; 
      f32 rotX, rotY; 
	  core::vector3df oldTarget; 
	  core::vector2df MousePos; 
      bool Keys[KEY_KEY_CODES_COUNT]; 
      bool MouseKeys[3]; 
      f32 targetMinDistance; 
      f32 targetMaxDistance; 

      enum MOUSE_BUTTON 
      { 
         MOUSE_BUTTON_LEFT, 
         MOUSE_BUTTON_MIDDLE, 
         MOUSE_BUTTON_RIGHT 
      }; 

      void allKeysUp(); 
      void allMouseKeysUp(); 
      bool isKeyDown(s32 key); 
      bool isMouseKeyDown(s32 key); 
      void animate(); 
      void updateAnimationState(); 
}; 


////////////////////////////////////

RTSCamera::RTSCamera(IrrlichtDevice* devicepointer,ISceneNode* parent,ISceneManager* smgr,s32 id, 
    f32 rs,f32 zs,f32 ts) 
	: ICameraSceneNode(parent,smgr,id,core::vector3df(1.0f,1.0f,1.0f),core::vector3df(0.0f,0.0f,0.0f), 
	core::vector3df(1.0f,1.0f,1.0f)),InputReceiverEnabled(true) 
{ 
   device = devicepointer; 
   BBox.reset(0,0,0); 

   UpVector.set(0.0f,1.0f,0.0f); 

   Fovy = core::PI / 2.5f; 
   Aspect = 4.0f / 3.0f; 
   ZNear = 1.0f; 
   ZFar = 3000.0f; 

   atMinDistance = false; 

   video::IVideoDriver* d = smgr->getVideoDriver(); 
   if (d) 
   { 
      screenDim.Width = (f32)d->getCurrentRenderTargetSize().Width; 
      screenDim.Height = (f32)d->getCurrentRenderTargetSize().Height; 
      Aspect = screenDim.Width / screenDim.Height; 
   } 

   zooming = false; 
   rotating = false; 
   moving = false; 
   translating = false; 
   zoomSpeed = zs; 
   rotateSpeed = rs; 
   translateSpeed = ts; 
   currentZoom = 100.0f; 
   targetMinDistance = 1.0f; 
   targetMaxDistance = 2000.0f; 
   Target.set(0.0f,0.0f,0.0f); 
   rotX = 0; 
   rotY = 0; 
   oldTarget = Target; 

   atMinDistance = false; 
   atMaxDistance = false; 

   allKeysUp(); 
   allMouseKeysUp(); 

   recalculateProjectionMatrix(); 
   recalculateViewArea(); 

   smgr->setActiveCamera(this); 
} 

RTSCamera::~RTSCamera() 
{ 
} 

bool RTSCamera::OnEvent(const SEvent& event) 
{ 
   if (!InputReceiverEnabled) 
      return false; 

   core::dimension2d<u32> ssize = SceneManager->getVideoDriver()->getScreenSize(); 

   if(event.EventType == EET_MOUSE_INPUT_EVENT) 
   { 
      switch(event.MouseInput.Event) 
      { 
         case EMIE_LMOUSE_PRESSED_DOWN: 
            //selectednode = SceneManager->getSceneCollisionManager()->getSceneNodeFromScreenCoordinatesBB(device->getCursorControl()->getPosition(),0xFF,false);
            
			//if(selectednode) 
            //   pointCameraAtNode(selectednode); 
            //else 
            { 
               selectednode = NULL; 
               MouseKeys[0] = true; 
            } 
            break; 
         case EMIE_RMOUSE_PRESSED_DOWN: 
            MouseKeys[2] = true; 
            break; 
         case EMIE_MMOUSE_PRESSED_DOWN: 
            MouseKeys[1] = true; 
            break; 
         case EMIE_LMOUSE_LEFT_UP: 
            MouseKeys[0] = false; 
            break; 
         case EMIE_RMOUSE_LEFT_UP: 
            MouseKeys[2] = false; 
            break; 
         case EMIE_MMOUSE_LEFT_UP: 
            MouseKeys[1] = false; 
            break; 
         case EMIE_MOUSE_MOVED: 
            MousePos.X = event.MouseInput.X / (f32)ssize.Width; 
            MousePos.Y = event.MouseInput.Y / (f32)ssize.Height; 
            break; 
         case EMIE_MOUSE_WHEEL: 
            currentZoom -= event.MouseInput.Wheel * 0.05f*zoomSpeed;  //***ALEX

            if (currentZoom <= targetMinDistance) 
               atMinDistance = true; 
            else if (currentZoom >= targetMaxDistance) 
               atMaxDistance = true; 
            else 
            { 
               atMinDistance = false; 
               atMaxDistance = false; 
            } 

            break; 
      } 
      return true; 
   } 

   if(event.EventType == EET_KEY_INPUT_EVENT) 
   { 
      Keys[event.KeyInput.Key] = event.KeyInput.PressedDown; 
      return true; 
   } 

   return false; 
} 

void RTSCamera::OnRegisterSceneNode() 
{ 
	video::IVideoDriver* driver = SceneManager->getVideoDriver(); 
   if (!driver) 
      return; 

   if (SceneManager->getActiveCamera() == this) 
   { 
      screenDim.Width = (f32)driver->getCurrentRenderTargetSize().Width; 
      screenDim.Height = (f32)driver->getCurrentRenderTargetSize().Height; 

	  driver->setTransform(video::ETS_PROJECTION,Projection); 

      //If UpVector and Vector to Target are the same, we have a problem. 
      //Correct it. 
	  core::vector3df pos = getAbsolutePosition(); 
	  core::vector3df tgtv = Target - pos; 
      tgtv.normalize(); 

	  core::vector3df up = UpVector; 
      up.normalize(); 

      f32 dp = tgtv.dotProduct(up); 
      if ((dp > -1.0001f && dp < -0.9999f) || (dp < 1.0001f && dp > 0.9999f)) 
         up.X += 1.0f; 

      View.buildCameraLookAtMatrixLH(pos,Target,up);  // Right hand camera: use "..RH" here and in the following 
      recalculateViewArea(); 

      SceneManager->registerNodeForRendering(this,ESNRP_CAMERA); 
   } 

   if (IsVisible) 
      ISceneNode::OnRegisterSceneNode(); 
} 

void RTSCamera::render() 
{ 
   video::IVideoDriver* driver = SceneManager->getVideoDriver(); 
   if (!driver) 
      return; 

   driver->setTransform(video::ETS_VIEW,View); 
} 

void RTSCamera::OnAnimate(u32 timeMs) 
{ 
   animate(); 

   ISceneNode::setPosition(Pos); 
   updateAbsolutePosition(); 

   //TODO Add Animators 
} 

void RTSCamera::setInputReceiverEnabled(bool enabled) 
{ 
   InputReceiverEnabled = enabled; 
} 

bool RTSCamera::isInputReceiverEnabled() const
{ 
   _IRR_IMPLEMENT_MANAGED_MARSHALLING_BUGFIX; 
   return InputReceiverEnabled; 
} 

const core::aabbox3d<f32>& RTSCamera::getBoundingBox() const 
{ 
   return BBox; 
} 

const core::matrix4& RTSCamera::getProjectionMatrix() const
{ 
   return Projection; 
} 

const SViewFrustum* RTSCamera::getViewFrustum() const 
{ 
   return &ViewArea; 
} 


const core::matrix4& RTSCamera::getViewMatrix() const
{ 
   return View; 
} 


void RTSCamera::setViewMatrixAffector(const core::matrix4& affector)
{
	Affector = affector;
}

const core::matrix4& RTSCamera::getViewMatrixAffector() const
{
	return Affector;
}


f32 RTSCamera::getNearValue() const
{ 
   return ZNear; 
} 

f32 RTSCamera::getFarValue() const 
{ 
   return ZFar; 
} 

f32 RTSCamera::getAspectRatio() const 
{ 
   return Aspect; 
} 

f32 RTSCamera::getFOV() const
{ 
   return Fovy; 
} 

void RTSCamera::setNearValue(f32 f) 
{ 
   ZNear = f; 
   recalculateProjectionMatrix(); 
} 

void RTSCamera::setFarValue(f32 f) 
{ 
   ZFar = f; 
   recalculateProjectionMatrix(); 
} 

void RTSCamera::setAspectRatio(f32 f) 
{ 
   Aspect = f; 
   recalculateProjectionMatrix(); 
} 

void RTSCamera::setFOV(f32 f) 
{ 
   Fovy = f; 
   recalculateProjectionMatrix(); 
} 

void RTSCamera::setUpVector(const core::vector3df& pos) 
{ 
   UpVector = pos; 
} 

void RTSCamera::setProjectionMatrix(const core::matrix4& projection, bool isOrthogonal) 
{ 
   Projection = projection; 
} 

void RTSCamera::setPosition(const core::vector3df& pos) 
{ 
   Pos = pos; 
   updateAnimationState(); 

   ISceneNode::setPosition(pos); 
} 

void RTSCamera::setTarget(const core::vector3df& pos) 
{ 
   Target = oldTarget = pos; 
   updateAnimationState(); 
} 

void RTSCamera::setZoomSpeed(f32 value) 
{ 
   zoomSpeed = value; 
} 

void RTSCamera::setTranslateSpeed(f32 value) 
{ 
   translateSpeed = value; 
} 

void RTSCamera::setRotationSpeed(f32 value) 
{ 
   rotateSpeed = value; 
} 

void RTSCamera::pointCameraAtNode(ISceneNode* selectednode) 
{ 
   core::vector3df totarget = getPosition() - getTarget(); 
   setPosition(selectednode->getPosition() + (totarget.normalize() * 100)); 
   setTarget(selectednode->getPosition()); 
   updateAnimationState(); 
} 

void RTSCamera::setMinZoom(f32 amount) 
{ 
   targetMinDistance = amount; 
} 

void RTSCamera::setMaxZoom(f32 amount) 
{ 
   targetMaxDistance = amount; 
} 

void RTSCamera::recalculateProjectionMatrix() 
{ 
   Projection.buildProjectionMatrixPerspectiveFovLH(Fovy,Aspect,ZNear,ZFar);   // Right hand camera: use "..RH" here and in the following
} 

void RTSCamera::recalculateViewArea() 
{ 
   core::matrix4 mat = Projection * View; 
   ViewArea = SViewFrustum(mat); 

   ViewArea.cameraPosition = getAbsolutePosition(); 
   ViewArea.recalculateBoundingBox(); 
} 

void RTSCamera::allKeysUp() 
{ 
   for(int i = 0;i < KEY_KEY_CODES_COUNT;i++) 
      Keys[i] = false; 
} 

void RTSCamera::allMouseKeysUp() 
{ 
   for (s32 i=0; i<3; ++i) 
      MouseKeys[i] = false; 
} 

bool RTSCamera::isKeyDown(s32 key) 
{ 
   return Keys[key]; 
} 

bool RTSCamera::isMouseKeyDown(s32 key) 
{ 
   return MouseKeys[key]; 
} 

void RTSCamera::animate() 
{ 
   //Rotation Vals 
   f32 nRotX = rotX; 
   f32 nRotY = rotY; 
   f32 nZoom = currentZoom; 

   //Translation Vals 
   core::vector3df translate(oldTarget); 
   core::vector3df tvectX = Pos - Target; 
   tvectX = tvectX.crossProduct(UpVector); 
   tvectX.normalize(); 

   //Zoom 
   if (isMouseKeyDown(MOUSE_BUTTON_RIGHT) && isMouseKeyDown(MOUSE_BUTTON_LEFT)) 
   { 
      if (!zooming) 
      { 
         zoomStartX = MousePos.X; 
         zoomStartY = MousePos.Y; 
         zooming = true; 
         nZoom = currentZoom; 
      } 
      else 
      { 
         f32 old = nZoom; 
         nZoom += (zoomStartX - MousePos.X) * zoomSpeed * 100; 

         if (nZoom < targetMinDistance) 
            nZoom = targetMinDistance; 
         else if (nZoom > targetMaxDistance) 
            nZoom = targetMaxDistance; 

         if (nZoom < 0) 
            nZoom = old; 
      } 
   } 
   else 
   { 
      if (zooming) 
      { 
         f32 old = currentZoom; 
         currentZoom = currentZoom + (zoomStartX - MousePos.X ) * zoomSpeed; 
         nZoom = currentZoom; 

         if (nZoom < 0) 
            nZoom = currentZoom = old; 
      } 
      zooming = false; 
   } 

   //Rotation 
   if (isMouseKeyDown(MOUSE_BUTTON_LEFT) && !zooming) 
   { 
      if (!rotating) 
      { 
         rotateStartX = MousePos.X; 
         rotateStartY = MousePos.Y; 
         rotating = true; 
         nRotX = rotX; 
         nRotY = rotY; 
      } 
      else 
      { 
         nRotX += (rotateStartX - MousePos.X) * rotateSpeed; 
         nRotY += (rotateStartY - MousePos.Y) * rotateSpeed; 
      } 
   } 
   else 
   { 
      if (rotating) 
      { 
         rotX = rotX + (rotateStartX - MousePos.X) * rotateSpeed; 
         rotY = rotY + (rotateStartY - MousePos.Y) * rotateSpeed; 
         nRotX = rotX; 
         nRotY = rotY; 
      } 

      rotating = false; 
   } 

   //Translate 
   if (isMouseKeyDown(MOUSE_BUTTON_RIGHT) && !zooming) 
   { 
      if (!translating) 
      { 
         translateStartX = MousePos.X; 
         translateStartY = MousePos.Y; 
         translating = true; 
      } 
      else 
      { 
         translate += tvectX * (translateStartX - MousePos.X) * translateSpeed; 
         translate.X += tvectX.Z * (translateStartY - MousePos.Y) * translateSpeed; 
         translate.Z -= tvectX.X * (translateStartY - MousePos.Y) * translateSpeed; 

         oldTarget = translate; 
      } 
   } 

   else if ( isKeyDown(KEY_UP) && !zooming) 
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df movevector = getPosition() - getTarget(); 
         movevector.Y = 0; 
         movevector.normalize(); 

         setPosition(getPosition() - movevector * translateSpeed); 
         setTarget(getTarget() - movevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   } 
   else if (  isKeyDown(KEY_DOWN) && !zooming) 
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df movevector = getPosition() - getTarget(); 
         movevector.Y = 0; 
         movevector.normalize(); 

         setPosition(getPosition() + movevector * translateSpeed); 
         setTarget(getTarget() + movevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   } 
   else if ( isKeyDown(KEY_LEFT) && !zooming) 
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df totargetvector = getPosition() - getTarget(); 
         totargetvector.normalize(); 
		 core::vector3df crossvector = totargetvector.crossProduct(getUpVector()); 
		 core::vector3df strafevector = crossvector.normalize(); 

         setPosition(getPosition() - strafevector * translateSpeed); 
         setTarget(getTarget() - strafevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   } 
   else if (  isKeyDown(KEY_RIGHT) && !zooming) 
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df totargetvector = getPosition() - getTarget(); 
         totargetvector.normalize(); 
		 core::vector3df crossvector = totargetvector.crossProduct(getUpVector()); 
		 core::vector3df strafevector = crossvector.normalize(); 

         setPosition(getPosition() + strafevector * translateSpeed); 
         setTarget(getTarget() + strafevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   } 
   else if (  isKeyDown(KEY_PRIOR) && !zooming) // ALE
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df movevector; 
         movevector.Y = 1; 

         setPosition(getPosition() + movevector * translateSpeed); 
         setTarget(getTarget() + movevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   } 
   else if (  isKeyDown(KEY_NEXT) && !zooming) // ALE
   { 
      if (!translating) 
         translating = true; 
      else 
      { 
		 core::vector3df movevector; 
         movevector.Y = -1; 

         setPosition(getPosition() + movevector * translateSpeed); 
         setTarget(getTarget() + movevector * translateSpeed); 
         updateAbsolutePosition(); 
      } 
   }

   //Set Position 
   Target = translate; 

   Pos.X = nZoom + Target.X; 
   Pos.Y = Target.Y; 
   Pos.Z = Target.Z; 

   Pos.rotateXYBy(nRotY,Target); 
   Pos.rotateXZBy(-nRotX,Target); 

   //Correct Rotation Error 
   UpVector.set(0,1,0); 
   UpVector.rotateXYBy(-nRotY, core::vector3df(0,0,0)); 
   UpVector.rotateXZBy(-nRotX+180.f, core::vector3df(0,0,0)); 

} 

void RTSCamera::updateAnimationState() 
{ 
   core::vector3df pos(Pos - Target); 

   // X rotation 
   core::vector2df vec2d(pos.X,pos.Z); 
   rotX = (f32)vec2d.getAngle(); 

   // Y rotation 
   pos.rotateXZBy(rotX,core::vector3df()); 
   vec2d.set(pos.X, pos.Y); 
   rotY = -(f32)vec2d.getAngle(); 

   // Zoom 
   currentZoom = (f32)Pos.getDistanceFrom(Target); 
} 



} // end namespace
} // end namespace


#endif 