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

#ifndef CHIRRCAMERA_H
#define CHIRRCAMERA_H


#include <irrlicht.h> 

#include "unit_IRRLICHT/ChApiIrr.h"

namespace irr
{
namespace scene
{

/// Class to create an interactive videocamera in Irrlicht, that is similar to
/// the Maya camera but hasn't the problems that the Maya camera has in
/// Irrlicht 1.5.
/// This code is based on work by "CmdKewin" (from the Irrlicht forum).


class ChApiIrr RTSCamera : public ICameraSceneNode
{
public: 
  RTSCamera(IrrlichtDevice* devicepointer,
            ISceneNode*     parent,
            ISceneManager*  smgr,
            s32             id,
            f32             rotateSpeed = -160.0f,
            f32             zoomSpeed = 1.0f,
            f32             translationSpeed = 0.003f);

  virtual ~RTSCamera() {}

  //Events 
  virtual void render(); 
  virtual bool OnEvent(const SEvent& event); 
  virtual void OnRegisterSceneNode();
  virtual void OnAnimate(u32 timeMs);

  //Setup 
  virtual void setInputReceiverEnabled(bool enabled) {InputReceiverEnabled = enabled;}
  virtual bool isInputReceiverEnabled() const; 

  //Gets 
  virtual const core::aabbox3d<f32>& getBoundingBox() const   {return BBox;}
  virtual const core::matrix4& getProjectionMatrix() const    {return Projection;}
  virtual const SViewFrustum* getViewFrustum() const          {return &ViewArea;}
  virtual const core::matrix4& getViewMatrix() const          {return View;}
  virtual const core::matrix4& getViewMatrixAffector() const  {return Affector;}
  virtual const core::vector3df& getUpVector() const          {return UpVector;}
  virtual const core::vector3df& getTarget() const            {return Target;}
  virtual f32 getNearValue() const                            {return ZNear;}
  virtual f32 getFarValue() const                             {return ZFar;}
  virtual f32 getAspectRatio() const                          {return Aspect;}
  virtual f32 getFOV() const                                  {return Fovy;}

  //Sets 
  virtual void setNearValue(f32 zn); 
  virtual void setFarValue(f32 zf); 
  virtual void setAspectRatio(f32 aspect); 
  virtual void setFOV(f32 fovy); 
  virtual void setViewMatrixAffector(const core::matrix4& affector);
  virtual void setUpVector(const core::vector3df& pos); 
  virtual void setProjectionMatrix(const core::matrix4& projection, bool isOrthogonal = false);
  virtual void setPosition(const core::vector3df& newpos); 
  virtual void setTarget(const core::vector3df& newpos); 
  virtual void setRotation(const core::vector3df& rotation) {}

  virtual void setZoomSpeed(f32 value); 
  virtual void setTranslateSpeed(f32 value); 
  virtual void setRotationSpeed(f32 value); 

  virtual void bindTargetAndRotation(bool bound) {}
  virtual bool getTargetAndRotationBinding(void) const {return false;}

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
  f32 Fovy;         /// Field of view, in radians. 
  f32 Aspect;       /// Aspect ratio. 
  f32 ZNear;        /// Value of the near view-plane. 
  f32 ZFar;         /// Z-value of the far view-plane. 

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
  //Hammad: Adding this so that GCC and clang on osx do not complain
  void updateMatrices();
};


} // end namespace
} // end namespace


#endif 
