/*
Author: Charles Ricchio

Contains a managment class for easy manipulation of the camera. ChOgreCameraManager doesn't actually manage any Ogre
camera objects,
but instead retains points in space and points to orient to in space for easy access for the actual camera object within
ChOgreApplication.
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreCamera.h"
#include <Ogre.h>
#include <core/ChQuaternion.h>
#include <core/ChVector.h>
#include <vector>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreCameraManager {
  public:
    ChOgreCameraManager(Ogre::SceneManager* SceneManager, Ogre::Viewport* Viewport);
    ~ChOgreCameraManager();

    virtual ChOgreCamera* createCamera(const std::string& Name = ("Camera" + std::to_string(g_CameraCount)));

    virtual ChOgreCamera* getCamera(unsigned int iterator);

    virtual ChOgreCamera* getCamera(const std::string& Name);

    virtual ChOgreCamera* operator[](unsigned int iterator);

    virtual ChOgreCamera* operator[](const std::string& Name);

    virtual void makeActive(ChOgreCamera* Camera);

  protected:
    std::vector<ChOgreCamera*> m_CameraList;
    Ogre::SceneManager* m_pSceneManager;
    Ogre::Viewport* m_pViewport;

    static unsigned int g_CameraCount;

  private:
};
}