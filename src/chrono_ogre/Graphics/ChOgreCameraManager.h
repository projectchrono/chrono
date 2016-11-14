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

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreCameraManager {
  public:
    ChOgreCameraManager(Ogre::SceneManager* SceneManager, Ogre::Viewport* Viewport);
    ~ChOgreCameraManager();

    ChOgreCamera* createCamera(const std::string& Name = ("Camera" + std::to_string(g_CameraCount)));

    ChOgreCamera* getCamera(unsigned int iterator);

    ChOgreCamera* getCamera(const std::string& Name);

    ChOgreCamera* operator[](unsigned int iterator);

    ChOgreCamera* operator[](const std::string& Name);

    void makeActive(ChOgreCamera* Camera);

  protected:
    std::vector<ChOgreCamera*> m_CameraList;
    Ogre::SceneManager* m_pSceneManager;
    Ogre::Viewport* m_pViewport;

    static unsigned int g_CameraCount;

  private:
};
}
}
