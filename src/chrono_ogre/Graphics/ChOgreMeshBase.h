#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <Ogre.h>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreMeshBase {
  public:
    ChOgreMeshBase();
    ChOgreMeshBase(Ogre::SceneManager* SceneManager);
    ChOgreMeshBase(Ogre::SceneManager* SceneManager, const std::string& MeshPath);
    ChOgreMeshBase(const ChOgreMeshBase& _other);
    ChOgreMeshBase(ChOgreMeshBase&& _other);
    ~ChOgreMeshBase();

    ChOgreMeshBase& operator=(const ChOgreMeshBase& _other);
    ChOgreMeshBase& operator=(ChOgreMeshBase&& _other);

    void assignSceneManager(Ogre::SceneManager* SceneManager);
    void loadMesh(const std::string& MeshPath);
    void setMesh(const Ogre::MeshPtr& pMesh);

    Ogre::SceneManager* getSceneManager() { return m_pSceneManager; }
    Ogre::Entity* getEntity() { return m_pEntity; }

  protected:
    // NOTE: Ogre hates it when you clean up for it, making it impossible to use shared pointers.
    Ogre::SceneManager* m_pSceneManager;
    Ogre::Entity* m_pEntity;
};
}