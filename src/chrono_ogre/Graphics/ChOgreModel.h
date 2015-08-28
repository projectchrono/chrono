#pragma once

#include "ChOgreMeshBase.h"

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreModel {
  public:
    ChOgreModel();
    ChOgreModel(Ogre::SceneManager* SceneManager);
    ChOgreModel(Ogre::SceneManager* SceneManager, const std::string& MeshPath);
    ChOgreModel(const ChOgreModel& _other);
    ChOgreModel(ChOgreModel&& _other);
    ~ChOgreModel();

    ChOgreModel& operator=(const ChOgreModel& _other);
    ChOgreModel& operator=(ChOgreModel&& _other);

    void assignSceneManager(Ogre::SceneManager* SceneManager);
    void loadMesh(const std::string& MeshPath);
    void setMesh(const Ogre::MeshPtr& pMesh);

    Ogre::SceneManager* getSceneManager() { return m_pSceneManager; }
    Ogre::SceneNode* getSceneNode() { return m_pNode; }

    ChOgreMeshBase mesh;

  protected:
    Ogre::SceneManager* m_pSceneManager;
    Ogre::SceneNode* m_pNode;
};
}