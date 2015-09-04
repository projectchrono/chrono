#include "ChOgreMeshBase.h"

namespace ChOgre {

ChOgreMeshBase::ChOgreMeshBase() {
    m_pEntity = nullptr;
    m_pSceneManager = nullptr;
}

ChOgreMeshBase::ChOgreMeshBase(Ogre::SceneManager* SceneManager) {
    assignSceneManager(SceneManager);
}

ChOgreMeshBase::ChOgreMeshBase(Ogre::SceneManager* SceneManager, const std::string& MeshPath) {
    assignSceneManager(SceneManager);
    loadMesh(MeshPath);
}

ChOgreMeshBase::ChOgreMeshBase(const ChOgreMeshBase& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pEntity = _other.m_pEntity;
    }
}

ChOgreMeshBase::ChOgreMeshBase(ChOgreMeshBase&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pEntity = std::move(_other.m_pEntity);
    }
}

ChOgreMeshBase::~ChOgreMeshBase() {
    if (m_pEntity != nullptr) {
        // m_pSceneManager->destroyEntity(m_pEntity.get());
    }
}

ChOgreMeshBase& ChOgreMeshBase::operator=(const ChOgreMeshBase& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pEntity = _other.m_pEntity;
    }

    return *this;
}

ChOgreMeshBase& ChOgreMeshBase::operator=(ChOgreMeshBase&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pEntity = std::move(_other.m_pEntity);
    }

    return *this;
}

void ChOgreMeshBase::assignSceneManager(Ogre::SceneManager* SceneManager) {
    m_pSceneManager = SceneManager;
}

void ChOgreMeshBase::loadMesh(const std::string& MeshPath) {
    m_pEntity = m_pSceneManager->createEntity(MeshPath);
}

void ChOgreMeshBase::setMesh(const Ogre::MeshPtr& pMesh) {
    m_pEntity = m_pSceneManager->createEntity(pMesh);
}
}