#include "ChOgreModel.h"

namespace ChOgre {

ChOgreModel::ChOgreModel() {
    m_pSceneManager = nullptr;
    m_pNode = nullptr;
}

ChOgreModel::ChOgreModel(Ogre::SceneManager* SceneManager) {
    assignSceneManager(SceneManager);
}

ChOgreModel::ChOgreModel(Ogre::SceneManager* SceneManager, const std::string& MeshPath) {
    assignSceneManager(SceneManager);
    loadMesh(MeshPath);
}

ChOgreModel::ChOgreModel(const ChOgreModel& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pNode = _other.m_pNode;
        mesh = _other.mesh;
    }
}

ChOgreModel::ChOgreModel(ChOgreModel&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pNode = std::move(_other.m_pNode);
        mesh = std::move(_other.mesh);
    }
}

ChOgreModel::~ChOgreModel() {
    if (m_pNode != nullptr) {
        // m_pSceneManager->getRootSceneNode()->removeChild(m_pNode.get());
    }
}

ChOgreModel& ChOgreModel::operator=(const ChOgreModel& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pNode = _other.m_pNode;
        mesh = _other.mesh;
    }

    return *this;
}

ChOgreModel& ChOgreModel::operator=(ChOgreModel&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pNode = std::move(_other.m_pNode);
        mesh = std::move(_other.mesh);
    }

    return *this;
}

void ChOgreModel::assignSceneManager(Ogre::SceneManager* SceneManager) {
    if (m_pNode != nullptr && m_pSceneManager != nullptr) {
        // m_pSceneManager->getRootSceneNode()->removeChild(m_pNode);
    }

    m_pSceneManager = SceneManager;

    m_pNode = m_pSceneManager->getRootSceneNode()->createChildSceneNode();

    mesh.assignSceneManager(SceneManager);
}

void ChOgreModel::loadMesh(const std::string& MeshPath) {
    mesh.loadMesh(MeshPath);

    m_pNode->attachObject(mesh.getEntity());
}

void ChOgreModel::setMesh(const Ogre::MeshPtr& pMesh) {
    mesh.setMesh(pMesh);

    m_pNode->attachObject(mesh.getEntity());
}
}