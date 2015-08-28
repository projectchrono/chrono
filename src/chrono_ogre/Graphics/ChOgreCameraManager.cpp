/*
Author: Charles Ricchio

All the defines for the ChOgreCameraManager
*/

#include "ChOgreCameraManager.h"

namespace ChOgre {

unsigned int ChOgreCameraManager::g_CameraCount = 0;

ChOgreCameraManager::ChOgreCameraManager(Ogre::SceneManager* SceneManager, Ogre::Viewport* Viewport) {
    m_pSceneManager = SceneManager;
    m_pViewport = Viewport;
}

ChOgreCameraManager::~ChOgreCameraManager() {
    for (unsigned int i = 0; i < m_CameraList.size(); i++) {
        if (m_CameraList[i]) {
            delete m_CameraList[i];
        }
    }
}

ChOgreCamera* ChOgreCameraManager::createCamera(const std::string& Name) {
    ChOgreCamera* l_pCamera = new ChOgreCamera(m_pSceneManager->createCamera(Name));
    m_CameraList.push_back(l_pCamera);

    g_CameraCount = m_CameraList.size();

    l_pCamera->getCamera()->setAspectRatio(
        (((float)(m_pViewport->getActualWidth())) / ((float)(m_pViewport->getActualHeight()))));
    l_pCamera->getCamera()->setNearClipDistance(5.0f);

    return l_pCamera;
}

ChOgreCamera* ChOgreCameraManager::getCamera(unsigned int iterator) {
    return m_CameraList[iterator];
}

ChOgreCamera* ChOgreCameraManager::getCamera(const std::string& Name) {
    for (unsigned int i = 0; i < m_CameraList.size(); i++) {
        if (m_CameraList[i]->getName() == Name) {
            return m_CameraList[i];
        }
    }
    return nullptr;
}

ChOgreCamera* ChOgreCameraManager::operator[](unsigned int iterator) {
    return getCamera(iterator);
}

ChOgreCamera* ChOgreCameraManager::operator[](const std::string& Name) {
    return getCamera(Name);
}

void ChOgreCameraManager::makeActive(ChOgreCamera* Camera) {
    m_pViewport->setCamera(Camera->getCamera());
}
}