#include "ChOgreCamera.h"

namespace ChOgre {

ChOgreCamera::ChOgreCamera(Ogre::Camera* Camera) : m_pCamera(Camera) {}

ChOgreCamera::ChOgreCamera(const ChOgreCamera& rhs) {
    if (this != &rhs) {
        m_pCamera = rhs.m_pCamera;
    }
}

ChOgreCamera::ChOgreCamera(ChOgreCamera&& lhs) {
    if (this != &lhs) {
        m_pCamera = lhs.m_pCamera;
        lhs.m_pCamera = nullptr;
    }
}

ChOgreCamera::~ChOgreCamera() {}

ChOgreCamera& ChOgreCamera::operator=(const ChOgreCamera& rhs) {
    if (this != &rhs) {
        m_pCamera = rhs.m_pCamera;
    }

    return *this;
}

ChOgreCamera& ChOgreCamera::operator=(ChOgreCamera&& lhs) {
    if (this != &lhs) {
        m_pCamera = lhs.m_pCamera;
        lhs.m_pCamera = nullptr;
    }

    return *this;
}

void ChOgreCamera::lookAt(float x, float y, float z) {
    m_pCamera->lookAt(x, y, z);
}

void ChOgreCamera::lookAt(const chrono::ChVector<>& dir) {
    m_pCamera->lookAt((Ogre::Real)dir.x, (Ogre::Real)dir.y, (Ogre::Real)dir.z);
}

void ChOgreCamera::orient(float pitch, float yaw) {
    chrono::ChVector<float> vCircle(0, std::sin(pitch), std::cos(pitch));
    chrono::ChVector<float> hCircle(std::cos(yaw), 0, std::sin(yaw));
    chrono::ChVector<float> dir(vCircle + hCircle);
    dir.Normalize();
    m_pCamera->setDirection(dir.x, dir.y, dir.z);
}

void ChOgreCamera::setPosition(float x, float y, float z) {
    m_pCamera->setPosition(x, y, z);
}

void ChOgreCamera::setPosition(const chrono::ChVector<>& pos) {
    m_pCamera->setPosition(pos.x, pos.y, pos.z);
}
}