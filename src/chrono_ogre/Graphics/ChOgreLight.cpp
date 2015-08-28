#include "ChOgreLight.h"

namespace ChOgre {

ChOgreLight::ChOgreLight(Ogre::SceneManager* SceneManager) {
    m_pLight = SceneManager->createLight();
}

ChOgreLight::ChOgreLight(Ogre::SceneManager* SceneManager, const std::string& Name) {
    m_pLight = SceneManager->createLight(Name);
}

ChOgreLight::~ChOgreLight() {}

void ChOgreLight::setType(LightTypes Type) {
    m_pLight->setType(Ogre::Light::LightTypes(Type));
}

void ChOgreLight::setDiffuse(const chrono::ChColor& color) {
    m_pLight->setDiffuseColour(color.R, color.G, color.B);
}

void ChOgreLight::setDiffuse(float r, float g, float b) {
    m_pLight->setDiffuseColour(r, g, b);
}

void ChOgreLight::setSpecular(const chrono::ChColor& color) {
    m_pLight->setSpecularColour(color.R, color.G, color.B);
}

void ChOgreLight::setSpecular(float r, float g, float b) {
    m_pLight->setSpecularColour(r, g, b);
}

void ChOgreLight::setPosition(const chrono::ChVector<>& position) {
    m_pLight->setPosition(position.x, position.y, position.z);
}

void ChOgreLight::setPosition(float x, float y, float z) {
    m_pLight->setPosition(x, y, z);
}

void ChOgreLight::setDirection(const chrono::ChVector<>& direction) {
    m_pLight->setDirection(direction.x, direction.y, direction.z);
}

void ChOgreLight::setDirection(float x, float y, float z) {
    m_pLight->setDirection(x, y, z);
}

void ChOgreLight::pointAt(const chrono::ChVector<>& position) {
    m_pLight->setDirection(
        (Ogre::Vector3(position.x, position.y, position.z) - m_pLight->getPosition()).normalisedCopy());
}

void ChOgreLight::pointAt(float x, float y, float z) {
    m_pLight->setDirection((Ogre::Vector3(x, y, z) - m_pLight->getPosition()).normalisedCopy());
}

void ChOgreLight::setIntensity(float i) {
    m_pLight->setPowerScale(i);
}

ChOgreLight::LightTypes ChOgreLight::getType() {
    return ChOgreLight::LightTypes(m_pLight->getType());
}

chrono::ChColor ChOgreLight::getDiffuse() {
    return chrono::ChColor(m_pLight->getDiffuseColour().r, m_pLight->getDiffuseColour().g,
                              m_pLight->getDiffuseColour().b, 1.0f);
}

chrono::ChColor ChOgreLight::getSpecular() {
    return chrono::ChColor(m_pLight->getSpecularColour().r, m_pLight->getSpecularColour().g,
                              m_pLight->getSpecularColour().b, 1.0f);
}

chrono::ChVector<> ChOgreLight::getPosition() {
    return chrono::ChVector<>(m_pLight->getPosition().x, m_pLight->getPosition().y, m_pLight->getPosition().z);
}

chrono::ChVector<> ChOgreLight::getDirection() {
    return chrono::ChVector<>(m_pLight->getDirection().x, m_pLight->getDirection().y, m_pLight->getDirection().z);
}

float ChOgreLight::getIntensity() {
    return m_pLight->getPowerScale();
}

std::string ChOgreLight::getName() {
    return m_pLight->getName();
}
}