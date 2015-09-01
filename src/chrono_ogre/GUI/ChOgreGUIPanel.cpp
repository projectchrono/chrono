/*
Author: Charles Ricchio
*/

#include "ChOgreGUIPanel.h"
#include <OGRE\Ogre.h>
#include <OGRE\OgreScriptCompiler.h>

namespace ChOgre {

unsigned int ECGUIPanel::g_count = 0;

ECGUIPanel::ECGUIPanel(Ogre::Overlay* Overlay) : ECGUIElement(Overlay) {
    m_pPanel = static_cast<Ogre::OverlayContainer*>(
        Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "Panel" + std::to_string(g_count)));

    m_pOverlay->add2D(m_pPanel);

    g_count++;
}

ECGUIPanel::~ECGUIPanel() {
    m_pOverlay->remove2D(m_pPanel);
    Ogre::OverlayManager::getSingleton().destroyOverlayElement(m_pPanel);
    g_count--;
}

void ECGUIPanel::setName(std::string Name) {
    m_Name = Name;
}

void ECGUIPanel::setPosition(double x, double y) {
    m_pPanel->setPosition((Ogre::Real)x, (Ogre::Real)y);
}

void ECGUIPanel::setSize(double x, double y) {
    m_pPanel->setDimensions((Ogre::Real)x, (Ogre::Real)y);
}

void ECGUIPanel::setColor(double r, double g, double b) {
    Ogre::Real _r = (Ogre::Real)r;
    Ogre::Real _g = (Ogre::Real)g;
    Ogre::Real _b = (Ogre::Real)b;

    // if (!m_pPanel->getMaterial()->getTechnique(0)->getPass(0)->getTextureUnitState("white.png")) {
    m_pPanel->setMaterialName("BaseWhite");
    m_pPanel->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState("white.png");
    //}
    // Next, we see the most convoluted way of changing the color of a UI element ever in the history of graphical
    // application development
    m_pPanel->getMaterial()->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setColourOperationEx(
        Ogre::LayerBlendOperationEx::LBX_MODULATE, Ogre::LayerBlendSource::LBS_TEXTURE,
        Ogre::LayerBlendSource::LBS_MANUAL, Ogre::ColourValue(), Ogre::ColourValue(_r, _g, _b));
}

void ECGUIPanel::setDecal(std::string Path) {
    m_pPanel->setMaterialName("BaseWhite");
    m_pPanel->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState(Path);
    m_pPanel->getMaterial()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
}

chrono::ChVector<> ECGUIPanel::getPosition() {
    chrono::ChVector<> _ret((double)m_pPanel->getLeft(), (double)m_pPanel->getTop(), 0);
    return _ret;
}

chrono::ChVector<> ECGUIPanel::getSize() {
    chrono::ChVector<> _ret((double)m_pPanel->getWidth(), (double)m_pPanel->getHeight(), 0);
    return _ret;
}
}