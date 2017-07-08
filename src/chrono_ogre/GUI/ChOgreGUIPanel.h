/*
Author: Charles Ricchio
*/

#pragma once

#include "ChOgreGUIElement.h"
#include <core/ChVector.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>

namespace ChOgre {

class ECGUIPanel : public ECGUIElement {
  public:
    ECGUIPanel(Ogre::Overlay* Overlay);
    ~ECGUIPanel();

    virtual void setName(std::string Name);
    virtual void setPosition(double x, double y);
    virtual void setSize(double x, double y);
    virtual void setColor(double r, double g, double b);

    virtual void setDecal(std::string FilePath);

    chrono::ChVector<> getPosition();
    chrono::ChVector<> getSize();

  protected:
    Ogre::OverlayContainer* m_pPanel;

    static unsigned int g_count;

  private:
};
}