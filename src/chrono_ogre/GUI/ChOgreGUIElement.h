#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <MYGUI/MyGUI.h>
#include <MYGUI/MyGUI_OgrePlatform.h>
#include <core/ChVector2.h>
#include <memory>
#include <string>

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIElement {
  public:
    ChOgreGUIElement();
    ChOgreGUIElement(MyGUI::Gui* GUI);
    ~ChOgreGUIElement();

    virtual void setName(std::string Name);
    virtual void setGUI(MyGUI::Gui* GUI);
    virtual void setPosition(const ChVector2<>& Position) = 0;
    virtual void setSize(const ChVector2<>& Size) = 0;
    virtual void setColor(float r, float g, float b) = 0;
    virtual void update(){};

    virtual std::string getName();
    virtual ChVector2<> getSize() = 0;
    virtual ChVector2<> getPosition() = 0;

  protected:
    std::string m_Name;
    MyGUI::Gui* m_pGUI;

  private:
};

typedef std::unique_ptr<ChOgreGUIElement> ChOgreGUIElementPtr;
}
}
