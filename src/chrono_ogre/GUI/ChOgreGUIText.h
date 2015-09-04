/*
Author: Charles Ricchio
*/

#pragma once

#include "ChOgreGUIElement.h"
#include <core/ChVector.h>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIText : public ChOgreGUIElement {
  public:
    ChOgreGUIText();
    ChOgreGUIText(MyGUI::Gui* GUI);
    ChOgreGUIText(const ChFloat3& Position, const ChFloat3& Size, MyGUI::Gui* GUI);
    ~ChOgreGUIText();

    virtual void setColor(float r, float g, float b);
    virtual void setTextColor(float r, float g, float b);
    virtual void setText(const std::string& Text);
    virtual void setFont(const std::string& Name);
    virtual void setPosition(const ChFloat3& Position);
    virtual void setSize(const ChFloat3& Size);

    virtual ChFloat3 getPosition() { return ChFloat3(m_pTextBox->getLeft(), m_pTextBox->getTop(), 0.f); };
    virtual ChFloat3 getSize() { return ChFloat3(m_pTextBox->getWidth(), m_pTextBox->getHeight(), 0.f); }

  protected:
    MyGUI::TextBox* m_pTextBox;

    static unsigned int g_count;

  private:
};

typedef std::unique_ptr<ChOgreGUIText> ChOgreGUITextPtr;
}