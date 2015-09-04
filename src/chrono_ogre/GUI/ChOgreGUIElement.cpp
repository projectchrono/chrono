#include "ChOgreGUIElement.h"

namespace ChOgre {

ChOgreGUIElement::ChOgreGUIElement() {
    m_pGUI = nullptr;
}

ChOgreGUIElement::ChOgreGUIElement(MyGUI::Gui* GUI) {
    m_pGUI = GUI;
}

ChOgreGUIElement::~ChOgreGUIElement() {}

void ChOgreGUIElement::setName(std::string Name) {
    m_Name = Name;
}

void ChOgreGUIElement::setGUI(MyGUI::Gui* GUI) {
    m_pGUI = GUI;
}

std::string ChOgreGUIElement::getName() {
    return m_Name;
}
}