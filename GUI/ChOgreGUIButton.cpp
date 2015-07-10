#include "ChOgreGUIButton.h"

namespace ChOgre {

	ChOgreGUIButton::ChOgreGUIButton() {
		m_pGUI = nullptr;
	}

	ChOgreGUIButton::ChOgreGUIButton(MyGUI::Gui* GUI) {
		m_pGUI = GUI;
	}

	ChOgreGUIButton::ChOgreGUIButton(const ChInt3& Position, const ChInt3& Size, MyGUI::Gui* GUI) {
		m_pGUI = GUI;
		
		m_pButton = m_pGUI->createWidget<MyGUI::Button>("Button", Position.x, Position.y, Size.x, Size.y, MyGUI::Align::Center, "Main");

		m_db = true;
		m_pressed = false;
	}

	ChOgreGUIButton::~ChOgreGUIButton() {
		m_pGUI->destroyWidget(m_pButton);
	}

	void ChOgreGUIButton::setColor(float r, float g, float b) {
		m_pButton->setColour(MyGUI::Colour(r, g, b));
	}

	void ChOgreGUIButton::setTextColor(float r, float g, float b) {
		m_pButton->setTextColour(MyGUI::Colour(r, g, b));
	}

	void ChOgreGUIButton::setText(const std::string& Text) {
		m_pButton->setCaption(Text);
	}

	void ChOgreGUIButton::setFont(const std::string& Name) {
		m_pButton->setFontName(Name);
	}
	
	void ChOgreGUIButton::setPosition(const ChInt3& Position) {
		m_pButton->setPosition(Position.x, Position.y);
	}

	void ChOgreGUIButton::setSize(const ChInt3& Size) {
		m_pButton->setSize(Size.x, Size.y);
	}

	void ChOgreGUIButton::update() {

	}

	void ChOgreGUIButton::setClickCallback(ChOgreGUICallback& Callback) {
		auto f_p = &ChOgreGUICallback::call;
		m_pButton->eventMouseButtonClick += MyGUI::newDelegate(&Callback, f_p);
	}

}