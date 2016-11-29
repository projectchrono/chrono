/*
Author: Charles Ricchio
*/

#include "ChOgreGUIImage.h"

namespace chrono {
	namespace ChOgre {

		unsigned int ChOgreGUIImage::g_count = 0;

		ChOgreGUIImage::ChOgreGUIImage() {
			m_pGUI = nullptr;
		}

		ChOgreGUIImage::ChOgreGUIImage(MyGUI::Gui* GUI) {
			m_pGUI = GUI;
		}

		ChOgreGUIImage::ChOgreGUIImage(const ChVector2<>& Position, const ChVector2<>& Size, MyGUI::Gui* GUI) {
			m_pGUI = GUI;

			m_pImageBox = m_pGUI->createWidgetReal<MyGUI::ImageBox>("ImageBox", Position.x, Position.y, Size.x, Size.y,
				MyGUI::Align::Center, "Main");
			//m_pTextBox->setDepth(int(Position.z)); //setDepth is not a function in mygui::button
		}

		ChOgreGUIImage::~ChOgreGUIImage() {
			m_pGUI->destroyWidget(m_pImageBox);
		}

		void ChOgreGUIImage::setColor(float r, float g, float b) {
			m_pImageBox->setColour(MyGUI::Colour(r, g, b));
		}

		void ChOgreGUIImage::setImage(const std::string& Path) {
			m_pImageBox->setImageTexture(Path);
		}

		void ChOgreGUIImage::setPosition(const ChVector2<>& Position) {
			m_pImageBox->setRealPosition(Position.x, Position.y);
			//m_pTextBox->setDepth(int(Position.z)); //setDepth is not a function in mygui::button
		}

		void ChOgreGUIImage::setSize(const ChVector2<>& Size) {
			m_pImageBox->setRealSize(Size.x, Size.y);
		}
	}
}
