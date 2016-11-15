#include "ChOgreSprite.h"

chrono::ChOgre::ChOgreSprite::ChOgreSprite(Ogre::SceneManager* SceneManager) {
	m_pSceneManager = SceneManager;

	m_pSceneNode = m_pSceneManager->getRootSceneNode()->createChildSceneNode();
	m_pBillboardSet = m_pSceneManager->createBillboardSet();

	m_pSceneNode->attachObject(m_pBillboardSet);

	m_pBillboard = m_pBillboardSet->createBillboard(0.f, 0.f, 0.f);
}

chrono::ChOgre::ChOgreSprite::~ChOgreSprite() {
	m_pBillboardSet->removeBillboard(m_pBillboard);
	m_pSceneManager->destroyBillboardSet(m_pBillboardSet);
	m_pSceneManager->getRootSceneNode()->removeChild(m_pSceneNode);
}
