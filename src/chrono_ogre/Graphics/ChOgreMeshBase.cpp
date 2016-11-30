#include "ChOgreMeshBase.h"

namespace chrono{
namespace ChOgre {

ChOgreMeshBase::ChOgreMeshBase() {
    m_pEntity = nullptr;
    m_pSceneManager = nullptr;
}

ChOgreMeshBase::ChOgreMeshBase(Ogre::SceneManager* SceneManager) {
    assignSceneManager(SceneManager);
}

ChOgreMeshBase::ChOgreMeshBase(Ogre::SceneManager* SceneManager, const std::string& MeshPath) {
    assignSceneManager(SceneManager);
    loadMesh(MeshPath);
}

ChOgreMeshBase::ChOgreMeshBase(const ChOgreMeshBase& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pEntity = _other.m_pEntity;
    }
}

ChOgreMeshBase::ChOgreMeshBase(ChOgreMeshBase&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pEntity = std::move(_other.m_pEntity);
    }
}

ChOgreMeshBase::~ChOgreMeshBase() {
    if (m_pEntity != nullptr) {
        // m_pSceneManager->destroyEntity(m_pEntity.get());
    }
}

ChOgreMeshBase& ChOgreMeshBase::operator=(const ChOgreMeshBase& _other) {
    if (this != &_other) {
        m_pSceneManager = _other.m_pSceneManager;
        m_pEntity = _other.m_pEntity;
    }

    return *this;
}

ChOgreMeshBase& ChOgreMeshBase::operator=(ChOgreMeshBase&& _other) {
    if (this != &_other) {
        m_pSceneManager = std::move(_other.m_pSceneManager);
        m_pEntity = std::move(_other.m_pEntity);
    }

    return *this;
}

void ChOgreMeshBase::assignSceneManager(Ogre::SceneManager* SceneManager) {
    m_pSceneManager = SceneManager;
}

void ChOgreMeshBase::loadMesh(const std::string& MeshPath) {
	m_pEntity = m_pSceneManager->createEntity(MeshPath);

	//Ogre::MaterialPtr m = m_pEntity->getSubEntity(0)->getMaterial()->clone(std::to_string((uint64_t)this));
}

void ChOgreMeshBase::setMesh(const Ogre::MeshPtr& pMesh) {
	m_pEntity = m_pSceneManager->createEntity(pMesh);
}

void ChOgreMeshBase::setColor(const chrono::ChVector<>& Color) {
	setColor(Color.x, Color.y, Color.z);
}

void ChOgreMeshBase::setColor(float r, float g, float b) {
//	auto pPass = m_pEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0);
//	pPass->setAmbient(r, g, b);
//	pPass->setDiffuse(r, g, b, 0);

//	auto pTechnique = m_pEntity->getSubEntity(0)->getMaterial()->getTechnique(0);
//	auto pPass = pTechnique->getPass(0);
//	auto pParameters = pPass->getFragmentProgramParameters();
//	pParameters->setNamedConstant("diffuse", Ogre::ColourValue(r, g, b));
//	pParameters->setNamedConstant("ambient", Ogre::ColourValue(r, g, b));

//	pParameters->setNamedConstant("ambient", Ogre::ColourValue(r, g, b));
//	pParameters->setNamedConstant("diffuse", Ogre::ColourValue(r, g, b));

//	m_pEntity->getSubEntity(0)->getTechnique()->setAmbient(r, g, b);
//	m_pEntity->getSubEntity(0)->getTechnique()->setDiffuse(r, g, b, 1);
}

}
}
