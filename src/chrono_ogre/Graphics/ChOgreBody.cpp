/*
Author: Charles Ricchio

Contains the definitions for ChOgreBody
*/

#include "ChOgreBody.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace ChOgre {

unsigned int ChOgreBody::m_MeshCount = 0;

ChOgreBody::ChOgreBody(Ogre::SceneManager* SceneManager, chrono::ChSystem* System) {
    m_pSceneManager = SceneManager;
    m_pChSystem = System;

    m_pBody = chrono::ChSharedPtr<ChBody>(new chrono::ChBody);
    System->AddBody(m_pBody);

    name = "";
    deletable = true;
}

ChOgreBody::~ChOgreBody() {
    // for (int i = 0; i < m_Models.size(); i++) {
    //	m_pSceneManager->getRootSceneNode()->removeChild(m_Models[i].getSceneNode().lock().get());
    //}
    m_pBody->RemoveCollisionModelsFromSystem();
    m_pChSystem->RemoveBody(m_pBody);
}

void ChOgreBody::update() {
    if (!isStaticMesh) {
        std::vector<chrono::ChSharedPtr<chrono::ChAsset> > l_pAssetList = m_pBody->GetAssets();
        for (int i = 0; i < l_pAssetList.size(); i++) {
            if (l_pAssetList[i].IsType<chrono::ChVisualization>()) {
                chrono::ChVector<> _pos = ((chrono::ChVisualization*)l_pAssetList[i].get_ptr())->Pos;
                // m_SceneNodes[i]->setPosition((_pos.x + m_pBody->GetPos().x), (_pos.y + m_pBody->GetPos().y), (_pos.z)
                // + m_pBody->GetPos().z);
                m_Models[i].getSceneNode()->setPosition((_pos.x + m_pBody->GetPos().x), (_pos.y + m_pBody->GetPos().y),
                                                        (_pos.z + m_pBody->GetPos().z));

                chrono::ChQuaternion<> l_q;

                chrono::ChBoxShape* shape = (chrono::ChBoxShape*)l_pAssetList[i].get_ptr();
                l_q = m_pBody->GetRot() % shape->Rot.Get_A_quaternion();
                l_q.Normalize();

                double __w = l_q.e0;
                double __x = l_q.e1;
                double __y = l_q.e2;
                double __z = l_q.e3;
                // m_SceneNodes[i]->setOrientation(__w, __x, __y, __z);
                m_Models[i].getSceneNode()->setOrientation(__w, __x, __y, __z);
            }
        }
    } else {
        // m_SceneNodes[0]->setPosition(m_pBody->GetPos().x, m_pBody->GetPos().y, m_pBody->GetPos().z);
        m_Models[0].getSceneNode()->setPosition(m_pBody->GetPos().x, m_pBody->GetPos().y, m_pBody->GetPos().z);
        chrono::ChQuaternion<> l_q;
        l_q = m_pBody->GetRot();
        l_q.Normalize();

        double __w = l_q.e0;
        double __x = l_q.e1;
        double __y = l_q.e2;
        double __z = l_q.e3;
        // m_SceneNodes[0]->setOrientation(__w, __x, __y, __z);
        m_Models[0].getSceneNode()->setOrientation(__w, __x, __y, __z);
    }
}

void ChOgreBody::refresh() {
    // for (int i = 0; i < m_SceneNodes.size(); i++) {
    //	if (m_SceneNodes[i]) {
    //		m_pSceneManager->getRootSceneNode()->removeChild(m_SceneNodes[i]);
    //		m_SceneNodes.clear();
    //	}
    //}
    m_Models.clear();
    for (int i = 0; i < m_pBody->GetAssets().size(); i++) {
        chrono::ChSharedPtr<chrono::ChAsset> temp_asset = m_pBody->GetAssets().at(i);

        // Ogre::SceneNode* l_pNode = m_pSceneManager->getRootSceneNode()->createChildSceneNode();
        // Ogre::Entity* l_pEntity = nullptr;
        ChOgreModel l_Model(m_pSceneManager);

        if (temp_asset.IsType<chrono::ChBoxShape>()) {
            l_Model.loadMesh("box.mesh");
            chrono::ChBoxShape* shape = (chrono::ChBoxShape*)temp_asset.get_ptr();
            double _w = shape->GetBoxGeometry().Size.x;
            double _h = shape->GetBoxGeometry().Size.y;
            double _d = shape->GetBoxGeometry().Size.z;
            l_Model.getSceneNode()->setScale((Ogre::Real)_w, (Ogre::Real)_h, (Ogre::Real)_d);

        } else if (temp_asset.IsType<chrono::ChCapsuleShape>()) {
        } else if (temp_asset.IsType<chrono::ChConeShape>()) {
            l_Model.loadMesh("cone.mesh");
            chrono::ChConeShape* shape = (chrono::ChConeShape*)temp_asset.get_ptr();
            double _r1 = shape->GetConeGeometry().rad.x;
            double _r2 = shape->GetConeGeometry().rad.z;
            double _h = shape->GetConeGeometry().rad.y;
            l_Model.getSceneNode()->setScale((Ogre::Real)_r1, (Ogre::Real)_h, (Ogre::Real)_r2);
        } else if (temp_asset.IsType<chrono::ChCylinderShape>()) {
            l_Model.loadMesh("cylinder.mesh");
            chrono::ChCylinderShape* shape = (chrono::ChCylinderShape*)temp_asset.get_ptr();

            double _r1 = shape->GetCylinderGeometry().rad;
            double _h = (shape->GetCylinderGeometry().p1 - shape->GetCylinderGeometry().p2).Length();
            l_Model.getSceneNode()->setScale((Ogre::Real)_r1, (Ogre::Real)_h, (Ogre::Real)_r1);
        } else if (temp_asset.IsType<chrono::ChEllipsoidShape>()) {
            l_Model.loadMesh("sphere.mesh");
            chrono::ChEllipsoidShape* shape = (chrono::ChEllipsoidShape*)temp_asset.get_ptr();

            double _rx = shape->GetEllipsoidGeometry().rad.x;
            double _ry = shape->GetEllipsoidGeometry().rad.y;
            double _rz = shape->GetEllipsoidGeometry().rad.z;
            l_Model.getSceneNode()->setScale((Ogre::Real)_rx, (Ogre::Real)_ry, (Ogre::Real)_rz);
        } else if (temp_asset.IsType<chrono::ChSphereShape>()) {
            l_Model.loadMesh("sphere.mesh");
            double _r = chrono::static_cast_chshared<chrono::ChSphereShape>(temp_asset)->GetSphereGeometry().rad;
            l_Model.getSceneNode()->setScale((Ogre::Real)_r, (Ogre::Real)_r, (Ogre::Real)_r);
        } else if (temp_asset.IsType<chrono::ChTriangleMeshShape>()) {
            std::string filepath = chrono::static_cast_chshared<chrono::ChTriangleMeshShape>(temp_asset)->GetName();

            std::string _base;
            std::string _ext;
            std::string _path;

            Ogre::StringUtil::splitFullFilename(filepath, _base, _ext, _path);

            if (_ext == "mesh") {
                l_Model.loadMesh(filepath);
            } else if (_ext == "obj") {
                std::vector<tinyobj::shape_t> _shapes;
                std::string error = tinyobj::LoadObj(_shapes, filepath.c_str());
                if (!error.empty()) {
                    Ogre::LogManager::getSingleton().logMessage(Ogre::LML_CRITICAL,
                                                                "! Could not load OBJ file: " + filepath + " !");
                    return;
                }

                for (unsigned int i = 0; i < _shapes.size(); i++) {
                    Ogre::ManualObject* _manual_object =
                        m_pSceneManager->createManualObject("Terrain Mesh " + std::to_string(m_MeshCount));

                    _manual_object->begin("lambert1", Ogre::RenderOperation::OT_TRIANGLE_LIST);

                    for (unsigned int j = 0; j < _shapes[i].mesh.positions.size(); j += 3) {
                        _manual_object->position(Ogre::Vector3(_shapes[i].mesh.positions[j + 0],
                                                               _shapes[i].mesh.positions[j + 1],
                                                               _shapes[i].mesh.positions[j + 2]));
                        _manual_object->normal(Ogre::Vector3(_shapes[i].mesh.normals[j + 0],
                                                             _shapes[i].mesh.normals[j + 1],
                                                             _shapes[i].mesh.normals[j + 2]));
                        _manual_object->colour(Ogre::ColourValue(0.5, 0.5, 0.5));

                        _manual_object->textureCoord(Ogre::Vector2(_shapes[i].mesh.texcoords[((j / 3) * 2) + 0],
                                                                   _shapes[i].mesh.texcoords[((j / 3) * 2) + 1]));
                    }

                    for (unsigned j = 0; j < _shapes[i].mesh.texcoords.size(); j += 2) {
                    }

                    for (unsigned j = 0; j < _shapes[i].mesh.indices.size(); j++) {
                        _manual_object->index(_shapes[i].mesh.indices[j]);
                    }

                    _manual_object->end();

                    l_Model.getSceneNode()->attachObject(_manual_object);
                }

                // m_SceneNodes.push_back(l_pNode);
                m_Models.push_back(std::move(l_Model));

                chrono::ChTriangleMeshShape* shape = (chrono::ChTriangleMeshShape*)temp_asset.get_ptr();

                double _sx = shape->GetScale().x;
                double _sy = shape->GetScale().y;
                double _sz = shape->GetScale().z;
                l_Model.getSceneNode()->setScale((Ogre::Real)_sx, (Ogre::Real)_sy, (Ogre::Real)_sz);

                m_MeshCount++;
                return;
            } else {
                return;
            }

            chrono::ChTriangleMeshShape* shape = (chrono::ChTriangleMeshShape*)temp_asset.get_ptr();

            double _sx = shape->GetScale().x;
            double _sy = shape->GetScale().y;
            double _sz = shape->GetScale().z;
            l_Model.getSceneNode()->setScale((Ogre::Real)_sx, (Ogre::Real)_sy, (Ogre::Real)_sz);
		} else if (temp_asset.IsType<chrono::ChTexture>()) {

		}

        // l_pNode->attachObject(l_pEntity);
        m_Models.push_back(std::move(l_Model));
    }
    isStaticMesh = false;
}

void ChOgreBody::setMesh(Ogre::ManualObject* Mesh, const chrono::ChVector<>& Scale) {
    // for (int i = 0; i < m_SceneNodes.size(); i++) {
    //	if (m_SceneNodes[i]) {
    //		m_pSceneManager->getRootSceneNode()->removeChild(m_SceneNodes[i]);
    //		m_SceneNodes.clear();
    //	}
    //}

    m_Models.clear();

    ChOgreModel l_Model(m_pSceneManager);
    l_Model.setMesh(Mesh->convertToMesh(Mesh->getName()));

    // Ogre::SceneNode* l_pNode = m_pSceneManager->getRootSceneNode()->createChildSceneNode();

    // l_pEntity->setMaterialName("BaseWhite");
    // l_pEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState("white.png");

    l_Model.getSceneNode()->setScale((Ogre::Real)Scale.x, (Ogre::Real)Scale.y, (Ogre::Real)Scale.z);

    m_Models.push_back(std::move(l_Model));
    isStaticMesh = true;
}

chrono::ChSharedPtr<ChBody> ChOgreBody::getChBody() {
    return m_pBody;
}

chrono::ChSharedPtr<ChBody> ChOgreBody::operator->() {
    return getChBody();
}
}
