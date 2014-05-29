/*	Simple obj mesh loader class that uses tiny_obj_loader to do all of the work
 */

#pragma once
#include "chrono_utils/opengl/core/ChOpenGLObject.h"
#include "tiny_obj_loader.h"
namespace chrono {
namespace utils {
class OBJLoader : public ChOpenGLBase {
 public:
   OBJLoader();
   void LoadObject(
         std::string fname,
         std::vector<std::vector<glm::vec3> > &vertices,
         std::vector<std::vector<glm::vec3> > &normals,
         std::vector<std::vector<glm::vec2> > &texcoords,
         std::vector<std::vector<GLuint> > &indices,
         std::vector<std::string> & names);
   void TakeDown() {
   }

 private:

};
}
}
