#ifndef CHOPENGLFRUSTUM_H
#define CHOPENGLFRUSTUM_H

/*  Based on code provided by: Perry Kivolowitz
 Draws a frustum (also cubes)
 Draws independent planes for each side, functions provided to draw plane based on normal or 4 points
 */

#include "chrono_utils/opengl/core/ChOpenGLObject.h"

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLFrustum : public ChOpenGLObject {
 public:
   ChOpenGLFrustum();
   void InitPlane(
         const glm::vec3 pos,
         const glm::vec3 normal,
         const glm::vec2 size,
         ChOpenGLMaterial mat);

   void InitPlane(
         glm::vec3 p1,
         glm::vec3 p2,
         glm::vec3 p3,
         glm::vec3 p4,
         ChOpenGLMaterial mat);

   bool Initialize(
         const glm::vec2 base,
         const glm::vec2 top,
         const float height,
         ChOpenGLMaterial mat);
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview);
   void TakeDown();

 private:
   std::vector<glm::vec3> position;
   std::vector<glm::vec3> normal;
   std::vector<glm::vec3> color;
   typedef ChOpenGLObject super;
};
}
}
#endif  // END of CHOPENGLMESH_H
