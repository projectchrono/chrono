#include "ChOpenGLVertexAttributes.h"

using namespace glm;
using namespace chrono::utils;
//This file contains the constructors for each vertex attribute type
//nothing really important here worth commenting....
ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT()
{
    this->position = vec3(0.0f);
    this->color_ambient = vec3(0.0f);
    this->color_diffuse = vec3(0.0f);
    this->color_specular = vec3(0.0f);
    this->color_glow = vec3(0.0f);
    this->normal = vec3(0.0f);
    this->texture_coordinate = vec2(0.0f);


}
ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT(const glm::vec3 & p, const glm::vec3 & c_a, const glm::vec3 & c_d, const glm::vec3 & c_s, const glm::vec3 & c_g, const glm::vec3 & n, const glm::vec2 & t)
{
    this->position = p;
    this->color_ambient = c_a;
    this->color_diffuse = c_d;
    this->color_specular = c_s;
    this->color_glow = c_g;
    this->normal = n;
    this->texture_coordinate = t;


}
ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT(const ChOpenGLVertexAttributesPADSNT & other)
{

    this->position = other.position;
    this->color_ambient = other.color_ambient;
    this->color_diffuse = other.color_diffuse;
    this->color_specular = other.color_specular;
    this->color_glow = other.color_glow;
    this->normal = other.normal;
    this->texture_coordinate = other.texture_coordinate;

}


ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT()
{
    this->position = vec3(0.0f);
    this->color = vec3(0.0f);
    this->normal = vec3(0.0f);
    this->texture_coordinate = vec2(0.0f);
}

ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT(const vec3 & p, const vec3 & c, const vec3 & n, const vec2 & t)
{
    this->position = p;
    this->color = c;
    this->normal = n;
    this->texture_coordinate = t;
}

ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT(const ChOpenGLVertexAttributesPCNT & other)
{
    this->position = other.position;
    this->color = other.color;
    this->normal = other.normal;
    this->texture_coordinate = other.texture_coordinate;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP(const vec3 & p)
{
    this->position = p;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP(const ChOpenGLVertexAttributesP & other)
{
    this->position = other.position;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP()
{
    this->position = vec3(0.0f);
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN(const vec3 & p, const vec3 & n)
{
    this->position = p;
    this->normal = n;
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN(const ChOpenGLVertexAttributesPN & other)
{
    this->position = other.position;
    this->normal = other.normal;
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN()
{
    this->position = vec3(0.0f);
    this->normal = vec3(0.0f, 0.0f, 1.0f);
}


ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN()
{
    this->position = vec3(0.0f);
    this->color = vec3(0.0f);
    this->normal = vec3(0.0f);
}

ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN(const vec3 & p, const vec3 & c, const vec3 & n)
{
    this->position = p;
    this->color = c;
    this->normal = n;
}

ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN(const ChOpenGLVertexAttributesPCN & other)
{
    this->position = other.position;
    this->color = other.color;
    this->normal = other.normal;
}
