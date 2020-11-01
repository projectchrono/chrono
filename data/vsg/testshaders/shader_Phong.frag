#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(set = 0, binding = 0) uniform LightSettings {
    vec3 light_pos;
} ls;

layout(location = 0) out vec4 FragColor;

layout(location = 0) in vec3 FragPos;
layout(location = 1) in vec3 Normal;
layout(location = 2) in vec3 AmbientColor;
layout(location = 3) in vec3 DiffuseColor;
layout(location = 4) in vec3 SpecularColor;
layout(location = 5) in float Shininess;

vec3 lightColor = vec3(1.0,1.0,1.0);
vec3 viewPos = vec3(0.0,0.0,0.0);

bool blinn = true;

void main() {
  // ambient
  vec3 ambient = lightColor * AmbientColor;
  
  // diffuse 
  vec3 norm = normalize(Normal);
  vec3 lightDir = normalize(ls.light_pos - FragPos);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = lightColor * (diff * DiffuseColor);

  // specular
  vec3 viewDir = normalize(viewPos - FragPos);
  float spec = 0.0;
  if(blinn) {
    vec3 halfwayDir = normalize(lightDir + viewDir);
    spec = pow(max(dot(norm, halfwayDir), 0.0), Shininess);
  }
  else {
    vec3 reflectDir = reflect(-lightDir, norm);
    spec = pow(max(dot(viewDir, reflectDir), 0.0), Shininess);
  }
  vec3 specular = lightColor * (spec * SpecularColor);  

  vec3 final_color = ambient + diffuse + specular;
  FragColor = vec4(final_color, 1);
}
