#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 ambientColor;
layout(location = 1) in vec3 diffuseColor;
layout(location = 2) in vec3 specularColor;
layout(location = 3) in vec3 normal;
layout(location = 4) in vec3 eye_vec;

layout(location = 0) out vec4 FragColor;

void main() {
	vec3 light_pos = vec3(100, 100, 100);
	float shininess = 100.0;
	
	vec3 final_color = ambientColor;	
	
	vec3 N = normalize(normal);
  	vec3 L = normalize(light_pos - eye_vec);
  	vec3 E = normalize(-eye_vec);

	float lambertTerm = dot(N, L);

  	if (lambertTerm > 0.0) {
    	final_color += diffuseColor * lambertTerm;
    	vec3 R = reflect(-L, N);
    	float specular = pow(max(dot(R, E), 0.0), shininess);
    	final_color += specularColor * specular;
  }

  FragColor = vec4(final_color, 1);
}
