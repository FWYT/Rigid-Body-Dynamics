#version 120

varying vec3 geom_normal;
varying vec3 geom_position;
varying vec3 light_position;

void main() {
	vec3 light_worldPos = vec3(0, 1, -1);
	// light_position = (gl_ModelViewMatrix * vec4(light_worldPos, 1)).xyz;
	light_position = light_worldPos;
	geom_position = (gl_ModelViewMatrix * gl_Vertex).xyz;
	geom_normal = gl_NormalMatrix * gl_Normal;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_FrontColor = gl_Color;
}