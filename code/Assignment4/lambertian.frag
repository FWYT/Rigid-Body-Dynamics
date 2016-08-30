#version 120 

varying vec3 geom_normal;
varying vec3 geom_position;
varying vec3 light_position;

uniform vec4 diffuse_color;

void main() {
	vec3 n = normalize(geom_normal);
	
	float ambient = 0.4;

	float lightStrength = 3;
	vec3 l = light_position - geom_position;
	float d = sqrt(dot(l, l));
	float dotProd = max(dot(n, l), 0);

	gl_FragColor = vec4(0, 0, 0, 1);
	gl_FragColor.xyz += ambient * diffuse_color.xyz;
	gl_FragColor.xyz += diffuse_color.xyz * lightStrength * dotProd / (d * d);
}