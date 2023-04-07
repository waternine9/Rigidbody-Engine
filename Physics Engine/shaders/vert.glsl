#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;


out VS_OUT {
  vec2 TexCoord;
  vec3 PosCoord;
  vec4 WorldPosCoord;
  vec4 LightPosCoord;
} vsOut;

uniform mat4 vp;
uniform mat4 lightMat;

void main()
{
	gl_Position = vp * vec4(aPos, 1.0);
	vsOut.PosCoord = vec3(vec4(aPos, 1.0));
	vsOut.LightPosCoord = lightMat * vec4(aPos, 1.0);
	vsOut.TexCoord = vec2(aTexCoord.x, aTexCoord.y);
	vsOut.WorldPosCoord = vec4(aPos, 1.0);
}
