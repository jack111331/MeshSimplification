#version 450 core

out vec4 finalColor;

in vec3 fPos;
in vec3 fNormal;


uniform vec3 viewPos;
uniform vec3 objectColor;

void main()
{
    finalColor = vec4(0.6, 0.4, 0.6, 1.0);
}