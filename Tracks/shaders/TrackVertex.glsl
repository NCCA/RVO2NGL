#version 410 core

/// @brief MVP matrix passed from our app
uniform mat4 MVP;

layout (location=0) in vec3 inVert;
layout (location=1) in vec3 inColour;
out vec3 outColour;

void main()
{

  // calculate the vertex position
  gl_Position = MVP*vec4(inVert, 1.0);
  outColour=inColour;
}
