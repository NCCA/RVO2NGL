#version 410 core
/// @brief our output fragment colour
layout (location =0) out vec4 fragColour;
// input colour from vert shader
in vec3 outColour;


void main()
{
  fragColour.rgb=outColour;
}
