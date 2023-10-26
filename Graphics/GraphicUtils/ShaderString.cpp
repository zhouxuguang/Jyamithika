
#include "ShaderString.h"

std::string triangle2d_shader = R"(
#shader vertex
#version 330 core
layout(location = 0) in vec2 position;

void main()
{
	gl_Position = vec4(position, 0.0, 1.0);
}


#shader fragment
#version 330 core

out vec4 FragColor;

void main()
{
	FragColor = vec4(1.0,1.0,0.0, 1.0f);
}
)";

std::string generic_line_shader = R"(
#shader vertex
#version 330 core
layout(location = 0) in vec2 position;

void main()
{
	gl_Position = vec4(position, 0.0, 1.0);
}


#shader fragment
#version 330 core

out vec4 FragColor;
uniform vec3 line_color;

void main()
{
	FragColor = vec4(line_color, 1.0f);
}
)";
