
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

std::string basic_poly_2d_shader = R"(
#shader vertex
#version 330 core
layout(location = 0) in vec3 vertex;
layout(location = 1) in vec3 incolor;

out vec3 tricolor;

void main()
{
	gl_Position = vec4(vertex, 1.0);
	tricolor = incolor;
}


#shader fragment
#version 330 core

in vec3 tricolor;
out vec4 FragColor;

void main()
{
	FragColor = vec4(tricolor, 0.5f);
}
)";

std::string line_shader_str = R"(
#shader vertex
#version 330 core
layout(location = 0) in vec2 vertex;

void main()
{
	gl_Position = vec4(vertex, 0.0, 1.0);
}


#shader fragment
#version 330 core

out vec4 FragColor;

void main()
{
	FragColor = vec4(0.95, 0.02, 0.03, 1.0f);
}
)";

std::string line2_shader_str = R"(
#shader vertex
#version 330 core
layout(location = 0) in vec2 vertex;

void main()
{
	gl_Position = vec4(vertex, 0.0, 1.0);
}


#shader fragment
#version 330 core

out vec4 FragColor;

void main()
{
	FragColor = vec4(0.0, 1.0, 0.5, 1.0f);
}
)"
;
