#include "glad\glad.h"
#include <GLFW\glfw3.h>
#include <array>

#include "GLM\glm.hpp"
#include "GLM\gtc\matrix_transform.hpp"
#include "GLM\gtc\type_ptr.hpp"

#include "GraphicUtils\Imgui\imgui.h"
#include "GraphicUtils\Imgui\imgui_impl_glfw.h"
#include "GraphicUtils\Imgui\imgui_impl_opengl3.h"

#include "GraphicUtils\ShaderProgram.h"
#include "GraphicUtils\VertexArray.h"
#include "GraphicUtils\VertexBuffer.h"
#include "GraphicUtils\GraphicUtils.h"

#include "GraphicUtils\Geometry\GFace.h"
#include "GraphicUtils\Geometry\GPoint.h"
#include "GraphicUtils\Geometry\GLine.h"

#include "Jyamithika\Core\Primitives\Point.h"
#include "Jyamithika\Core\Primitives\PolygonDCEL.h"
#include "Jyamithika\Convexhull.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::milli;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

const unsigned int SCREEN_WIDTH = 900;
const unsigned int SCREEN_HEIGHT = 900;

bool firstMouse = true;
float lastX = SCREEN_WIDTH / 2.0f;
float lastY = SCREEN_HEIGHT / 2.0f;

int option = 0;

//Camera camera(glm::vec3(0.0f, 0.0f, 12.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


void setup_pointcloud(std::vector<jmk::Point2d>& points)
{
	points.push_back(jmk::Point2d(0.7, 0.4));
	points.push_back(jmk::Point2d(0.4, 0.1));
	points.push_back(jmk::Point2d(0.3, 0.8));
	points.push_back(jmk::Point2d(0.05, 0.7));
	points.push_back(jmk::Point2d(-0.05, 0.8));
	points.push_back(jmk::Point2d(-0.5, 0.5));
	points.push_back(jmk::Point2d(-0.2, 0.2));
	points.push_back(jmk::Point2d(-0.4, 0.05));
	points.push_back(jmk::Point2d(-0.6, 0.15));
	points.push_back(jmk::Point2d(-0.7, -0.2));
	points.push_back(jmk::Point2d(-0.3, -0.5));
	points.push_back(jmk::Point2d(-0.1, -0.3));
	points.push_back(jmk::Point2d(0.2, -0.8));
	points.push_back(jmk::Point2d(0.1, -0.05));
	points.push_back(jmk::Point2d(0.5, -0.1));
}

void setup_pointcloud_2(std::vector<jmk::Point2d>& points)
{
	points.push_back(jmk::Point2d(0.2, 0.4));
	points.push_back(jmk::Point2d(0.6, 0.2));
	points.push_back(jmk::Point2d(0.8, 0.5));
	points.push_back(jmk::Point2d(0.9, 0.7));
	points.push_back(jmk::Point2d(0.5, 0.9));
	points.push_back(jmk::Point2d(0.4, 0.6));
}

int main(void)
{
	std::srand(std::time(nullptr));
	GLFWwindow* window;

	//Set up glfw context and window
	{
		/* Initialize the library */
		if (!glfwInit())
			return -1;

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		/* Create a windowed mode window and its OpenGL context */
		window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Convexhull 2D", NULL, NULL);
		if (!window)
		{
			glfwTerminate();
			return -1;
		}

		/* Make the window's context current */
		glfwMakeContextCurrent(window);
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
			return -1;
		}

		//glEnable(GL_DEPTH_TEST);
	}

	//Set up ImGui
	{
		// Setup Dear ImGui context
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();

		// Setup Dear ImGui style
		ImGui::StyleColorsDark();
		//ImGui::StyleColorsClassic();

		// Setup Platform/Renderer bindings
		ImGui_ImplOpenGL3_Init("#version 130");
		ImGui_ImplGlfw_InitForOpenGL(window, true);
	}

	std::vector<jmk::Point2d> points;
	std::vector<jmk::Point2d> hull_points_giftwrapping;
	std::vector<jmk::Point2d> hull_points_grahams;

	std::vector<float> point_data;
	std::vector<float> convexhull_giftwrapping_edge_data;
	std::vector<float> convexhull_grahams_edge_data;

	setup_pointcloud(points);
	getReactanglePointClouds(points, point_data);

	jmk::convexhull2DGiftwrapping(points,hull_points_giftwrapping);
	jmk::Polygon2d polygon1(hull_points_giftwrapping);
	auto edge_list = polygon1.getEdgeList();
	get2DLinePointsFromDCEL2d(edge_list, convexhull_giftwrapping_edge_data);

	jmk::convexhull2DModifiedGrahams(points, hull_points_grahams);
	jmk::Polygon2d polygon2(hull_points_grahams);
	edge_list = polygon2.getEdgeList();
	get2DLinePointsFromDCEL2d(edge_list, convexhull_grahams_edge_data);

	VertexArray VAO_points;
	VertexBuffer VBO_points(point_data.data(), point_data.size());
	VAO_points.addVertexLayout(0, 2, GL_FALSE, 2 * sizeof(float), 0);

	VertexArray VAO_convexhulledges;
	VertexBuffer VBO_edge(convexhull_giftwrapping_edge_data.data(), convexhull_giftwrapping_edge_data.size());
	VAO_convexhulledges.addVertexLayout(0, 2, GL_FALSE, 2 * sizeof(float), 0);

	VertexArray VAO_convexhullgrahams;
	VertexBuffer VBO_edge_grahms(convexhull_grahams_edge_data.data(), convexhull_grahams_edge_data.size());
	VAO_convexhullgrahams.addVertexLayout(0, 2, GL_FALSE, 2 * sizeof(float), 0);

	ShaderProgram shader("C:/Users/intellect/source/repos/Jyamithika/Graphics/GraphicUtils/Shaders/triangle2d.shader");
	ShaderProgram line_shader("C:/Users/intellect/source/repos/Jyamithika/Graphics/GraphicUtils/Shaders/generic_line.shader");
	unsigned int line_color_loc = line_shader.getUniformId("line_color");

	glm::vec3 red = glm::vec3(0.95f, 0.02f, 0.03f);
	glm::vec3 blue = glm::vec3(0.55f, 0.42f, 0.83f);
	glm::vec3 green = glm::vec3(0.15f, 0.92f, 0.13f);
	glm::vec3 yellow = glm::vec3(0.9f, 0.95f, 0.2f);

	bool show_points = true, show_convexhull_giftwrapping = false, show_convexhull_grahams = false;

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window))
	{
		// per-frame time logic
		// --------------------
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		processInput(window);
		/* Render here */

		glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		if (show_points)
		{
			shader.activeAsCurrentShader();
			VAO_points.bindVertexArray();
			glDrawArrays(GL_TRIANGLES, 0, point_data.size() / 2);
		}

		if (show_convexhull_giftwrapping)
		{
			line_shader.activeAsCurrentShader();
			glLineWidth(4);
			glUniform3fv(line_color_loc, 1, glm::value_ptr(red));
			VAO_convexhulledges.bindVertexArray();
			glDrawArrays(GL_LINES, 0, convexhull_giftwrapping_edge_data.size() / 2);
		}

		if (show_convexhull_grahams)
		{
			line_shader.activeAsCurrentShader();
			glLineWidth(4);
			glUniform3fv(line_color_loc, 1, glm::value_ptr(green));
			VAO_convexhullgrahams.bindVertexArray();
			glDrawArrays(GL_LINES, 0, convexhull_grahams_edge_data.size() / 2);
		}

		ImGui::Begin(" Sample : Convexhull in 2D");
		ImGui::Checkbox("Original Points", &show_points);
		ImGui::Checkbox("Show Convexhull Giftwrapping", &show_convexhull_giftwrapping);
		ImGui::Checkbox("Show Convexhull Grahams", &show_convexhull_grahams);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		option = 1;
	}
	else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
	{
		option = 2;
	}
	else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{
		option = 3;
	}
	else
	{
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);
	}
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	return;
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	//camera.ProcessMouseMovement(xoffset, yoffset, true);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	//camera.ProcessMouseScroll(yoffset);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}