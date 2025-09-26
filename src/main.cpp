#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include "ConstrainedIntepolation.hpp"

// --- Shader Sources ---
const char* vertexShaderSource = R"glsl(
    #version 400 core
    layout (location = 0) in vec2 aPos;
    uniform mat4 projection;
    void main() 
    {
        gl_Position = projection * vec4(aPos.x, aPos.y, 0.0, 1.0);
    }
)glsl";

const char* fragmentShaderSource = R"glsl(
    #version 400 core
    out vec4 FragColor;
    uniform vec3 ourColor;
    void main() 
    {
        FragColor = vec4(ourColor, 1.0);
    }
)glsl";

// --- Global State ---
// "슬라럼" 경로 예시 데이터 // zoom = 6.0
std::vector<double> t = {0.0, 1.5, 3.0, 4.5};
std::vector<double> x = {-9.0, -3.0, 3.0, 9.0};
std::vector<double> y = {-2.0, 2.0, -2.0, 2.0};
std::vector<double> direction = {45.0, -45.0, 45.0, -45.0};
std::vector<double> curvature = {0.0, -0.5, 0.5, 0.0};

// // "루프" 경로 예시 데이터  // zoom = 8.0
std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
std::vector<double> x = {-4.0, -2.0, 0.0, 2.0, 4.0};
std::vector<double> y = {-5.0, -1.0, 1.0, -1.0, -5.0};
std::vector<double> direction = {45.0, 90.0, 0.0, 270.0, -45.0};
std::vector<double> curvature = {0.0, 0.2, 0.2, 0.2, 0.0};

// // "장애물 회피" 경로 예시 데이터 // zoom = 6.0
std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
std::vector<double> x = {-8.0, -3.0, 0.0, 3.0, 8.0};
std::vector<double> y = {-2.0, -2.0, 2.0, -2.0, 2.0};
std::vector<double> direction = {0.0, 45.0, 0.0, -45.0, 0.0};
std::vector<double> curvature = {0.0, 0.3, -0.5, 0.3, 0.0};

int selected_point = 0;
bool needs_update = true;
std::optional<ConstrainedInterpolation> interpolator;

// --- OpenGL Handles ---
GLuint shaderProgram;
GLuint splineVAO, splineVBO, gridVAO, gridVBO, pointsVAO, pointsVBO, vectorsVAO, vectorsVBO, circlesVAO, circlesVBO;
GLsizei spline_vert_count = 0;
GLsizei grid_vert_count = 0;
GLsizei points_vert_count = 0;
GLsizei vectors_vert_count = 0;
std::vector<std::pair<GLint, GLsizei>> circle_draw_commands;

// --- Helper Functions ---
void checkShaderCompilation(GLuint shader) 
{
    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) 
    {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        throw std::runtime_error("Shader compilation failed: " + std::string(infoLog));
    }
}

void checkProgramLinking(GLuint program) 
{
    int success;
    char infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) 
    {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        throw std::runtime_error("Shader program linking failed: " + std::string(infoLog));
    }
}

void setupShaders() 
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    checkShaderCompilation(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    checkShaderCompilation(fragmentShader);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    checkProgramLinking(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void setupBuffers() 
{
    auto setupVaoVbo = [](GLuint& vao, GLuint& vbo) 
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    };
    setupVaoVbo(splineVAO, splineVBO);
    setupVaoVbo(gridVAO, gridVBO);
    setupVaoVbo(pointsVAO, pointsVBO);
    setupVaoVbo(vectorsVAO, vectorsVBO);
    setupVaoVbo(circlesVAO, circlesVBO);

    std::vector<float> grid_verts;
    int grid_size = 100;
    for (int i = -grid_size; i <= grid_size; ++i) 
    {
        grid_verts.push_back((float)i); grid_verts.push_back((float)-grid_size);
        grid_verts.push_back((float)i); grid_verts.push_back((float)grid_size);
        grid_verts.push_back((float)-grid_size); grid_verts.push_back((float)i);
        grid_verts.push_back((float)grid_size); grid_verts.push_back((float)i);
    }
    grid_vert_count = grid_verts.size() / 2;
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, grid_verts.size() * sizeof(float), grid_verts.data(), GL_STATIC_DRAW);
}

void updateDynamicBuffers() {
    if (!interpolator) return;

    std::vector<float> spline_verts, point_verts, vector_verts, circle_verts;
    circle_draw_commands.clear();

    for (double time = t.front(); time <= t.back(); time += 0.01) {
        if (auto pos = (*interpolator)(time)) {
            spline_verts.push_back(static_cast<float>(pos->first));
            spline_verts.push_back(static_cast<float>(pos->second));
        }
    }

    GLint circle_first = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        point_verts.push_back(static_cast<float>(x[i]));
        point_verts.push_back(static_cast<float>(y[i]));

        vector_verts.push_back(static_cast<float>(x[i]));
        vector_verts.push_back(static_cast<float>(y[i]));
        double angle_rad = direction[i] * M_PI / 180.0;
        vector_verts.push_back(static_cast<float>(x[i] + 0.5 * cos(angle_rad)));
        vector_verts.push_back(static_cast<float>(y[i] + 0.5 * sin(angle_rad)));

        if (std::abs(curvature[i]) > 1e-6) {
            double radius = 1.0 / curvature[i];
            double center_x = x[i] - radius * sin(angle_rad);
            double center_y = y[i] + radius * cos(angle_rad);
            
            GLsizei circle_vert_count_for_this_circle = 0;
            for (int j = 0; j <= 360; ++j) {
                double circle_angle = j * M_PI / 180.0;
                circle_verts.push_back(static_cast<float>(center_x + std::abs(radius) * cos(circle_angle)));
                circle_verts.push_back(static_cast<float>(center_y + std::abs(radius) * sin(circle_angle)));
                circle_vert_count_for_this_circle++;
            }
            circle_draw_commands.emplace_back(circle_first, circle_vert_count_for_this_circle);
            circle_first += circle_vert_count_for_this_circle;
        }
    }

    spline_vert_count = spline_verts.size() / 2;
    points_vert_count = point_verts.size() / 2;
    vectors_vert_count = vector_verts.size() / 2;

    glBindBuffer(GL_ARRAY_BUFFER, splineVBO);
    glBufferData(GL_ARRAY_BUFFER, spline_verts.size() * sizeof(float), spline_verts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER, point_verts.size() * sizeof(float), point_verts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vectorsVBO);
    glBufferData(GL_ARRAY_BUFFER, vector_verts.size() * sizeof(float), vector_verts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, circlesVBO);
    glBufferData(GL_ARRAY_BUFFER, circle_verts.size() * sizeof(float), circle_verts.data(), GL_DYNAMIC_DRAW);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) 
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT) 
    {
        needs_update = true;
        switch (key) 
        {
            case GLFW_KEY_LEFT: selected_point = (selected_point > 0) ? selected_point - 1 : t.size() - 1; break;
            case GLFW_KEY_RIGHT: selected_point = (selected_point < t.size() - 1) ? selected_point + 1 : 0; break;
            case GLFW_KEY_A: direction[selected_point] += 5.0; break;
            case GLFW_KEY_D: direction[selected_point] -= 5.0; break;
            case GLFW_KEY_W: curvature[selected_point] += 1.0; break;
            case GLFW_KEY_S: curvature[selected_point] -= 1.0; if (std::abs(curvature[selected_point]) < 0.05) curvature[selected_point] = 0; break;
            case GLFW_KEY_ESCAPE:glfwSetWindowShouldClose(window, GLFW_TRUE); break;
        }
    }
}

void printInstructions() 
{
    std::cout << "--- Controls ---\n"
              << "Left/Right Arrows: Select control point\n"
              << "A/D: Adjust direction of selected point\n"
              << "W/S: Adjust curvature of selected point\n"
              << "ESC: Exit\n"
              << "----------------\n";
}

Eigen::Matrix4f createOrthoMatrix(float left, float right, float bottom, float top, float near, float far) 
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 0) = 2.0f / (right - left);
    mat(1, 1) = 2.0f / (top - bottom);
    mat(2, 2) = -2.0f / (far - near);
    mat(0, 3) = -(right + left) / (right - left);
    mat(1, 3) = -(top + bottom) / (top - bottom);
    mat(2, 3) = -(far + near) / (far - near);
    return mat;
}

int main() {
    try {
        if (!glfwInit()) throw std::runtime_error("Failed to initialize GLFW");

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        GLFWwindow* window = glfwCreateWindow(1280, 720, "Constrained Interpolation Visualizer (Core Profile)", NULL, NULL);
        if (!window) throw std::runtime_error("Failed to create GLFW window");

        glfwMakeContextCurrent(window);
        glfwSetKeyCallback(window, keyCallback);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) 
        {
            throw std::runtime_error("Failed to initialize GLAD");
        }

        setupShaders();
        setupBuffers();
        printInstructions();

        glUseProgram(shaderProgram);
        GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
        GLint colorLoc = glGetUniformLocation(shaderProgram, "ourColor");

        while (!glfwWindowShouldClose(window)) 
        {
            if (needs_update) 
            {
                interpolator = ConstrainedInterpolation::create(t, x, y, direction, curvature);
                updateDynamicBuffers();
                needs_update = false;
            }

            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            glViewport(0, 0, width, height);

            float aspect_ratio = (float)width / (float)height;
            float zoom = 6.0f;
            Eigen::Matrix4f proj = createOrthoMatrix(-zoom * aspect_ratio, zoom * aspect_ratio, -zoom, zoom, -1.0f, 1.0f);
            glUniformMatrix4fv(projLoc, 1, GL_FALSE, proj.data());

            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            // Draw Grid
            glUniform3f(colorLoc, 0.2f, 0.2f, 0.2f);
            glBindVertexArray(gridVAO);
            glDrawArrays(GL_LINES, 0, grid_vert_count);

            // Draw Spline
            glUniform3f(colorLoc, 1.0f, 1.0f, 0.0f);
            glLineWidth(2.0f);
            glBindVertexArray(splineVAO);
            glDrawArrays(GL_LINE_STRIP, 0, spline_vert_count);

            // Draw Circles
            glUniform3f(colorLoc, 1.0f, 0.0f, 1.0f);
            glLineWidth(1.0f);
            glBindVertexArray(circlesVAO);
            for (const auto& cmd : circle_draw_commands) {
                glDrawArrays(GL_LINE_LOOP, cmd.first, cmd.second);
            }

            // Draw Vectors
            glUniform3f(colorLoc, 0.0f, 1.0f, 1.0f);
            glLineWidth(1.5f);
            glBindVertexArray(vectorsVAO);
            glDrawArrays(GL_LINES, 0, vectors_vert_count);
            
            // Draw Points
            glPointSize(10.0f);
            glBindVertexArray(pointsVAO);
            for (GLsizei i = 0; i < points_vert_count; ++i) 
            {
                glUniform3f(colorLoc, (i == selected_point) ? 1.0f : 0.0f, (i == selected_point) ? 0.0f : 1.0f, 0.0f);
                glDrawArrays(GL_POINTS, i, 1);
            }

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwTerminate();
    return 0;
}