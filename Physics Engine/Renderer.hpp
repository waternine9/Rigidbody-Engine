#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

GLenum glCheckError_(const char *file, int line)
{
    GLenum errorCode;
    while ((errorCode = glGetError()) != GL_NO_ERROR)
    {
        std::string error;
        switch (errorCode)
        {
            case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
            case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
            case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
            case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
            case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
            case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
            case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
        }
        std::cout << error << " | " << file << " (" << line << ")" << std::endl;
    }
    return errorCode;
}
#define glCheckError() glCheckError_(__FILE__, __LINE__) 

class Shader
{
private:
    unsigned int _id = -1;
    void checkCompileErrors(GLuint shader, std::string type)
    {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
public:
    void Load(const char* vertexShaderPath, const char* fragmentShaderPath)
    {
        
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        vShaderFile.open(vertexShaderPath);
        fShaderFile.open(fragmentShaderPath);
        std::stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();		
        vShaderFile.close();
        fShaderFile.close();
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();
        
        unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(vertexShader, 1, &vShaderCode, NULL);
        glCompileShader(vertexShader);
        checkCompileErrors(vertexShader, "VERTEX");
        glShaderSource(fragmentShader, 1, &fShaderCode, NULL);
        glCompileShader(fragmentShader);
        checkCompileErrors(fragmentShader, "FRAGMENT");
        
        _id = glCreateProgram();
        
        glAttachShader(_id, vertexShader);
        glAttachShader(_id, fragmentShader);
        glLinkProgram(_id);
        checkCompileErrors(_id, "PROGRAM");
    
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        std::cout << "LOADED SHADER ID " << _id << "\n";

    }

    void Use()
    {
        glUseProgram(_id);
    }
    void SetUniform1f(const char* name, float x)
    {
        int loc = glGetUniformLocation(_id, name);
        glUniform1f(loc, x);
    }
    void SetUniform2f(const char* name, float x, float y)
    {
        int loc = glGetUniformLocation(_id, name);
        glUniform2f(loc, x, y);
    }
    void SetUniform3f(const char* name, float x, float y, float z)
    {
        int loc = glGetUniformLocation(_id, name);
        glUniform3f(loc, x, y, z);
    }
    void SetUniform4f(const char* name, float x, float y, float z, float w)
    {
        int loc = glGetUniformLocation(_id, name);
        glUniform4f(loc, x, y, z, w);
    }
    void SetUniformMat4(const char* name, float* data, const char* debug = nullptr)
    {
        int loc = glGetUniformLocation(_id, name);
        if (loc == -1) std::cout << "ERROR IN SET UNIFORM MAT4 " << (debug ? debug : " NO DEBUG INFO PROVIDED") << std::endl;
        glUniformMatrix4fv(loc, 1, GL_FALSE, data);
    }
    unsigned int GetUniformLocation(const char* name)
    {
        return glGetUniformLocation(_id, name);
    }
};

class RendererModel
{
private:
    std::vector<PhysVector3*> _bindedVerts;
    std::vector<int> _indices;
    std::vector<PhysVector2> _uvs;
    std::vector<PhysVector3> _normals;
    unsigned int _vao;
    unsigned int _vbo;
    unsigned int _ebo;
    unsigned int _texture;
    PhysQuaternion* _bindedRot;
public:
    void Init(std::vector<PhysVector3> *verts, std::vector<PhysVector2> &uvs, std::vector<PhysVector3> &normals, std::vector<int> &indices, PhysQuaternion *rotToBind, const char* texturePath)
    {
        _bindedRot = rotToBind;
        _bindedVerts = std::vector<PhysVector3*>();
        _indices = std::vector<int>();

        int width, height, channels;
        void* data = stbi_load(texturePath, &width, &height, &channels, 0);
        
        
        std::cout << width << " " << height << " " << channels << "\n";
        
        glGenTextures(1, &_texture);
        glBindTexture(GL_TEXTURE_2D, _texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        stbi_image_free(data);

        _indices = indices;
        _uvs = uvs;
        _normals = normals;
        for (PhysVector3 &v : *verts)
        {
            _bindedVerts.push_back(&v);
        }
        std::vector<float> vertices;
        int i = -1;
        for (PhysVector3 *v : _bindedVerts)
        {
            i++;
            vertices.push_back(v->x);
            vertices.push_back(v->y);
            vertices.push_back(v->z);
            vertices.push_back(uvs[i].x);
            vertices.push_back(uvs[i].y);
            PhysVector3 normal = (*_bindedRot) * normals[i];
            vertices.push_back(normal.x);
            vertices.push_back(normal.y);
            vertices.push_back(normal.z);
        }
        glGenVertexArrays(1, &_vao);
        glGenBuffers(1, &_vbo);
        glGenBuffers(1, &_ebo);
        // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
        glBindVertexArray(_vao);

        glBindBuffer(GL_ARRAY_BUFFER, _vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }
    void Update()
    {
        std::vector<float> vertices;
        int i = 0;
        for (PhysVector3 *v : _bindedVerts)
        {
            vertices.push_back(v->x);
            vertices.push_back(v->y);
            vertices.push_back(v->z);
            vertices.push_back(_uvs[i].x);
            vertices.push_back(_uvs[i].y);
            PhysVector3 normal = (*_bindedRot) * _normals[i];
            vertices.push_back(normal.x);
            vertices.push_back(normal.y);
            vertices.push_back(normal.z);
            i++;
        }
        glBindBuffer(GL_ARRAY_BUFFER, _vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
    }
    void Render(Shader* shader)
    {
        unsigned int texLoc = shader->GetUniformLocation("texture1");
        shader->Use();
        glUniform1i(texLoc, 0);
        glCheckError();
        glActiveTexture(GL_TEXTURE0 + 0); // Texture unit 0
        glCheckError();
        glBindTexture(GL_TEXTURE_2D, _texture);
        glCheckError();

        glBindVertexArray(_vao);
        glCheckError();
        glDrawElements(GL_TRIANGLES, _indices.size(), GL_UNSIGNED_INT, 0);
        glCheckError();
    }
};

struct Camera
{
    glm::vec3 position;
    glm::vec3 target;
    float fov;
};

class Renderer
{
private:
    GLFWwindow* _window;
    std::vector<RendererModel*> _models;
    Shader *_shader = nullptr;
    Shader *_shadowShader = nullptr;
    int _resX, _resY;
    unsigned int _shadowFBO;
    unsigned int _shadowDepthMap;
    const unsigned int SHADOW_WIDTH = 8056, SHADOW_HEIGHT = 8056;

public:
    Camera camera;
    void CreateWindow(const char* name, int resX, int resY)
    {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        _resX = resX;
        _resY = resY;
        _window = glfwCreateWindow(resX, resY, name, NULL, NULL);
        glfwMakeContextCurrent(_window);
    }
    void Init()
    {
        _models = std::vector<RendererModel*>();
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return;
        }
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        _shader = new Shader();
        _shader->Load("shaders/vert.glsl", "shaders/frag.glsl");
        _shadowShader = new Shader();
        _shadowShader->Load("shaders/shadow_vert.glsl", "shaders/shadow_frag.glsl");

        glGenFramebuffers(1, &_shadowFBO);
        glCheckError();
        unsigned int depthMap;
        glGenTextures(1, &_shadowDepthMap);
        glCheckError();
        glBindTexture(GL_TEXTURE_2D, _shadowDepthMap);
        glCheckError();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 
                    SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glCheckError();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glCheckError();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glCheckError();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); 
        glCheckError();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); 
        glCheckError();

        glBindFramebuffer(GL_FRAMEBUFFER, _shadowFBO);
        glCheckError();
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _shadowDepthMap, 0);
        glCheckError();
        glDrawBuffer(GL_NONE);
        glCheckError();
        glReadBuffer(GL_NONE);
        glCheckError();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glCheckError();
        
    }
    void AddModel(std::vector<PhysVector3> *verts, std::vector<PhysVector2> &uvs, std::vector<PhysVector3> &normals, std::vector<int> &indices, PhysQuaternion* rotToBind, const char* texturePath)
    {
        RendererModel* model = new RendererModel();
        model->Init(verts, uvs, normals, indices, rotToBind, texturePath);
        _models.push_back(model);
    }
    void Update()
    {
        for (RendererModel* model : _models)
        {
            model->Update();
        }
    }
    void Render()
    {
        unsigned int textureLoc = _shader->GetUniformLocation("texture1");
        unsigned int depthMapLoc = _shader->GetUniformLocation("shadowTexture");
        
        glm::mat4 lsm = glm::ortho(-40.0f, 40.0f, -40.0f, 40.0f, 1.0f, 800.0f) * glm::lookAt(glm::vec3(-200.0f, -200.0f, -200.0f), { 0.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f });
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glCheckError();
        glBindFramebuffer(GL_FRAMEBUFFER, _shadowFBO);
        glCheckError();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glCheckError();
        _shadowShader->Use();
        _shadowShader->SetUniformMat4("lightMat", glm::value_ptr(lsm));
        glCheckError();
        for (RendererModel* model : _models)
        {
            model->Render(_shadowShader);
        }
        
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glCheckError();
        glViewport(0, 0, _resX, _resY);
        glCheckError();
        glm::mat4 view          = glm::mat4(1.0f);
        glm::mat4 projection    = glm::mat4(1.0f);
        view = glm::lookAt(camera.position, camera.target, { 0.0f, -1.0f, 0.0f });
        projection = glm::perspective(glm::radians(camera.fov), (float)_resX / (float)_resY, 0.1f, 1000.0f);
        glm::mat4 vp = projection * view;
        _shader->Use();
        _shader->SetUniformMat4("vp", glm::value_ptr(vp));
        glCheckError();
        _shader->SetUniformMat4("lightMat", glm::value_ptr(lsm), "BBRUH");
        glCheckError();
        glActiveTexture(GL_TEXTURE1); // Texture unit 1
        glCheckError();
        glBindTexture(GL_TEXTURE_2D, _shadowDepthMap);
        glCheckError();
        glUniform1i(depthMapLoc, 1);
        glCheckError();
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glCheckError();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glCheckError();
        for (RendererModel* model : _models)
        {
            model->Render(_shader);
        }
        glfwSwapBuffers(_window);
        glCheckError();
        glfwPollEvents();
        glCheckError();
    }
    bool IsKeyPressed(int keycode)
    {
        return glfwGetKey(_window, keycode) == GLFW_PRESS;
    }
    bool LoopUntilClosed()
    {
        return !glfwWindowShouldClose(_window);
    }
};