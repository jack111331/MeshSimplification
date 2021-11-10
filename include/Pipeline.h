//
// Created by Edge on 2020/12/23.
//

#ifndef RAY_PIPELINE_H
#define RAY_PIPELINE_H

#include <stdint.h>
#include <vector>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include "Camera.h"
#include "OpenMeshDef.h"

struct ObjectInfo {
    uint32_t m_vao;
    int m_indicesAmount;
};


struct ShadeObject {
    ShadeObject(Tri_Mesh *mesh, const glm::mat4& transformMat = glm::mat4(1.0)): m_mesh(mesh), m_transformMat(transformMat) {}
    Tri_Mesh *m_mesh;
    glm::mat4 m_transformMat;
};

class ShaderProgram;

class PassSetting {
public:
    // TODO can switch to UBO to pass
    glm::mat4 m_projectionMatrix;
    glm::mat4 m_viewMatrix;
    Camera *m_camera;
};

// Pass Tree
class Pass {
public:
    Pass(PassSetting *passSetting) : m_shader(nullptr), m_passSetting(passSetting), m_outputFrameBufferId(0) {

    }

    void setupPassSetting(PassSetting *setting) {
        m_passSetting = setting;
    }

    virtual void renderPass(const std::vector<ShadeObject *> &shadingList) {
        for (auto pass: m_requirePass) {
            // TODO filter flag to filter what shadingList should be seperate to shade
            // Assume each pass is configured beforehand
            pass->renderPass(shadingList);
        }
    };

    uint32_t getOutputFrameBuffer() {
        return m_outputFrameBufferId;
    }

    void addRequirePass(Pass *pass) {
        m_requirePass.push_back(pass);
    }

    virtual void specifyInput(size_t inputId, uint32_t textureId) {}

    virtual uint32_t getOutputFrameTexture(size_t frameId) = 0;

    virtual std::string getType() = 0;

protected:
    ShaderProgram *m_shader;
    uint32_t m_outputFrameBufferId;
    PassSetting *m_passSetting;
    std::vector<Pass *> m_requirePass;
};

class Pipeline {
public:
    Pipeline() : m_window(nullptr), m_camera(nullptr) {}

    void setupCamera(Camera *camera) {
        m_camera = camera;
    }

    virtual void setupEnvironment();

    virtual void setupPipeline() = 0;

    virtual void pipelineRender() = 0;

    virtual void pipelineLoop() = 0;

    Camera *m_camera;

    std::vector<ShadeObject *> m_objectList;

protected:
    void printGraphicCardInfo() {
        const GLubyte *vendor = glGetString(GL_VENDOR); // Returns the vendor
        const GLubyte *renderer = glGetString(GL_RENDERER);
        std::cout << "Current Vendor: " << (const char *) vendor << std::endl;
        std::cout << "Current Renderer: " << (const char *) renderer << std::endl;
    }

    void setupGUIEnvironment();

    virtual void setupGUILayout() = 0;

    static const char GLSL_VERSION[];


    GLFWwindow *m_window;
};

class LocalRenderingPipeline : public Pipeline {
public:
    virtual void pipelineRender();

    virtual void pipelineLoop();

protected:
    virtual void renderAllPass() = 0;

    virtual void setupGUILayout();

    virtual void blitFrameBuffer() {};

    Pass *m_shadingPass;

};

#endif //RAY_PIPELINE_H
