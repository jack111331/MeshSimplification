//
// Created by Edge on 2020/12/31.
//

#include "glad/glad.h"
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <wx/wx.h>
#include "PhongShadingPipeline.h"
#include "ShaderProgram.h"

PhongShadingPass::PhongShadingPass(PassSetting *passSetting) : Pass(passSetting), m_outputFrameTextureId(0),
                                                               m_outputFrameDepthStencilBufferId(0) {
    m_shader = new ShaderProgram(ShaderInclude::load("resource/shader/local_shading/mesh.vs"), ShaderInclude::load("resource/shader/local_shading/face_mesh.fs"));
    m_anotherShader = new ShaderProgram(ShaderInclude::load("resource/shader/local_shading/mesh.vs"), ShaderInclude::load("resource/shader/local_shading/edge_mesh.fs"));
    PhongShadingSetting *phongPassSetting = (PhongShadingSetting *) passSetting;

    glGenFramebuffers(1, &m_outputFrameBufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, m_outputFrameBufferId);

    glGenTextures(1, &m_outputFrameTextureId);
    glBindTexture(GL_TEXTURE_2D, m_outputFrameTextureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, phongPassSetting->m_camera->m_width, phongPassSetting->m_camera->m_height,
                 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_outputFrameTextureId, 0);

    glGenRenderbuffers(1, &m_outputFrameDepthStencilBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, m_outputFrameDepthStencilBufferId);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, phongPassSetting->m_camera->m_width,
                          phongPassSetting->m_camera->m_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                              m_outputFrameDepthStencilBufferId);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Phong shading pass frame buffer object successful setup" << std::endl;
    } else {
        std::cout << "Phong shading pass frame buffer object failed setup" << std::endl;
        exit(1);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void PhongShadingPass::renderPass(const std::vector<ShadeObject *> &shadingList) {
    Pass::renderPass(shadingList);

    PhongShadingSetting *setting = (PhongShadingSetting *) m_passSetting;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    if (setting) {
        // shader binding

        m_shader->bind();
        for (auto object: shadingList) {

            // object binding
            glBindVertexArray(object->m_mesh->m_vao);

            m_shader->uniformMat4f("projection", setting->m_projectionMatrix);
            m_shader->uniformMat4f("view", setting->m_viewMatrix);

            // world transformation
            m_shader->uniformMat4f("model", object->m_transformMat);

            const Vec3f &eyeCoord = setting->m_camera->m_eyeCoord;
            m_shader->uniform3f("viewPos", eyeCoord.x, eyeCoord.y, eyeCoord.z);

            // draw call
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object->m_mesh->m_triFaceEbo);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glEnable(GL_DEPTH_TEST);
            glPolygonOffset(2.0, 2.0);
            glDrawElements(GL_TRIANGLES, object->m_mesh->m_faceIndices, GL_UNSIGNED_INT, 0);
        }
        m_anotherShader->bind();
        for (auto object: shadingList) {

            // object binding
            glBindVertexArray(object->m_mesh->m_vao);

            m_anotherShader->uniformMat4f("projection", setting->m_projectionMatrix);
            m_anotherShader->uniformMat4f("view", setting->m_viewMatrix);

            // world transformation
            m_anotherShader->uniformMat4f("model", object->m_transformMat);

            const Vec3f &eyeCoord = setting->m_camera->m_eyeCoord;
            m_anotherShader->uniform3f("viewPos", eyeCoord.x, eyeCoord.y, eyeCoord.z);

            // draw call
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object->m_mesh->m_edgeEbo);
            glLineWidth(1.2);
            glDrawElements(GL_LINES, object->m_mesh->m_edgeIndices, GL_UNSIGNED_INT, 0);
        }

    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void PhongShadingPipeline::setupPipeline() {
    m_camera = new Camera(800, 600, Vec3f(0, 0, 1), 45, -90, 0, Vec3f(0, 1, 0));
    shadingSetting = new PhongShadingSetting();
    shadingSetting->m_projectionMatrix = glm::perspective(glm::radians(m_camera->m_fov * 2.0f),
                                                          (float) m_camera->m_width / (float) m_camera->m_height,
                                                          0.1f, 100.0f);
    Vec3f direction = m_camera->getDirection();
    shadingSetting->m_viewMatrix = glm::lookAt(
            glm::vec3(m_camera->m_eyeCoord.x, m_camera->m_eyeCoord.y, m_camera->m_eyeCoord.z),
            glm::vec3(m_camera->m_eyeCoord.x + direction.x,
                      m_camera->m_eyeCoord.y + direction.y,
                      m_camera->m_eyeCoord.z + direction.z),
            glm::vec3(m_camera->m_up.x, m_camera->m_up.y, m_camera->m_up.z));
    shadingSetting->m_camera = m_camera;

    m_shadingPass = new PhongShadingPass(shadingSetting);
    m_shadingPass->setupPassSetting(shadingSetting);

}

void PhongShadingPipeline::renderAllPass() {
    Vec3f direction = m_camera->getDirection();

    shadingSetting->m_viewMatrix = glm::lookAt(
            glm::vec3(m_camera->m_eyeCoord.x, m_camera->m_eyeCoord.y, m_camera->m_eyeCoord.z),
            glm::vec3(m_camera->m_eyeCoord.x + direction.x,
                      m_camera->m_eyeCoord.y + direction.y,
                      m_camera->m_eyeCoord.z + direction.z),
            glm::vec3(m_camera->m_up.x, m_camera->m_up.y, m_camera->m_up.z));
    m_shadingPass->renderPass(m_objectList);
//    blitFrameBuffer();
}


void PhongShadingPipeline::blitFrameBuffer() {
    PhongShadingPass *pass = (PhongShadingPass *) m_shadingPass;
    glBindFramebuffer(GL_READ_FRAMEBUFFER,
                      pass->getOutputFrameBuffer());
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glBlitFramebuffer(0, 0, m_camera->m_width, m_camera->m_width, 0, 0, m_camera->m_width, m_camera->m_height,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

