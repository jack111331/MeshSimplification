//
// Created by Edge on 2020/7/8.
//

#ifndef RAY_CAMERA_H
#define RAY_CAMERA_H


#include <glm/glm.hpp>
#include "Utility.h"

class Camera {
public:
    Camera() : m_screen(nullptr), m_width(0), m_height(0) {}
    Camera(int width, int height, Vec3f eyeCoord, double fov, double yaw, double pitch, Vec3f up);
    void toPpm(const std::string &filename) const;
    void initializeScreen();
    void bufferToTexture(uint32_t bufferId) const;

    Vec3f getDirection() {
        double yaw = glm::radians(m_yaw);
        double pitch = glm::radians(m_pitch);
        return Vec3f(cos(pitch) * cos(yaw), sin(pitch), cos(pitch) * sin(yaw));
    }

    void setEyeCoord(const Vec3f &coord) {
        m_eyeCoord = coord;
    }

    void translateForward();
    void translateBackward();

    void translateYaw(float dx, float dy);

    // Need more abstraction, better use adapter to adapt different type of file

    int m_width, m_height;
    Vec3f **m_screen;

    Vec3f m_eyeCoord;
    float m_fov;
    Vec3f m_up;
    double m_yaw, m_pitch;

    static Vec3f mouseTrackBall(float dx, float dy) {
        const float TRACKBALL_R = 0.8f;

        float r2 = dx*dx + dy*dy;
        float t = 0.5f * TRACKBALL_R * TRACKBALL_R;

        Vec3f pos(dx, dy, 0);
        if (r2 < t)
            pos[2] = sqrt(2*t - r2);
        else
            pos[2] = t / sqrt(r2);
        return pos;
    }

};


#endif //RAY_CAMERA_H
