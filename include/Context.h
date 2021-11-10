//
// Created by Edge on 2021/7/12.
//

#ifndef RAY_CONTEXT_H
#define RAY_CONTEXT_H

#include "Pipeline.h"
#include "OpenMeshDef.h"

class Context {
public:
    static Context *getInstance();

    Pipeline *getPipeline() {
        return m_pipeline;
    }

    void setPipeline(Pipeline *pipeline) {
        m_pipeline = pipeline;
    }

    Tri_Mesh *m_mesh = nullptr;

private:
    Context() : m_pipeline(nullptr) {}
    static Context *m_instance;
    Pipeline *m_pipeline;
};

#endif //RAY_CONTEXT_H
