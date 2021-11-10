//
// Created by Edge on 2020/12/26.
//

#ifndef RAY_PIPELINEFACTORY_H
#define RAY_PIPELINEFACTORY_H


#include "Pipeline.h"
#include "PhongShadingPipeline.h"

class PipelineFactory {
public:
    static Pipeline *generatePipeline(const std::string &typeName, const std::string &pipelineName) {
        if (typeName == "local-rendering") {
            if (pipelineName == "phong-shading") {
                return new PhongShadingPipeline();
            }
        }
        return nullptr;
    }
};


#endif //RAY_PIPELINEFACTORY_H
