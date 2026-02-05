#pragma once
#include <glm/glm.hpp>

// The uniform buffer object used in this example
struct UniformBufferObject {
    alignas(16) glm::mat4 mvpMat;
    alignas(16) glm::mat4 mMat;
    alignas(16) glm::mat4 nMat;
};

struct GlobalUniformBufferObject {
    alignas(16) glm::vec3 lightDir;
    alignas(16) glm::vec4 lightColor;
    alignas(16) glm::vec3 eyePos;
    alignas(16) glm::vec4 eyeDir;

    //  traffic light state
    // 0 = RED, 1 = YELLOW, 2 = GREEN
    alignas(16) glm::ivec4 traffic; // traffic.x = state (rest unused)
};
