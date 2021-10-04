#pragma once

#include <Buffer.hpp>

#include <glm/glm.hpp>

//std
#include <memory>

namespace vkr {
struct Light {
    Light(float intensity, glm::vec3 color) : intensity{intensity}, color{color} {}
    float intensity{1.0};
    glm::vec3 color{1.f, 1.f, 1.f};

    std::unique_ptr<Buffer> uniformBuffer = nullptr;
};

}  // namespace vkr