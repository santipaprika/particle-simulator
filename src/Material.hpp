#pragma once

#include <Texture.hpp>

// libs
#include <glm/glm.hpp>
#include <memory>

namespace vkr {

class Material {
   public:
    Material(std::shared_ptr<Texture> albedo, glm::vec4 diffuseColor = glm::vec4(1.f,1.f,1.f,1.f));
    ~Material();

    std::shared_ptr<Texture> getAlbedo() { return (_albedo); }
    bool hasAlbedo() { return _albedo != nullptr; }
    glm::vec4 getDiffuseColor() { return _diffuseColor; }
    void setDiffuseColor(glm::vec4 color) { _diffuseColor = color; }

   private:
    std::shared_ptr<Texture> _albedo {nullptr};
    glm::vec4 _diffuseColor;
};

}  // namespace vkr