#include <Material.hpp>

namespace vkr {

Material::Material(std::shared_ptr<Texture> albedo, glm::vec4 diffuseColor) : _albedo{albedo}, _diffuseColor{diffuseColor} {
}

Material::~Material() {
}

}  // namespace vkr