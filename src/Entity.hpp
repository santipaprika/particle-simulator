/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Particle.h>
#include <ParticleSystem.h>

#include <Buffer.hpp>
#include <Hair.hpp>
#include <Light.hpp>
#include <Material.hpp>
#include <Mesh.hpp>

// libs
#include <glm/gtc/matrix_transform.hpp>

// std
#include <memory>

namespace vkr {
struct FrameInfo;
class Scene;

struct TransformComponent {
    glm::vec3 translation{};
    glm::vec3 scale{1.f, 1.f, 1.f};
    glm::vec3 rotation{};

    // Matrix corresponds to Translate * Ry * Rx * Rz * Scale
    // Rotations correspond to Tait-bryan angles of Y(1), X(2), Z(3)
    // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    glm::mat4 mat4();

    glm::mat3 normalMatrix();
};

struct EntityUBO {
    glm::mat4 projectionView;
    glm::mat4 model;
    glm::mat4 normalMatrix;
    glm::vec3 camPos;
};

struct SimplePushConstantData {
    float brightness;
};

class Entity {
   public:
    using id_t = unsigned int;

    static Entity createEntity() {
        static id_t currentId = 0;
        return Entity{currentId++};
    }

    Entity(const Entity&) = delete;
    Entity& operator=(const Entity&) = delete;
    Entity(Entity&&) = default;
    Entity& operator=(Entity&&) = default;

    id_t getId() { return id; }

    bool update(float dt, std::vector<std::shared_ptr<Entity>> &kinematicEntities);
    void render(glm::mat4 camProjectionView, FrameInfo& frameInfo, VkPipelineLayout pipelineLayout);

    TransformComponent transform{};

    // COMPONENTS (Might be a vector?)
    std::shared_ptr<Mesh> mesh{nullptr};
    std::shared_ptr<Hair> hair{nullptr};
    std::shared_ptr<Material> material{nullptr};
    std::shared_ptr<Light> light{nullptr};
    std::shared_ptr<Particle> particle{nullptr};
    std::shared_ptr<ParticleSystem> particleSystem{nullptr};
    std::shared_ptr<Plane> plane{nullptr};
    bool isKinematic{false};

    VkDescriptorSet descriptorSet;

    // Transform specific uniform buffer
    std::vector<std::unique_ptr<Buffer>> uboBuffers;

    void createUniformBuffer(Device& device);
    void updateDescriptorSet(Device& device, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout);

   private:
    Entity(id_t objId) : id{objId} {}

    id_t id;
};

}  // namespace vkr
