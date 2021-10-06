#pragma once

#include <Camera.hpp>
#include <Entity.hpp>
#include <Texture.hpp>

namespace vkr {

class Scene {
   public:
    Scene(Device& device);
    ~Scene();

    // Initialization
    void initialize();
    void loadEntities();
    void loadLights();
    void loadCameraSkybox();
    void loadParticles();

    // Getters
    std::vector<Entity>& getEntities() { return entities; }
    std::vector<Entity>& getLightEntities() { return lights; }
    Camera& getMainCamera() { return mainCamera; }

    // Update
    void updateScene(float dt, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout);
    void spawnParticles(std::shared_ptr<ParticleSystem> particleSystem);

   private:
    std::vector<Entity> entities;
    std::vector<Entity> lights;
    std::vector<Texture> textures;
    Camera mainCamera;

    Device& device;
};

}  // namespace vkr
