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
    void initializeKinematicEntities();

    // Getters
    std::vector<std::shared_ptr<Entity>>& getEntities() { return entities; }
    std::vector<std::shared_ptr<Entity>>& getLightEntities() { return lights; }
    Camera& getMainCamera() { return mainCamera; }

    // Update
    void updateScene(float dt, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout);
    void spawnParticles(std::shared_ptr<ParticleSystem> particleSystem);

   private:
    std::vector<std::shared_ptr<Entity>> entities;
    std::vector<std::shared_ptr<Entity>> kinematicEntities;
    std::vector<std::shared_ptr<Entity>> lights;
    std::vector<Texture> textures;
    Camera mainCamera;

    Device& device;

    std::shared_ptr<Material> blankMaterial;
    void createBox();
};

}  // namespace vkr
