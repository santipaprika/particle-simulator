#pragma once

#include <Camera.hpp>
#include <Entity.hpp>
#include <Texture.hpp>
#include <InputController.hpp>
#include <Window.hpp>

namespace vkr {

struct KinematicEntities {
    std::vector<std::shared_ptr<Entity>> kinematicPlaneEntities;
    std::vector<std::shared_ptr<Entity>> kinematicTriangleEntities;
    std::vector<std::shared_ptr<Entity>> kinematicSphereEntities;
};

class Scene {
   public:
    enum Scenario { SCENE1, SCENE2, SCENE3 };

    Scene(Device& device, Window& window);
    ~Scene();

    // Initialization
    void initialize();
    void loadEntities(Scenario scenario = SCENE2);
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
    void renderUI();

    void reset();
    bool showParticles{false};

   private:
    std::vector<std::shared_ptr<Entity>> entities;
    std::vector<std::shared_ptr<Entity>> particleSystemEntities;
    KinematicEntities kinematicEntities;
    std::vector<std::shared_ptr<Entity>> lights;
    std::vector<Texture> textures;
    Camera mainCamera;

    Device& device;
    Window& window;

    std::shared_ptr<Material> blankMaterial;
    std::shared_ptr<Material> particleMaterial;

    std::shared_ptr<Mesh> sphereMesh;
    void createBox();

    InputController inputController;

};

}  // namespace vkr
