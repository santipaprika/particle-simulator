#include <Scene.hpp>
#include <Utils.hpp>

namespace vkr {

Scene::Scene(Device& device) : device{device} {
}

Scene::~Scene() {
}

void Scene::initialize() {
    loadEntities();
    loadLights();
    loadCameraSkybox();
    loadParticles();
}

void Scene::loadEntities() {
    std::string textures_path(TEXTURES_PATH);
    std::shared_ptr<Texture> texture = Texture::createTextureFromFile(device, (textures_path + "/test.jpg"));
    std::shared_ptr<Material> material = std::make_shared<Material>(texture);

    std::shared_ptr<Texture> blankTexture = Texture::createTextureFromFile(device, (textures_path + "/blank.jpg"));
    std::shared_ptr<Material> blankMaterial = std::make_shared<Material>(blankTexture);

    std::string models_path(MODELS_PATH);

    // Mesh Entities
    std::shared_ptr<Mesh> mesh =
        Mesh::createModelFromFile(device, (models_path + "/flat_vase.obj").c_str());
    auto flatVase = Entity::createEntity();
    flatVase.mesh = mesh;
    flatVase.material = material;
    flatVase.transform.translation = {-.5f, .5f, 2.5f};
    flatVase.transform.scale = {1.5f, 1.5f, 1.5f};
    entities.push_back(std::move(flatVase));

    mesh = Mesh::createModelFromFile(device, (models_path + "/smooth_vase.obj").c_str());
    auto smoothVase = Entity::createEntity();
    smoothVase.mesh = mesh;
    smoothVase.material = material;
    smoothVase.transform.translation = {.5f, .5f, 2.5f};
    smoothVase.transform.scale = {1.5f, 1.5f, 1.5f};
    entities.push_back(std::move(smoothVase));

    // Floor
    mesh = Mesh::createModelFromFile(device, (models_path + "/cube.obj").c_str());
    auto floor = Entity::createEntity();
    floor.mesh = mesh;
    floor.material = blankMaterial;
    floor.transform.translation = {0.f, 0.f, 0.f};
    floor.transform.scale = {1.5f, 0.01f, 1.5f};
    entities.push_back(std::move(floor));

    // Hair Entities
    auto hairEntity = Entity::createEntity();
    hairEntity.hair = std::make_shared<Hair>(device, (models_path + "/wWavy.hair").c_str());

    // TO DO: Might not have a material, support multiple descriptor set layouts!
    hairEntity.material = material;
    hairEntity.transform.translation = {0.f, 2.f, 2.5f};
    hairEntity.transform.scale = {0.03f, 0.03f, 0.03f};
    hairEntity.transform.rotation = {PI_2, PI_2, 0};

    entities.push_back(std::move(hairEntity));
}

void Scene::loadLights() {
    // Light Entities
    auto mainLight = Entity::createEntity();
    Light light{1.f, glm::vec3{1.f, 1.f, 1.f}};
    mainLight.light = std::make_shared<Light>(1.f, glm::vec3(1.f, 1.f, 1.f));
    mainLight.transform.translation = {0.f, 2.f, 0.f};
    mainLight.transform.rotation = {PI_2, PI_2, 0};

    lights.push_back(std::move(mainLight));
}

void Scene::loadCameraSkybox() {
    mainCamera.setViewTarget(glm::vec3(0.f, 2.f, -2.f), glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f));
    mainCamera.loadSkybox(device);
}

void Scene::loadParticles() {
    // One particle

    std::string textures_path(TEXTURES_PATH);
    std::shared_ptr<Texture> texture = Texture::createTextureFromFile(device, (textures_path + "/test.jpg"));
    std::shared_ptr<Material> material = std::make_shared<Material>(texture);

    std::string models_path(MODELS_PATH);
    std::shared_ptr<Mesh> mesh =
        Mesh::createModelFromFile(device, (models_path + "/sphere.obj").c_str());

    // ParticleSystem particleSystem;
    // particleSystem.setParticleSystem(10);
    // particleSystem.iniParticleSystem(ParticleSystem::ParticleSystemType::Fountain);

    // for (int i = 0; i < 10; i++) {
    //     auto p = Entity::createEntity();
    //     p.transform.translation = particleSystem.getParticle(i).getCurrentPosition();
    //     p.transform.scale = glm::vec3(0.1f, 0.1f, 0.1f);
    // }
    // for (int i = 0; i < 10; i++) {
        auto p = Entity::createEntity();
        p.particle = std::make_shared<Particle>(0.0f, 2.0f, 0.0f);
        p.particle->setLifetime(7.0f);
        //	p.setFixed(true);
        p.particle->setBouncing(0.8f);
        p.particle->addForce(0, -9.8f, 0);

        p.mesh = mesh;
        p.material = material;
        p.transform.translation = {0.0f, 10.0f, 0.0f};
        p.transform.scale = {0.1f, 0.1f, 0.1f};
        entities.push_back(std::move(p));
    // }
}

void Scene::updateScene(float dt) {
    for (auto& entity : entities)
        entity.update(dt);
}

}  // namespace vkr