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
    initializeKinematicEntities();
}

void Scene::loadEntities() {
    std::string textures_path(TEXTURES_PATH);
    std::shared_ptr<Texture> texture = Texture::createTextureFromFile(device, (textures_path + "/test.jpg"));
    std::shared_ptr<Material> material = std::make_shared<Material>(texture);

    std::shared_ptr<Texture> blankTexture = Texture::createTextureFromFile(device, (textures_path + "/blank.jpg"));
    blankMaterial = std::make_shared<Material>(blankTexture);

    std::string models_path(MODELS_PATH);

    // Mesh Entities
    std::shared_ptr<Mesh> mesh =
        Mesh::createModelFromFile(device, (models_path + "/flat_vase.obj").c_str());
    auto flatVase = std::make_shared<Entity>(Entity::createEntity());
    flatVase->mesh = mesh;
    flatVase->material = material;
    flatVase->transform.translation = {-.5f, .5f, 2.5f};
    flatVase->transform.scale = {1.5f, 1.5f, 1.5f};
    // std::shared_ptr<Entity> flatVaseP = std::make_shared<Entity>(flatVase);
    entities.push_back(flatVase);

    mesh = Mesh::createModelFromFile(device, (models_path + "/smooth_vase.obj").c_str());
    auto smoothVase = std::make_shared<Entity>(Entity::createEntity());
    smoothVase->mesh = mesh;
    smoothVase->material = material;
    smoothVase->transform.translation = {.5f, .5f, 2.5f};
    smoothVase->transform.scale = {1.5f, 1.5f, 1.5f};
    entities.push_back(smoothVase);

    createBox();
}

void Scene::loadLights() {
    // Light Entities
    // auto mainLight = Entity::createEntity();
    // Light light{1.f, glm::vec3{1.f, 1.f, 1.f}};
    // mainLight.light = std::make_shared<Light>(1.f, glm::vec3(1.f, 1.f, 1.f));
    // mainLight.transform.translation = {0.f, 2.f, 0.f};
    // mainLight.transform.rotation = {PI_2, PI_2, 0};

    // lights.push_back(std::move(mainLight));
}

void Scene::loadCameraSkybox() {
    mainCamera.setViewTarget(glm::vec3(0.f, 2.f, -2.f), glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f));
    mainCamera.loadSkybox(device);
}

void Scene::loadParticles() {
    // Create particle system
    auto particleSystemRoot = std::make_shared<Entity>(Entity::createEntity());
    particleSystemRoot->particleSystem = std::make_shared<ParticleSystem>();
    particleSystemRoot->particleSystem->setParticleSystem(10);
    // particleSystemRoot.particleSystem->iniParticleSystem();

    spawnParticles(particleSystemRoot->particleSystem);
    entities.push_back(particleSystemRoot);
}

void Scene::initializeKinematicEntities() {
    for (auto& entity : entities) {
        if (entity->isKinematic) kinematicEntities.push_back(entity);
    }
}

void Scene::updateScene(float dt, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout) {
    for (int i = 0; i < entities.size(); i++) {
        if (!entities[i]->update(dt, kinematicEntities)) {
            vkQueueWaitIdle(device.graphicsQueue());
            vkFreeDescriptorSets(device.device(), descriptorPool, 1, &entities[i]->descriptorSet);
            entities.erase(entities.begin() + i);

            i--;
        }
    }

    static float counter = 0;
    counter += dt;

    for (int i = 0; i < entities.size(); i++) {
        if (entities[i]->particleSystem) {
            if (counter < entities[i]->particleSystem->getSpawnTime()) continue;

            std::shared_ptr<ParticleSystem> particleSystem = entities[i]->particleSystem;
            spawnParticles(particleSystem);

            // Create buffers and sets for the new particle entities (last S entities where S=number of particles per spawn)
            for (size_t j = entities.size() - particleSystem->getNumParticlesPerSpawn(); j < entities.size(); j++) {
                entities[j]->createUniformBuffer(device);
                entities[j]->updateDescriptorSet(device, descriptorPool, descriptorSetLayout);
            }

            counter = 0;
        }
    }
}

void Scene::spawnParticles(std::shared_ptr<ParticleSystem> particleSystem) {
    std::string textures_path(TEXTURES_PATH);

    std::string models_path(MODELS_PATH);
    std::shared_ptr<Mesh> mesh =
        Mesh::createModelFromFile(device, (models_path + "/sphere.obj").c_str());

    particleSystem->iniParticleSystem();

    for (int i = 0; i < particleSystem->getNumParticlesPerSpawn(); i++) {
        auto p = std::make_shared<Entity>(Entity::createEntity());

        int offset = static_cast<int>(particleSystem->getNumParticles()) - particleSystem->getNumParticlesPerSpawn();
        p->particle = std::make_shared<Particle>(particleSystem->getParticle(i + offset));
        p->transform.translation = p->particle->getCurrentPosition();
        p->transform.scale = glm::vec3(0.1f, 0.1f, 0.1f);
        p->particle->setBouncing(0.8f);
        p->particle->addForce(0, -9.8f, 0);
        p->mesh = mesh;
        p->material = blankMaterial;
        p->particle->setLifetime(7.0f);
        entities.push_back(p);
        //	p.setFixed(true);
    }
}

void Scene::createBox() {
    std::string models_path(MODELS_PATH);
    std::shared_ptr<Mesh> mesh = Mesh::createModelFromFile(device, (models_path + "/cube.obj").c_str());
    float particleSize = 0.1f;
    float wallSize = 3.f;

    // Floor ---------------------------------------------
    auto floor = std::make_shared<Entity>(Entity::createEntity());
    floor->mesh = mesh;
    floor->material = blankMaterial;
    floor->transform.translation = {0.f, 0.f, 0.f};
    floor->transform.scale = {wallSize, 0.01f, wallSize};

    std::shared_ptr<Plane> planeFloor = std::make_shared<Plane>();
    planeFloor->setPlaneNormal(0, 1, 0);
    planeFloor->setPlanePoint(0.f, particleSize, 0.f);

    floor->plane = planeFloor;
    floor->isKinematic = true;

    // Left Wall ---------------------------------------------
    auto leftWall = std::make_shared<Entity>(Entity::createEntity());
    leftWall->mesh = mesh;
    leftWall->material = blankMaterial;
    leftWall->transform.translation = {wallSize, wallSize, 0.f};
    leftWall->transform.scale = {0.01f, wallSize, wallSize};

    std::shared_ptr<Plane> planeLeft = std::make_shared<Plane>();
    planeLeft->setPlaneNormal(-1.f, 0.f, 0.f);
    planeLeft->setPlanePoint(floor->transform.scale.x - particleSize, 0.f, 0.);

    leftWall->plane = planeLeft;
    leftWall->isKinematic = true;

    // Right Wall ---------------------------------------------
    auto rightWall = std::make_shared<Entity>(Entity::createEntity());
    rightWall->mesh = mesh;
    rightWall->material = blankMaterial;
    rightWall->transform.translation = {-wallSize, wallSize, 0.f};
    rightWall->transform.scale = {0.01f, wallSize, wallSize};

    std::shared_ptr<Plane> planeRight = std::make_shared<Plane>();
    planeRight->setPlaneNormal(1, 0, 0);
    planeRight->setPlanePoint(-floor->transform.scale.x + particleSize, 0.f, 0.f);

    rightWall->plane = planeRight;
    rightWall->isKinematic = true;

    // Front Wall ---------------------------------------------
    auto frontWall = std::make_shared<Entity>(Entity::createEntity());
    frontWall->mesh = mesh;
    frontWall->material = blankMaterial;
    frontWall->transform.translation = {0.f, wallSize, wallSize};
    frontWall->transform.scale = {wallSize, wallSize, 0.01f};

    std::shared_ptr<Plane> planeFront = std::make_shared<Plane>();
    planeFront->setPlaneNormal(0, 0, -1);
    planeFront->setPlanePoint(0.f, 0.f, floor->transform.scale.x - particleSize);

    frontWall->plane = planeFront;
    frontWall->isKinematic = true;

    // Back Wall ---------------------------------------------
    auto backWall = std::make_shared<Entity>(Entity::createEntity());
    backWall->mesh = mesh;
    backWall->material = blankMaterial;
    backWall->transform.translation = {0.f, wallSize, -wallSize};
    backWall->transform.scale = {wallSize, wallSize, 0.01f};

    std::shared_ptr<Plane> planeBack = std::make_shared<Plane>();
    planeBack->setPlaneNormal(0, 0, 1);
    planeBack->setPlanePoint(0.f, 0.f, -floor->transform.scale.x + particleSize);

    backWall->plane = planeBack;
    backWall->isKinematic = true;

    entities.push_back(floor);
    entities.push_back(leftWall);
    entities.push_back(rightWall);
    entities.push_back(frontWall);
    entities.push_back(backWall);
}

}  // namespace vkr