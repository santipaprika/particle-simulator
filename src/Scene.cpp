#include <imgui.h>

#include <Mesh.hpp>
#include <Scene.hpp>
#include <Utils.hpp>

namespace vkr {

Scene::Scene(Device& device) : device{device} {
}

Scene::~Scene() {
}

void Scene::initialize() {
    loadEntities(SCENE3);
    loadLights();
    loadCameraSkybox();
    initializeKinematicEntities();
}

void Scene::loadEntities(Scenario scenario) {
    std::string textures_path(TEXTURES_PATH);

    std::shared_ptr<Texture> blankTexture = Texture::createTextureFromFile(device, (textures_path + "/blank.jpg"));
    std::shared_ptr<Material> material = std::make_shared<Material>(blankTexture);
    material->setDiffuseColor(glm::vec4(0.f, 1.f, 0.f, 1.f));
    blankMaterial = std::make_shared<Material>(blankTexture);

    // Mesh Entities
    std::string models_path(MODELS_PATH);
    sphereMesh = Mesh::createModelFromFile(device, (models_path + "/sphere.obj").c_str());

    // Walls
    createBox();

    // Sphere
    std::shared_ptr<Mesh> mesh = Mesh::createModelFromFile(device, (models_path + "/sphere.obj").c_str());
    auto sphere = std::make_shared<Entity>(Entity::createEntity());
    sphere->mesh = mesh;
    sphere->material = material;
    sphere->transform.translation = {0.f, 2.f, 0.f};
    sphere->transform.scale = {0.8f, 0.8f, 0.8f};
    sphere->colliderType = Entity::ColliderType::SPHERE;
    entities.push_back(sphere);

    particleMaterial = std::make_shared<Material>(blankTexture);
    particleMaterial->setDiffuseColor(glm::vec4(1.f, 0.f, 0.f, 1.f));

    switch (scenario) {
        case SCENE1: {
            // Triangle
            std::shared_ptr<Mesh> mesh = Mesh::createTriangle(device);
            auto triangle = std::make_shared<Entity>(Entity::createEntity());
            triangle->mesh = mesh;
            triangle->material = material;
            triangle->transform.translation = {1.7f, 1.f, 0.f};
            triangle->transform.scale = {2.f, 2.f, 2.f};
            // triangle->transform.rotation = {0.f, 0.f, PI_2/3.f};
            triangle->colliderType = Entity::ColliderType::TRIANGLE;
            entities.push_back(triangle);

            auto triangle2 = std::make_shared<Entity>(Entity::createEntity());
            triangle2->mesh = mesh;
            triangle2->material = material;
            triangle2->transform.translation = {-1.7f, 1.f, 0.f};
            triangle2->transform.scale = {2.f, 2.f, 2.f};
            // triangle2->transform.rotation = {0.f, 0.f, -PI_2/3.f};
            triangle2->colliderType = Entity::ColliderType::TRIANGLE;
            entities.push_back(triangle2);

            loadParticles();

        } break;

        case SCENE2: {
            // Hair
            auto hairEntity = std::make_shared<Entity>(Entity::createEntity());
            hairEntity->hair = std::make_shared<Hair>(device, (models_path + "/wWavy.hair").c_str());

            hairEntity->material = material;
            hairEntity->transform.translation = {0.f, 4.5f, 0.f};
            hairEntity->transform.scale = {0.03f, 0.03f, 0.03f};
            hairEntity->transform.rotation = {PI_2, PI_2, 0};

            hairEntity->hair->loadParticles(hairEntity->transform);
            for (int i = 0; i < (hairEntity->hair->builder.defaultSegments + 1) * hairEntity->hair->numStrands; i++) {
                auto p = std::make_shared<Entity>(Entity::createEntity());
                p->particle = hairEntity->hair->builder.verticesParticles[i];
                p->transform.translation = p->particle->getCurrentPosition();
                p->transform.scale = glm::vec3(0.02f, 0.02f, 0.02f);
                p->mesh = mesh;
                p->material = particleMaterial;
                entities.push_back(std::move(p));
            }

            entities.push_back(std::move(hairEntity));

        } break;

        case SCENE3: {
            // Cloth
            auto clothEntity = std::make_shared<Entity>(Entity::createEntity());
            clothEntity->cloth = std::make_shared<Cloth>(device, 10, 0.2f);

            clothEntity->material = material;
            clothEntity->transform.translation = {-1.f, 4.5f, 0.f};
            // clothEntity->transform.scale = {1.f,1.f,};
            // clothEntity->transform.rotation = {PI_2, PI_2, 0};

            clothEntity->cloth->loadParticles(clothEntity->transform);
            for (int i = 0; i < clothEntity->cloth->builder.vertices.size(); i++) {
                auto p = std::make_shared<Entity>(Entity::createEntity());
                p->particle = clothEntity->cloth->builder.verticesParticles[i];
                p->transform.translation = p->particle->getCurrentPosition();
                p->transform.scale = glm::vec3(0.02f, 0.02f, 0.02f);
                p->mesh = mesh;
                p->material = particleMaterial;
                entities.push_back(std::move(p));
            }

            entities.push_back(std::move(clothEntity));
        } break;
        default: {
            break;
        }
    }
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
    particleSystemRoot->particleSystem->setType(ParticleSystem::ParticleSystemType::Waterfall);

    particleSystemRoot->particleSystem->spawnParticles(sphereMesh, particleMaterial, entities);
    entities.push_back(particleSystemRoot);
    particleSystemEntities.push_back(particleSystemRoot);
}

void Scene::initializeKinematicEntities() {
    float particleSize = 0.02f;

    for (auto& entity : entities) {
        if (entity->colliderType == Entity::ColliderType::NONE) continue;

        std::vector<Mesh::Vertex> vertices = entity->mesh->getBuilder().vertices;
        switch (entity->colliderType) {
            case Entity::ColliderType::PLANE:
                kinematicEntities.kinematicPlaneEntities.push_back(entity);
                break;
            case Entity::ColliderType::TRIANGLE:
                // only for non-indexed single-triangle meshes
                for (int i = 0; i < 3; i += 3) {
                    glm::vec3 normal = glm::normalize(entity->transform.normalMatrix() * glm::normalize(
                                                                                             glm::cross(vertices[i + 2].position - vertices[i].position,
                                                                                                        vertices[i + 1].position - vertices[i].position)));
                    std::shared_ptr<Plane> trianglePlane = std::make_shared<Plane>();
                    trianglePlane->setPlaneNormal(normal.x, normal.y, normal.z);
                    glm::vec3 offset = particleSize * normal * 0.99f;

                    // Front face
                    entity->triangleColliderVertices[0] = offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[i].position, 1.f));
                    entity->triangleColliderVertices[1] = offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[i + 1].position, 1.f));
                    entity->triangleColliderVertices[2] = offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[i + 2].position, 1.f));
                    // Back face
                    entity->triangleColliderVertices[3] = -offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[2 * i].position, 1.f));
                    entity->triangleColliderVertices[4] = -offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[2 * i + 1].position, 1.f));
                    entity->triangleColliderVertices[5] = -offset + glm::vec3(entity->transform.mat4() * glm::vec4(vertices[2 * i + 2].position, 1.f));

                    trianglePlane->setPlanePoint(entity->triangleColliderVertices[0].x,
                                                 entity->triangleColliderVertices[0].y,
                                                 entity->triangleColliderVertices[0].z);
                    entity->plane = trianglePlane;

                    std::shared_ptr<Plane> triangleBackfacePlane = std::make_shared<Plane>();
                    triangleBackfacePlane->setPlaneNormal(-normal.x, -normal.y, -normal.z);
                    triangleBackfacePlane->setPlanePoint(entity->triangleColliderVertices[3].x,
                                                         entity->triangleColliderVertices[3].y,
                                                         entity->triangleColliderVertices[3].z);
                    entity->backfacePlane = triangleBackfacePlane;
                }

                kinematicEntities.kinematicTriangleEntities.push_back(entity);
                break;

            case Entity::ColliderType::SPHERE:  // must be centered sphere and regularly scaled
                kinematicEntities.kinematicSphereEntities.push_back(entity);
                break;

            default:
                break;
        }
    }
}

void Scene::updateScene(float dt, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout) {
    for (auto& particleSystemEntity : particleSystemEntities) {
        particleSystemEntity->particleSystem->updateInScene(dt, kinematicEntities);
    }

    for (int i = 0; i < entities.size(); i++) {
        if (!entities[i]->update(dt)) {
            // remove entity
            vkQueueWaitIdle(device.graphicsQueue());
            vkFreeDescriptorSets(device.device(), descriptorPool, 1, &entities[i]->descriptorSet);
            entities.erase(entities.begin() + i);

            i--;
        }
    }

    static float counter = 0;
    counter += dt;

    for (auto& particleSystemEntity : particleSystemEntities) {
        if (counter < particleSystemEntity->particleSystem->getSpawnTime()) continue;

        std::shared_ptr<ParticleSystem> particleSystem = particleSystemEntity->particleSystem;
        particleSystem->spawnParticles(sphereMesh, particleMaterial, entities);

        // Create buffers and sets for the new particle entities (last S entities where S=number of particles per spawn)
        for (size_t j = entities.size() - particleSystem->getNumParticlesPerSpawn(); j < entities.size(); j++) {
            entities[j]->createUniformBuffer(device);
            entities[j]->updateDescriptorSet(device, descriptorPool, descriptorSetLayout);
        }

        counter = 0;
    }
    for (auto& entity : entities) {
        if (entity->hair) {
            entity->hair->update(dt, kinematicEntities, entity->transform);
        } else if (entity->cloth) {
            entity->cloth->update(dt, kinematicEntities, entity->transform);
        }
    }
}

void Scene::renderUI() {
    for (auto& entity : entities) {
        if (entity->hair) {
            ImGui::Checkbox("Show Particles", &showParticles);
            entity->hair->renderUI();

            // ImGui::Separator();  // --------------
            // ImGui::Text("Position");
            // ImGui::SliderFloat3("##pos", (float*)&entity->transform.translation, -5.5f, 5.5f);
            // if (ImGui::IsItemEdited()) {
            //     for (int i=0; i<entity->hair->builder.verticesParticles.size(); i++) {
            //         std::shared_ptr<Particle> particle = entity->hair->builder.verticesParticles[i];
            //         particle->setPreviousPosition(particle->getCurrentPosition());
            //         particle->setPosition(entity->transform.mat4() * glm::vec4(entity->hair->builder.vertices[i].position,1.f));
            //     }
            // }
        }

        if (entity->cloth) {
            ImGui::Checkbox("Show Particles", &showParticles);
            entity->cloth->renderUI();
        }
    }

    for (auto& particleSystemEntity : particleSystemEntities) {
        particleSystemEntity->particleSystem->renderUI();
    }
}

void Scene::createBox() {
    std::string models_path(MODELS_PATH);
    std::shared_ptr<Mesh> mesh = Mesh::createModelFromFile(device, (models_path + "/cube.obj").c_str());
    float particleSize = 0.02f;
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
    floor->colliderType = Entity::ColliderType::PLANE;

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
    leftWall->colliderType = Entity::ColliderType::PLANE;

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
    rightWall->colliderType = Entity::ColliderType::PLANE;

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
    frontWall->colliderType = Entity::ColliderType::PLANE;

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
    backWall->colliderType = Entity::ColliderType::PLANE;

    entities.push_back(floor);
    entities.push_back(leftWall);
    entities.push_back(rightWall);
    entities.push_back(frontWall);
    entities.push_back(backWall);
}

}  // namespace vkr