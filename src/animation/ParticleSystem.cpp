#include "ParticleSystem.h"

#include <imgui.h>

#include <Entity.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <Scene.hpp>
#include <random>

#define rand01() ((float)std::rand() / RAND_MAX)

ParticleSystem::ParticleSystem() : m_spawnTime(0.5f), m_force({0.f, -9.8f, 0.f}) {
}

ParticleSystem::~ParticleSystem() {
}

void ParticleSystem::setParticleSystem(int numParticles) {
    // m_particleSystem.resize(numParticles);
    m_numParticles = numParticles;
}

std::shared_ptr<Particle> ParticleSystem::getParticle(int i) {
    return m_particleSystem[i];
}

void ParticleSystem::iniParticleSystem() {
    switch (m_type) {
        case ParticleSystemType::Waterfall: {
            for (int i = 0; i < m_numParticles; i++) {
                std::shared_ptr<Particle> particle = std::make_shared<Particle>();
                particle->setPosition(glm::vec3(0.0f, 3.0f, 0.0f));
                particle->setVelocity((rand01() - 0.5f), 0.f, 0.5f * rand01());
                m_particleSystem.push_back(std::move(particle));
            }
        } break;
        case ParticleSystemType::Fountain: {
            for (int i = 0; i < m_numParticles; i++) {
                std::shared_ptr<Particle> particle = std::make_shared<Particle>();
                particle->setPosition(0.f, 3.f, 0.f);
                particle->setVelocity(2.f * (rand01() - 0.5f), 5.f, 2.f * (rand01() - 0.5f));
                m_particleSystem.push_back(std::move(particle));
            }
        } break;
    }
}

void ParticleSystem::updateInScene(const float& dt, vkr::KinematicEntities& kinematicEntities) {
    for (int i = 0; i < m_particleSystem.size(); i++) {
        if (m_particleSystem[i]->isDead()) {
            m_particleSystem.erase(m_particleSystem.begin() + i);
            i--;
        } else {
            m_particleSystem[i]->updateInScene(dt, kinematicEntities, m_solver);
        }
    }
}

void ParticleSystem::spawnParticles(std::shared_ptr<vkr::Mesh>& mesh, std::shared_ptr<vkr::Material> material, std::vector<std::shared_ptr<vkr::Entity>>& sceneEntities) {
    iniParticleSystem();

    for (int i = 0; i < getNumParticlesPerSpawn(); i++) {
        auto p = std::make_shared<vkr::Entity>(vkr::Entity::createEntity());

        int offset = static_cast<int>(getNumParticles()) - getNumParticlesPerSpawn();
        p->particle = getParticle(i + offset);
        p->transform.translation = p->particle->getCurrentPosition();
        p->transform.scale = glm::vec3(0.05f, 0.05f, 0.05f);
        p->particle->setBouncing(m_bouncing);
        p->particle->setFriction(m_friction);
        p->particle->setForce(m_force);
        p->particle->setSize(0.05f);
        p->mesh = mesh;
        p->material = material;
        p->particle->setLifetime(m_lifetime);
        sceneEntities.push_back(p);
        //	p.setFixed(true);
    }
}

void ParticleSystem::renderUI() {
    ImGui::Text("Force");
    ImGui::SliderFloat3("##force", (float*)&m_force, -20.f, 20.f);
    if (ImGui::IsItemEdited()) {
        for (auto& particle : m_particleSystem) {
            particle->setForce(m_force);
        }
    }
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Bouncing");
    ImGui::SliderFloat("##bouncing", &m_bouncing, 0.00001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto& particle : m_particleSystem) {
            particle->setBouncing(m_bouncing);
        }
    }
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Friction");
    ImGui::SliderFloat("##friction", &m_friction, 0.00001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto& particle : m_particleSystem) {
            particle->setFriction(m_friction);
        }
    }
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Solver");
    const char* items[] = {"Euler Explicit", "Euler Semi-implicit", "Verlet"};
    ImGui::Combo("##solver", (int*)&m_solver, items, IM_ARRAYSIZE(items));
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Particles per Spawn");
    ImGui::SliderInt("##spawnParticles", &m_numParticles, 1, 30);
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Spawn Period");
    ImGui::SliderFloat("##spawnPeriod", &m_spawnTime, 0.2f, 5.f);
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Particle Lifespan");
    ImGui::SliderFloat("##particleLifespan", &m_lifetime, 0.2f, 10.f);
    if (ImGui::IsItemEdited()) {
        for (auto& particle : m_particleSystem) {
            particle->setLifetime(m_lifetime);
        }
    }
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Spawn Type");
    const char* items_type[] = {"Fountain", "Waterfall"};
    ImGui::Combo("##d", (int*)&m_type, items_type, IM_ARRAYSIZE(items_type));
}
