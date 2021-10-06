#include "ParticleSystem.h"

#include <random>

#define rand01() ((float)std::rand() / RAND_MAX)

ParticleSystem::ParticleSystem() : m_spawnTime(0.5f) {
}

ParticleSystem::~ParticleSystem() {
}

void ParticleSystem::setParticleSystem(int numParticles) {
    m_particleSystem.resize(numParticles);
    m_numParticles = numParticles;
}

Particle ParticleSystem::getParticle(int i) {
    return m_particleSystem[i];
}

void ParticleSystem::iniParticleSystem() {
    switch (type) {
        case ParticleSystemType::Waterfall: {
            for (int i = 0; i < m_numParticles; i++) {
                Particle particle;
                particle.setPosition(glm::vec3(0.0f, 2.0f, 0.0f));
                particle.setVelocity((rand01() - 0.5f), 0.f, 0.5f * rand01());
                m_particleSystem.push_back(particle);
            }
        } break;
        case ParticleSystemType::Fountain: {
            for (int i = 0; i < m_numParticles; i++) {
                Particle particle;
                particle.setPosition(0.f, 2.f, 0.f);
                particle.setVelocity(2.f * (rand01() - 0.5f), 5.f, 2.f * (rand01() - 0.5f));
                m_particleSystem.push_back(particle);
            }
        } break;
    }
}

void ParticleSystem::updateParticleSystem(const float& dt, Particle::UpdateMethod method) {
    for (int i = 0; i < m_numParticles; i++) {
        m_particleSystem[i].setForce(0.0f, -9.8f, 0.0f);
        m_particleSystem[i].updateParticle(dt, method);
    }
}
