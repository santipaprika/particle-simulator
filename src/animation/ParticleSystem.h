#pragma once
#include <Particle.h>
#include <vector>
#include <memory>
#include <Plane.h>


namespace vkr {
    class Mesh;
    class Material;
    class Entity;
    class Scene;
}

class ParticleSystem
{
public:
	enum class ParticleSystemType : std::int8_t { Fountain, Waterfall };
	ParticleSystem();
	~ParticleSystem();
	void iniParticleSystem();
	void updateParticleSystem(const float& dt, Particle::UpdateMethod method);
    void updateInScene(const float& dt, vkr::KinematicEntities& kinematicEntities);
    void renderUI();
    
    // Getters
	std::shared_ptr<Particle> getParticle(int i);
    size_t getNumParticles() { return m_particleSystem.size(); }
    int getNumParticlesPerSpawn() { return m_numParticles; }
    float getSpawnTime() { return m_spawnTime; }
	
    // Setters
    void setParticleSystem(int numParticles);
    void setType(ParticleSystemType type) { this->type = type; }

    void spawnParticles(std::shared_ptr<vkr::Mesh>& mesh, std::shared_ptr<vkr::Material> material, std::vector<std::shared_ptr<vkr::Entity>>& sceneEntities);

private:
	int m_numParticles;
    float m_spawnTime;
    glm::vec3 m_force;
    float m_bouncing{0.8f};
    float m_friction{0.15f};
    Particle::UpdateMethod m_solver{Particle::UpdateMethod::Verlet};
	std::vector<std::shared_ptr<Particle>> m_particleSystem;
    ParticleSystemType type { ParticleSystemType::Fountain };
};

