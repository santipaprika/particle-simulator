#pragma once
#include <Particle.h>
#include <vector>
#include <memory>
#include <Plane.h>

class ParticleSystem
{
public:
	enum class ParticleSystemType : std::int8_t { Fountain, Waterfall };
	ParticleSystem();
	~ParticleSystem();
	void setParticleSystem(int numParticles);
	Particle getParticle(int i);
	void iniParticleSystem();
	void updateParticleSystem(const float& dt, Particle::UpdateMethod method);
    size_t getNumParticles() { return m_particleSystem.size(); }
    int getNumParticlesPerSpawn() { return m_numParticles; }

private:
	int m_numParticles;
    float m_spawnFrequency;
	std::vector<Particle> m_particleSystem;
    ParticleSystemType type { ParticleSystemType::Fountain };
};

