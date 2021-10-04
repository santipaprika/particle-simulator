#pragma once
#include "Particle.h"
#include <vector>
#include "Plane.h"

class ParticleSystem
{
public:
	enum class ParticleSystemType : std::int8_t { Fountain, Waterfall };
	ParticleSystem();
	~ParticleSystem();
	void setParticleSystem(int numParticles);
	Particle getParticle(int i);
	void iniParticleSystem(ParticleSystemType systemType);
	void updateParticleSystem(const float& dt, Particle::UpdateMethod method);

private:
	int m_numParticles;
	std::vector<Particle> m_particleSystem;
};

