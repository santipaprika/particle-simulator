#include "ParticleSystem.h"
#include <random>
#include <iostream>

#define rand01() ((float)std::rand()/RAND_MAX)


ParticleSystem::ParticleSystem()
{
}


ParticleSystem::~ParticleSystem()
{
}

void ParticleSystem::setParticleSystem(int numParticles){
	m_particleSystem.resize(numParticles);
	m_numParticles = numParticles;
}


Particle ParticleSystem::getParticle(int i){
	return m_particleSystem[i];
}

void ParticleSystem::iniParticleSystem(ParticleSystemType systemType){

	switch (systemType)
	{
		case ParticleSystemType::Waterfall:
		{							
 		for (int i = 0; i < m_numParticles; i++)
		   {
			   m_particleSystem[i].setPosition(glm::vec3(0.0f, 10.0f, 0.0f));
			   m_particleSystem[i].setVelocity((rand01() - 0.5), 0, 0.5*rand01());
		   }
		}
		break;
		case ParticleSystemType::Fountain:
		{
           for (int i = 0; i < m_numParticles; i++)
		   {
			   m_particleSystem[i].setPosition(0, 0, 0);
			   m_particleSystem[i].setVelocity(2 * (rand01() - 0.5), 5, 2 * rand01());
		   }
		}
		break;
	}

}




void ParticleSystem::updateParticleSystem(const float& dt, Particle::UpdateMethod method){

	for (int i = 0; i < m_numParticles; i++)
	{
		m_particleSystem[i].setForce(0.0f, -9.8f, 0.0f);
		m_particleSystem[i].updateParticle(dt, method);
	}
}

