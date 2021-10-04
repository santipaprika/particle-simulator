#pragma once
#include <glm\glm.hpp>
#include "Plane.h"

class Particle
{
public:
	enum class UpdateMethod : std::int8_t { EulerOrig, EulerSemi, Verlet };

	Particle();
	Particle(const float& x, const float& y, const float& z);
//	Particle(glm::vec3 pos, glm::vec3 vel, float bouncing = 1.0f, bool fixed = false, int lifetime = -1, glm::vec3 force = glm::vec3(0, 0, 0));
	~Particle();
	//setters
	void setPosition(const float& x, const float& y, const float& z);
	void setPosition(glm::vec3 pos);
	void setPreviousPosition(const float& x, const float& y, const float& z);
	void setPreviousPosition(glm::vec3 pos);
	void setVelocity(const float& x, const float& y, const float& z);
	void setVelocity(glm::vec3 vel);
	void setForce(const float& x, const float& y, const float& z);
	void setForce(glm::vec3 force);
	void setBouncing(float bouncing);
	void setLifetime(float lifetime);
	void setFixed(bool fixed);
    void setTimeStep(float dt) { m_dt = dt; }


	//getters
	glm::vec3 getCurrentPosition();
	glm::vec3 getPreviousPosition();
	glm::vec3 getForce();
	glm::vec3 getVelocity();
	float getBouncing();
	float getLifetime();
	bool isFixed();

	//other
	void addForce(glm::vec3 force);
	void addForce(const float& x, const float& y, const float& z);
	void updateParticle(const float& dt, UpdateMethod method = UpdateMethod::EulerOrig);
    glm::vec3 updateInScene(float frameTime);
	bool collisionParticlePlane(Plane p);
	void correctCollisionParticlePlain(Plane p);


private:
	glm::vec3 m_currentPosition;
	glm::vec3 m_previousPosition;
	glm::vec3 m_force;
	glm::vec3 m_velocity;

	float m_bouncing;
	float m_lifetime;
    float m_dt;
	bool  m_fixed;

};

