#include <Particle.h>

#include <glm/gtx/string_cast.hpp>
#include <iostream>

//Particle::Particle()
//{
//}

Particle::Particle() : m_currentPosition(0, 0, 0), m_previousPosition(0, 0, 0), m_velocity(0, 0, 0), m_force(0, 0, 0), m_bouncing(1), m_lifetime(50), m_fixed(false), m_dt(0.0001f) {
}

Particle::Particle(const float& x, const float& y, const float& z) : m_previousPosition(0, 0, 0), m_velocity(0, 0, 0), m_force(0, 0, 0), m_bouncing(1), m_lifetime(50), m_fixed(false), m_dt(0.0001f) {
    m_currentPosition.x = x;
    m_currentPosition.y = y;
    m_currentPosition.z = z;
}

/*
Particle::Particle(glm::vec3 pos, glm::vec3 vel, float bouncing, bool fixed, int lifetime, glm::vec3 force) :
m_currentPosition(pos), m_previousPosition(pos), m_force(force), m_velocity(vel), m_bouncing(bouncing), m_lifetime(lifetime), m_fixed(fixed)
{
}
*/

Particle::~Particle() {
}

//setters
void Particle::setPosition(const float& x, const float& y, const float& z) {
    glm::vec3 pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    m_currentPosition = pos;
}
void Particle::setPosition(glm::vec3 pos) {
    m_currentPosition = pos;
}

void Particle::setPreviousPosition(const float& x, const float& y, const float& z) {
    glm::vec3 pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    m_previousPosition = pos;
}

void Particle::setPreviousPosition(glm::vec3 pos) {
    m_previousPosition = pos;
}

void Particle::setForce(const float& x, const float& y, const float& z) {
    glm::vec3 force;
    force.x = x;
    force.y = y;
    force.z = z;
    m_force = force;
}

void Particle::setForce(glm::vec3 force) {
    m_force = force;
}

void Particle::addForce(const float& x, const float& y, const float& z) {
    glm::vec3 force{0.f};
    force.x += x;
    force.y += y;
    force.z += z;
    m_force = force;
}

void Particle::addForce(glm::vec3 force) {
    m_force += force;
}

void Particle::setVelocity(const float& x, const float& y, const float& z) {
    glm::vec3 vel;
    vel.x = x;
    vel.y = y;
    vel.z = z;
    m_velocity = vel;
}

void Particle::setVelocity(glm::vec3 vel) {
    m_velocity = vel;
}

void Particle::setBouncing(float bouncing) {
    m_bouncing = bouncing;
}

void Particle::setLifetime(float lifetime) {
    m_lifetime = lifetime;
}

void Particle::setFixed(bool fixed) {
    m_fixed = fixed;
}

//getters
glm::vec3 Particle::getCurrentPosition() {
    return m_currentPosition;
}

glm::vec3 Particle::getPreviousPosition() {
    return m_previousPosition;
}

glm::vec3 Particle::getForce() {
    return m_force;
}

glm::vec3 Particle::getVelocity() {
    return m_velocity;
}

float Particle::getBouncing() {
    return m_bouncing;
}

float Particle::getLifetime() {
    return m_lifetime;
}

bool Particle::isFixed() {
    return m_fixed;
}

void Particle::updateParticle(const float& dt, UpdateMethod method) {
    if (!m_fixed) {
        switch (method) {
            case UpdateMethod::EulerOrig: {
                m_previousPosition = m_currentPosition;
                m_currentPosition += m_velocity * dt;
                m_velocity += m_force * dt;
            } break;
            case UpdateMethod::EulerSemi: {
                m_previousPosition = m_currentPosition;
                m_velocity += m_force * dt;
                m_currentPosition += m_velocity * dt;
            } break;
            case UpdateMethod::Verlet: {
                // to be implemented
            } break;
        }
    }
    return;
}

glm::vec3 Particle::updateInScene(float frameTime) {
    int numSteps = static_cast<int>(std::round(frameTime / m_dt));

    // animation loop
    for (int step = 1; step < numSteps; step++) {
        // call solver types: EulerOrig, EulerSemi and Verlet(to be implemented)
        updateParticle(m_dt, Particle::UpdateMethod::EulerSemi);

        Plane plane;
        plane.setPlaneNormal(0, 1, 0);
        plane.setPlanePoint(0, 0, 0);
        //Check Floor collisions
        if (collisionParticlePlane(plane)) {
            correctCollisionParticlePlain(plane);
        }
    }

    return m_currentPosition;
}

bool Particle::collisionParticlePlane(Plane p) {
    float sign;
    sign = glm::dot(m_currentPosition, p.normal) + p.d;
    sign *= glm::dot(m_previousPosition, p.normal) + p.d;

    if (sign <= 0) {
        return true;
    }
    return false;
}

void Particle::correctCollisionParticlePlain(Plane p) {
    m_currentPosition = m_currentPosition - (1 + m_bouncing) * (glm::dot(m_currentPosition, p.normal) + p.d) * p.normal;
    m_velocity = m_velocity - (1 + m_bouncing) * (glm::dot(m_velocity, p.normal) + p.d) * p.normal;
}
