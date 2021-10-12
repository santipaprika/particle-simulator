#pragma once
#include <Plane.h>

#include <glm\glm.hpp>
#include <memory>
#include <vector>

namespace vkr {
class Entity;
struct KinematicEntities;
}  // namespace vkr

class Particle {
   public:
    enum class UpdateMethod : std::int8_t { EulerOrig,
                                            EulerSemi,
                                            Verlet };

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
    void setSize(float size) { m_size = size; }
    void setFriction(float friction) { m_friction = friction; }

    //getters
    glm::vec3 getCurrentPosition();
    glm::vec3 getPreviousPosition();
    glm::vec3 getForce();
    glm::vec3 getVelocity();
    float getBouncing();
    float getLifetime();
    bool isDead() { return m_currentTime >= m_lifetime; }
    bool isFixed();

    //other
    void addForce(glm::vec3 force);
    void addForce(const float& x, const float& y, const float& z);
    void updateParticle(const float& dt, UpdateMethod method = UpdateMethod::EulerOrig);
    void updateInScene(float frameTime, vkr::KinematicEntities& kinematicEntities, UpdateMethod method);
    bool collisionParticlePlane(Plane& p);
    bool collisionParticleTriangle(Plane& p, std::array<glm::vec3, 3>& vertices);
    bool collisionParticleSphere(glm::vec3 center, float radius, Plane& plane);
    void correctCollisionParticlePlain(Plane& p);

    float computeTriangleArea(glm::vec3 edge1, glm::vec3 edge2);

   private:
    glm::vec3 m_currentPosition;
    glm::vec3 m_previousPosition;
    glm::vec3 m_previousPreviousPosition;
    glm::vec3 m_force;
    glm::vec3 m_velocity;

    float m_bouncing;
    float m_friction{0.8f};
    float m_lifetime;
    float m_currentTime{0.f};
    float m_timeSinceLastUpdate{0.f};
    float m_dt;
    float m_size{0.05f};
    bool m_fixed;

    bool m_firstUpdate{true};
};
