#include <Particle.h>

#include <Entity.hpp>
#include <Scene.hpp>
#include <glm/gtx/string_cast.hpp>

#define EPS 0.001f

Particle::Particle() : m_currentPosition(0, 0, 0), m_previousPosition(0, 0, 0), m_bouncing(0.f), m_lifetime(50), m_fixed(false) {
}

Particle::Particle(const float& x, const float& y, const float& z) : m_previousPosition(0, 0, 0), m_bouncing(0.f), m_lifetime(50), m_fixed(false) {
    m_currentPosition.x = x;
    m_currentPosition.y = y;
    m_currentPosition.z = z;
    m_previousPosition = glm::vec3(x, y, z);
}

Particle::~Particle() {
}

//setters
void Particle::setPosition(const float& x, const float& y, const float& z) {
    glm::vec3 pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    m_currentPosition = pos;
    m_previousPosition = pos;
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
    switch (method) {
        case UpdateMethod::EulerOrig: {
            m_previousPosition = m_currentPosition;
            m_currentPosition += m_velocity * dt;
            m_velocity += m_force * dt / m_mass;
        } break;
        case UpdateMethod::EulerSemi: {
            m_previousPosition = m_currentPosition;
            m_velocity += m_force * dt / m_mass;
            m_currentPosition += m_velocity * dt;
        } break;
        case UpdateMethod::Verlet: {
            if (m_firstUpdate) {
                m_previousPreviousPosition = m_currentPosition - dt * dt * m_force / m_mass;
                m_firstUpdate = false;
            } else {
                m_previousPreviousPosition = m_previousPosition;
            }
            m_previousPosition = m_currentPosition;
            m_currentPosition = 1.9999f * m_previousPosition - 0.9999f * m_previousPreviousPosition + dt * dt * m_force / m_mass;
            m_velocity = (m_currentPosition - m_previousPosition) / dt;

        } break;
    }
    return;
}

void Particle::updateInScene(float frameTime, int numSteps, vkr::KinematicEntities& kinematicEntities, UpdateMethod method) {
    
    // call solver types: EulerOrig, EulerSemi and Verlet(to be implemented)
    updateParticle(m_dt, method);
    
    //Check collisions
    checkAndCorrectCollisionParticlePlane(kinematicEntities.kinematicPlaneEntities);

    for (auto triangleEntity : kinematicEntities.kinematicTriangleEntities) {
        // Check both triangle faces (in different planes to compensate particle size)
        std::array<glm::vec3, 3> frontVerts{triangleEntity->triangleColliderVertices[0],
                                            triangleEntity->triangleColliderVertices[1],
                                            triangleEntity->triangleColliderVertices[2]};

        if (collisionParticleTriangle(*triangleEntity->plane, frontVerts)) {
            correctCollisionParticlePlain(*triangleEntity->plane);
            continue;
        }

        std::array<glm::vec3, 3> backVerts{triangleEntity->triangleColliderVertices[3],
                                           triangleEntity->triangleColliderVertices[4],
                                           triangleEntity->triangleColliderVertices[5]};

        if (collisionParticleTriangle(*triangleEntity->backfacePlane, backVerts)) {
            correctCollisionParticlePlain(*triangleEntity->backfacePlane);
        }
    }

    Plane plane;
    bool bugged{false};
    for (auto sphereEntity : kinematicEntities.kinematicSphereEntities) {
        if (collisionParticleSphere(sphereEntity->transform.translation, sphereEntity->transform.scale.x, plane, bugged)) {
            if (bugged) m_currentPosition = m_previousPosition;
            correctCollisionParticlePlain(plane);
        }
        bugged = false;
    }
}

void Particle::addSpringForce(std::shared_ptr<Particle> nextParticle, glm::vec3& gravity, bool isFirstParticle) {
    glm::vec3 particlesDifference = nextParticle->getCurrentPosition() - m_currentPosition;
    float particlesDistance = glm::length(particlesDifference);
    glm::vec3 dir = particlesDifference / particlesDistance;

    // Elastic force (magnitude)
    float fs = m_stiffness * (particlesDistance - m_desiredLength);

    // Damping force (magnitude)
    float fd = m_damping * glm::dot((nextParticle->getVelocity() - m_velocity), dir);

    // Sum forces and set direction
    glm::vec3 f = (fs + fd) * dir;

    // Apply force to the current and next particle
    if (isFirstParticle) {
        setForce(gravity);
    } else {
        addForce(f);  // Current particle will already contain gravity force added by previous one
    }
    nextParticle->setForce(gravity - f);
}

bool Particle::collisionParticlePlane(Plane& p) {
    float sign, sign2;
    sign = glm::dot(m_currentPosition, p.normal) + p.d;
    sign2 = glm::dot(m_previousPosition, p.normal) + p.d;

    return sign < 0 && sign2 > 0;
}

bool Particle::collisionParticleTriangle(Plane& p, std::array<glm::vec3, 3>& vertices) {
    // Check collision with triangle plane
    if (collisionParticlePlane(p) && glm::dot(p.normal, m_previousPosition - p.point) >= 0) {
        // Check collision with actual triangle
        glm::vec3 projPos = m_currentPosition - (m_currentPosition - (vertices[0])) * p.normal * p.normal;
        glm::vec3 x_v1 = vertices[0] - projPos;
        glm::vec3 x_v2 = vertices[1] - projPos;
        glm::vec3 x_v3 = vertices[2] - projPos;
        glm::vec3 v1_v2 = vertices[0] - vertices[1];
        glm::vec3 v1_v3 = vertices[0] - vertices[2];

        return computeTriangleArea(x_v2, x_v3) + computeTriangleArea(x_v3, x_v1) +
                   computeTriangleArea(x_v1, x_v2) <=
               computeTriangleArea(v1_v2, v1_v3) + EPS;
    }

    return false;
}

bool Particle::collisionParticleSphere(glm::vec3 center, float radius, Plane& plane, bool& bugged) {
    // Discard particles outside the sphere
    float extendedRadius = radius + m_size;
    if (glm::length(m_currentPosition - center) > extendedRadius) return false;

    // Compute P as a contact sphere-segment
    glm::vec3 v = m_currentPosition - m_previousPosition;
    float a = glm::dot(v, v);
    float b = glm::dot(2.f * v, m_previousPosition - center);
    float c = glm::dot(center, center) + glm::dot(m_previousPosition, m_previousPosition) -
              glm::dot(2.f * m_previousPosition, center) - extendedRadius * extendedRadius;
    float sol1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);

    // Define tangent plane on P
    if (sol1 < 1 && sol1 > 0) {
        glm::vec3 P = m_previousPosition + v * sol1;
        plane.setPlaneNormal(glm::normalize(P - center));
        plane.setPlanePoint(P);
        return true;
    }

    float sol2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    if (sol2 < 1 && sol2 > 0) {
        glm::vec3 P = m_previousPosition + v * sol2;
        plane.setPlaneNormal(glm::normalize(P - center));
        plane.setPlanePoint(P);
        return true;
    }

    // Fix precision errors
    glm::vec3 P = m_previousPosition;
    plane.setPlaneNormal(glm::normalize(P - center));
    plane.setPlanePoint(P);
    bugged = true;
    return true;
}

void Particle::correctCollisionParticlePlain(Plane& p) {
    m_currentPosition = m_currentPosition - (1 + m_bouncing) * (glm::dot(m_currentPosition, p.normal) + p.d) * p.normal;
    m_velocity = m_velocity - (1 + m_bouncing) * (glm::dot(m_velocity, p.normal)) * p.normal;
    glm::vec3 normal_velocity = glm::dot(p.normal, m_velocity) * p.normal;
    m_velocity -= m_friction * (m_velocity - normal_velocity);

    // to apply in verlet
    m_firstUpdate = true;
}

void Particle::checkAndCorrectCollisionParticlePlane(std::vector<std::shared_ptr<vkr::Entity>>& planes, int depth) {
    if (depth == 1)
        int a = 1;
    for (int i = 0; i < 5; i++) {
        if (collisionParticlePlane(*(planes[i]->plane))) {
            correctCollisionParticlePlain(*(planes[i]->plane));

            for (int j = 0; j < i; j++) {
                if (collisionParticlePlane(*(planes[j]->plane))) {
                    correctCollisionParticlePlain(*(planes[j]->plane));
                }
                // checkAndCorrectCollisionParticlePlane(planes, 1);
            }
        }
    }
}

float Particle::computeTriangleArea(glm::vec3 edge1, glm::vec3 edge2) {
    return glm::length(glm::cross(edge1, edge2)) / 2.f;
}