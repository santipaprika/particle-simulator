#pragma once

#include <Particle.h>
#include <vulkan/vulkan.h>

#include <Device.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

class Particle;

namespace vkr {

struct KinematicEntities;
struct TransformComponent;
class Entity;

class Cloth {
   public:
    struct Vertex {
        glm::vec3 position{};
        glm::vec3 color{};
        glm::vec3 normal{};
        glm::vec2 uv{};

        static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
        static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();

        bool operator==(const Vertex &other) const {
            return position == other.position && color == other.color && normal == other.normal && uv == other.uv;
        }
    };

    struct Builder {
        std::vector<Vertex> vertices{};
        std::vector<std::shared_ptr<Particle>> verticesParticles{};
        std::vector<uint32_t> indices{};

        // Vectors containing the internal forces neighboring particle indices for each particle.
        std::vector<std::vector<int>> streachParticlesIdx;
        std::vector<std::vector<std::array<int, 2>>> shearParticlesIdx;
        std::vector<std::vector<std::array<int, 2>>> bendParticlesIdx;

        int gridSize;
        float cellSize;

        void createClothModel(int gridSize, float cellSize);
        void defineTrianglesInCell(int i, int j);
        void reset();
    };

    Cloth(Device &device, int gridSize, float cellSize);
    ~Cloth();

    void loadParticles(TransformComponent &transform);

    void draw(VkCommandBuffer commandBuffer);
    void bind(VkCommandBuffer commandBuffer);

    void renderUI();
    Builder builder;

    void update(float dt, KinematicEntities &kinematicEntities, TransformComponent &transform);
    void updateBuffers();

   private:
    void createVertexBuffers(const std::vector<Vertex> &vertices);
    void createIndexBuffers(const std::vector<uint32_t> &indices);
    void applyGravityForce();

    Device &device;

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    void *data;

    VkBuffer vertexBuffer{VK_NULL_HANDLE};
    VkDeviceMemory vertexBufferMemory;
    uint32_t vertexCount;

    bool hasIndexBuffer = false;
    VkBuffer indexBuffer{VK_NULL_HANDLE};
    VkDeviceMemory indexBufferMemory;
    uint32_t indexCount;

    float clothMass{2.f};

    float stiffness{1000.f};
    float damping{4.f};
    float bouncing{0.5f};
    float friction{0.5f};
    float airFriction{0.993f};

    bool fixed{true};
    Particle::UpdateMethod solver{Particle::UpdateMethod::EulerSemi};

    glm::vec3 gravity{0.f, -0.8f, 0.f};
};

}  // namespace vkr
