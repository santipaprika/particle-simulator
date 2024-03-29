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

struct ClothLengths {
    float streachLength;
    float shearLength;
    float bendLength;
};

struct InternalClothParameters {
    float streach{1.f};
    float shear{1.f};
    float bend{1.f};
};

class Cloth {
   public:
    enum { ALL_FIXED,
           FIXED_BY_SIDE,
           FIXED_BY_CORNER,
           NOT_FIXED };
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
    uint32_t particleCount;

    bool hasIndexBuffer = false;
    VkBuffer indexBuffer{VK_NULL_HANDLE};
    VkDeviceMemory indexBufferMemory;
    uint32_t indexCount;

    float clothMass{2.5f};

    float stiffness{1000.f};
    float damping{0.8f};
    float bouncing{0.5f};
    float friction{0.5f};
    float airFriction{0.996f};

    InternalClothParameters internalClothParameters;

    int fixed{ALL_FIXED};
    Particle::UpdateMethod solver{Particle::UpdateMethod::EulerSemi};

    ClothLengths clothLengths;

    glm::vec3 gravity{0.f, -1.5f, 0.f};
};

}  // namespace vkr
