#pragma once

#include <cyHairFile.h>
#include <vulkan/vulkan.h>

#include <Device.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <Particle.h>

class Particle;

namespace vkr {

struct KinematicEntities;
struct TransformComponent;
class Entity;

class Hair {
   public:
    struct Vertex {
        glm::vec3 position{};
        glm::vec3 color{};
        glm::vec3 direction{};

        static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
        static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();

        bool operator==(const Vertex &other) const {
            return position == other.position && color == other.color && direction == other.direction;
        }
    };

    struct Builder {
        std::vector<Vertex> vertices{};
        std::vector<std::shared_ptr<Particle>> verticesParticles{};
        std::vector<uint32_t> indices{};
        int defaultSegments{0};

        void loadHairModel(const char *filename, cyHairFile &hairfile, float *&dirs);
    };

    Hair(Device &device, const char *filename);
    ~Hair();

    void loadParticles(TransformComponent& transform);

    void draw(VkCommandBuffer commandBuffer);
    void bind(VkCommandBuffer commandBuffer);

    void renderUI();
    Builder builder;
    int numStrands{5};

    void update(float dt, KinematicEntities &kinematicEntities, TransformComponent& transform);
    void updateBuffers();

   private:
    void createVertexBuffers(const std::vector<Vertex> &vertices, int numSegments);
    void createIndexBuffers(const std::vector<uint32_t> &indices, int numSegments);

    cyHairFile hair;
    float *dirs;

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

    float segmentLength;
    float strandMass{0.5f};

    float stiffness{1000.f};
    float damping{4.f};
    float bouncing{0.7f};
    float friction{0.01f};
    
    bool fixed{true};
    Particle::UpdateMethod solver{Particle::UpdateMethod::EulerSemi};

    glm::vec3 gravity{0.f, -0.8f, 0.f};
};

}  // namespace vkr
