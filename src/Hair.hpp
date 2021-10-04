#pragma once

#include <cyHairFile.h>
#include <vulkan/vulkan.h>

#include <Device.hpp>
#include <glm/glm.hpp>
#include <vector>

namespace vkr {

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
        std::vector<uint32_t> indices{};

        void loadHairModel(const char *filename, cyHairFile &hairfile, float *&dirs);
    };

    Hair(Device &device, const char *filename);
    ~Hair();

    void draw(VkCommandBuffer commandBuffer);
    void bind(VkCommandBuffer commandBuffer);

   private:
    void createVertexBuffers(const std::vector<Vertex> &vertices);
    void createIndexBuffers(const std::vector<uint32_t> &indices);

   private:
    cyHairFile hair;
    float *dirs;

    // std::vector<std::vector<Vertex>> strandsVertices;
    // std::vector<Vertex> strandsVertices;
    Device &device;

    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    uint32_t vertexCount;

    bool hasIndexBuffer = false;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;
    uint32_t indexCount;
};

}  // namespace vkr
