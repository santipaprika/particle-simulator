/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Buffer.hpp>
#include <Device.hpp>

// libs
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// std
#include <memory>
#include <vector>

namespace vkr {
class Mesh {
   public:
    struct Vertex {
        glm::vec3 position{};
        glm::vec3 color{};
        glm::vec3 normal{};
        glm::vec2 uv{};

        static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
        static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();

        bool operator==(const Vertex &other) const {
            return position == other.position && color == other.color && normal == other.normal &&
                   uv == other.uv;
        }
    };

    struct Builder {
        std::vector<Vertex> vertices{};
        std::vector<uint32_t> indices{};

        void loadModel(const std::string &filepath);
    };

    Mesh(Device &device, const Mesh::Builder &builder);
    ~Mesh();

    Mesh(const Mesh &) = delete;
    Mesh &operator=(const Mesh &) = delete;

    // Generators
    static std::unique_ptr<Mesh> createModelFromFile(
        Device &device, const std::string &filepath);
    static std::unique_ptr<Mesh> createTriangle(Device &device);

        void bind(VkCommandBuffer commandBuffer);
    void draw(VkCommandBuffer commandBuffer);

    // Getters
    const Builder& getBuilder() { return builder; }

   private:
    void createVertexBuffers(const std::vector<Vertex> &vertices);
    void createIndexBuffers(const std::vector<uint32_t> &indices);

    Device &device;

    std::unique_ptr<Buffer> vertexBuffer;
    uint32_t vertexCount;

    bool hasIndexBuffer = false;
    std::unique_ptr<Buffer> indexBuffer;
    uint32_t indexCount;

    Builder builder;
};
}  // namespace vkr
