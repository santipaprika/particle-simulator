#pragma once

#include <Device.hpp>

// img
#include <stb_image.h>

// libs
#include <glm/glm.hpp>

// std
#include <array>
#include <memory>
#include <vector>

namespace vkr {
class Texture {
   public:
    struct Builder {
        stbi_uc *pixels;
        std::array<stbi_uc *, 6> cubemapPixels;
        int texWidth, texHeight, texChannels;

        void loadImage(const std::string &filepath);
        void loadCubemap(const std::string &filepath);
    };

    Texture(Device &device, const Builder &builder);
    ~Texture();

    void destroy();

    Texture(const Texture &) = delete;
    Texture &operator=(const Texture &) = delete;

    void createImage(const Builder &builder, VkFormat format, VkSampleCountFlagBits numSamples,
                     VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties,
                     VkImage &image, VkDeviceMemory &imageMemory, bool isCubemap = false);
    static std::unique_ptr<Texture> createTextureFromFile(Device &device, const std::string &filepath);
    static std::unique_ptr<Texture> createCubemapFromFile(Device &device, const std::string &filepath);
    VkImage getTextureImage() { return textureImage; }
    const VkDescriptorImageInfo &getDescriptorInfo() { return descriptorInfo; }

    void createTextureImageView(bool isCubemap = false);
    void createTextureSampler();

   private:
    Device &device;

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    VkImage textureImage;
    VkDeviceMemory textureImageMemory;

    VkDescriptorImageInfo descriptorInfo;
};
}  // namespace vkr