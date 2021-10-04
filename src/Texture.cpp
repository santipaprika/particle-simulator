#include <SwapChain.hpp>
#include <Texture.hpp>
#include <iostream>
#include <stdexcept>

namespace vkr {

void Texture::Builder::loadImage(const std::string& filepath) {
    pixels = stbi_load(filepath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

    if (!pixels) {
        throw std::runtime_error("failed to load texture image!");
    }
}

void Texture::Builder::loadCubemap(const std::string& filepath) {
    std::array<std::string, 6> facePathStrings{{"posx.jpg", "negx.jpg", "posy.jpg", "negy.jpg", "posz.jpg", "negz.jpg"}};
    for (int i = 0; i < 6; i++) {
        cubemapPixels[i] = stbi_load((filepath + facePathStrings[i]).c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

        if (!cubemapPixels[i]) {
            throw std::runtime_error("failed to load texture image!");
        }
    }
}

Texture::Texture(Device& device, const Texture::Builder& builder) : device{device} {
    uint32_t numPixels = builder.texWidth * builder.texHeight * 4;
    VkDeviceSize bufferSize = builder.pixels ? numPixels : numPixels * 6;
    device.createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        stagingBuffer, stagingBufferMemory);

    void* data;
    vkMapMemory(device.device(), stagingBufferMemory, 0, bufferSize, 0, &data);
    if (builder.pixels)
        memcpy(data, builder.pixels, static_cast<size_t>(bufferSize));
    else {
        stbi_uc* imagesData = (stbi_uc*)malloc(bufferSize);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < numPixels; j++) {
                imagesData += numPixels * i + j;
                *imagesData = builder.cubemapPixels[i][j];
                imagesData -= numPixels * i + j;
            }
        }
        memcpy(data, imagesData, static_cast<size_t>(bufferSize));
        free(imagesData);
    }
    vkUnmapMemory(device.device(), stagingBufferMemory);

    bool isCubemap = false;
    if (builder.pixels)
        stbi_image_free(builder.pixels);
    else {
        for (int i = 0; i < 6; i++)
            stbi_image_free(builder.cubemapPixels[i]);

        isCubemap = true;
    }

    createImage(builder, VK_FORMAT_R8G8B8A8_SRGB, VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_TILING_OPTIMAL,
                VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                textureImage, textureImageMemory, isCubemap);

    // could be improved executing asynchronously
    device.transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, isCubemap);
    descriptorInfo.imageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    if (!isCubemap)
        device.copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(builder.texWidth), static_cast<uint32_t>(builder.texHeight));
    else
        device.copyBufferToCubemap(stagingBuffer, textureImage, static_cast<uint32_t>(builder.texWidth), static_cast<uint32_t>(builder.texHeight), numPixels);

    device.transitionImageLayout(textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, isCubemap);
    descriptorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
    vkFreeMemory(device.device(), stagingBufferMemory, nullptr);

    createTextureImageView(isCubemap);
    createTextureSampler();
}

Texture::~Texture() {
    destroy();
}

void Texture::destroy() {
    if (descriptorInfo.imageView) {
        vkDestroyImageView(device.device(), descriptorInfo.imageView, nullptr);
        descriptorInfo.imageView = nullptr;
    }
    if (textureImage) {
        vkDestroyImage(device.device(), textureImage, nullptr);
        textureImage = nullptr;
    }
    if (descriptorInfo.sampler) {
        vkDestroySampler(device.device(), descriptorInfo.sampler, nullptr);
        descriptorInfo.sampler = nullptr;
    }
    if (textureImageMemory) {
        vkFreeMemory(device.device(), textureImageMemory, nullptr);
        textureImageMemory = nullptr;
    }
}

void Texture::createImage(const Builder& builder, VkFormat format, VkSampleCountFlagBits numSamples,
                          VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image,
                          VkDeviceMemory& imageMemory, bool isCubemap) {
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = builder.texWidth;
    imageInfo.extent.height = builder.texHeight;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = isCubemap ? 6 : 1;
    imageInfo.format = format;
    imageInfo.samples = numSamples;
    imageInfo.tiling = tiling;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = usage;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.flags = isCubemap ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0;

    if (vkCreateImage(device.device(), &imageInfo, nullptr, &image) != VK_SUCCESS) {
        throw std::runtime_error("failed to create image!");
    }

    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(device.device(), image, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = device.findMemoryType(memRequirements.memoryTypeBits, properties);

    if (vkAllocateMemory(device.device(), &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate image memory!");
    }

    vkBindImageMemory(device.device(), image, imageMemory, 0);
}

std::unique_ptr<Texture> Texture::createTextureFromFile(Device& device, const std::string& filepath) {
    Builder builder{};
    builder.loadImage(filepath);

    return std::make_unique<Texture>(device, builder);
}

std::unique_ptr<Texture> Texture::createCubemapFromFile(Device& device, const std::string& filepath) {
    Builder builder{};
    builder.loadCubemap(filepath);

    return std::make_unique<Texture>(device, builder);
}

void Texture::createTextureImageView(bool isCubemap) {
    descriptorInfo.imageView = SwapChain::createImageView(device, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT, isCubemap);
}

void Texture::createTextureSampler() {
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

    // Need device properties to know anisotropy limits
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device.physicalDevice(), &properties);
    samplerInfo.anisotropyEnable = VK_TRUE;
    samplerInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;

    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = 0.0f;

    if (vkCreateSampler(device.device(), &samplerInfo, nullptr, &descriptorInfo.sampler) != VK_SUCCESS) {
        throw std::runtime_error("failed to create texture sampler!");
    }
}

}  // namespace vkr