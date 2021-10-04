#include <Hair.hpp>
#include <Utils.hpp>

namespace vkr {

Hair::Hair(Device &device, const char *filename) : device{device} {
    Builder builder;
    builder.loadHairModel(filename, this->hair, dirs);
    createVertexBuffers(builder.vertices);
    createIndexBuffers(builder.indices);
}

Hair::~Hair() {
    delete[] dirs;
    vkDestroyBuffer(device.device(), vertexBuffer, nullptr);
    vkFreeMemory(device.device(), vertexBufferMemory, nullptr);

    if (hasIndexBuffer) {
        vkDestroyBuffer(device.device(), indexBuffer, nullptr);
        vkFreeMemory(device.device(), indexBufferMemory, nullptr);
    }
}

void Hair::Builder::loadHairModel(const char *filename, cyHairFile &hairfile, float *&dirs) {
    // Load the hair model
    int result = hairfile.LoadFromFile(filename);
    // Check for errors
    switch (result) {
        case CY_HAIR_FILE_ERROR_CANT_OPEN_FILE:
            printf("Error: Cannot open hair file!\n");
            return;
        case CY_HAIR_FILE_ERROR_CANT_READ_HEADER:
            printf("Error: Cannot read hair file header!\n");
            return;
        case CY_HAIR_FILE_ERROR_WRONG_SIGNATURE:
            printf("Error: File has wrong signature!\n");
            return;
        case CY_HAIR_FILE_ERROR_READING_SEGMENTS:
            printf("Error: Cannot read hair segments!\n");
            return;
        case CY_HAIR_FILE_ERROR_READING_POINTS:
            printf("Error: Cannot read hair points!\n");
            return;
        case CY_HAIR_FILE_ERROR_READING_COLORS:
            printf("Error: Cannot read hair colors!\n");
            return;
        case CY_HAIR_FILE_ERROR_READING_THICKNESS:
            printf("Error: Cannot read hair thickness!\n");
            return;
        case CY_HAIR_FILE_ERROR_READING_TRANSPARENCY:
            printf("Error: Cannot read hair transparency!\n");
            return;
        default:
            printf("Hair file \"%s\" loaded.\n", filename);
    }
    int hairCount = hairfile.GetHeader().hair_count;
    int vertexCount = hairfile.GetHeader().point_count;
    printf("Number of hair strands = %d\n", hairCount);
    printf("Number of hair points = %d\n", vertexCount);

    // Allocate space for directions
    dirs = (float *)malloc(sizeof(float) * vertexCount * 3);
    // Compute directions
    if (hairfile.FillDirectionArray(dirs) == 0) {
        printf("Error: Cannot compute hair directions!\n");
    }

    // Fill strand array with vertices
    float *pointsArray = hairfile.GetPointsArray();
    unsigned short *segmentsArray = hairfile.GetSegmentsArray();
    unsigned short defaultSegments = hairfile.GetHeader().d_segments;
    float *colorArray = hairfile.GetColorsArray();
    glm::vec3 defaultColor(hairfile.GetHeader().d_color[0], hairfile.GetHeader().d_color[1], hairfile.GetHeader().d_color[2]);

    int pointIdx = 0;
    int p1 = 0, p2 = 0, p3 = 0;
    for (int i = 0; i < hairCount; i++) {
        short numSegments = segmentsArray ? segmentsArray[i] : defaultSegments;
        for (short j = 0; j < numSegments+1; j++) {  // using lines, nPoints = nSegments + 1
            indices.push_back(pointIdx / 3);
            p1 = pointIdx++, p2 = pointIdx++, p3 = pointIdx++;
            glm::vec3 point(pointsArray[p1], pointsArray[p2], pointsArray[p3]);
            glm::vec3 direction(dirs[p1], dirs[p2], dirs[p3]);
            glm::vec3 color = colorArray ? glm::vec3(colorArray[p1], colorArray[p2], colorArray[p3]) : defaultColor;

            vertices.push_back(Vertex{point, color, direction});
        }
        // Set primitive restart
        indices.push_back(0xFFFFFFFF);
    }

    printf("Number of stored hair points = %zd\n", vertices.size());
    printf("Number of indices = %zd\n", indices.size());
}

void Hair::createVertexBuffers(const std::vector<Vertex> &vertices) {
    vertexCount = static_cast<uint32_t>(vertices.size());
    VkDeviceSize bufferSize = sizeof(Vertex) * static_cast<uint32_t>(vertexCount);

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    device.createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        stagingBuffer, stagingBufferMemory);

    void *data;
    vkMapMemory(device.device(), stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), static_cast<size_t>(bufferSize));
    vkUnmapMemory(device.device(), stagingBufferMemory);

    device.createBuffer(
        bufferSize,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

    device.copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

    vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
    vkFreeMemory(device.device(), stagingBufferMemory, nullptr);
}

void Hair::createIndexBuffers(const std::vector<uint32_t> &indices) {
    indexCount = static_cast<uint32_t>(indices.size());
    hasIndexBuffer = indexCount > 0;

    if (!hasIndexBuffer) {
        return;
    }

    VkDeviceSize bufferSize = sizeof(indices[0]) * indexCount;

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    device.createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        stagingBuffer, stagingBufferMemory);

    void *data;
    vkMapMemory(device.device(), stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), static_cast<size_t>(bufferSize));
    vkUnmapMemory(device.device(), stagingBufferMemory);

    device.createBuffer(
        bufferSize,
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

    device.copyBuffer(stagingBuffer, indexBuffer, bufferSize);

    vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
    vkFreeMemory(device.device(), stagingBufferMemory, nullptr);
}

void Hair::draw(VkCommandBuffer commandBuffer) {
    if (hasIndexBuffer) {
        vkCmdDrawIndexed(commandBuffer, indexCount, 1, 0, 0, 0);
    } else {
        vkCmdDraw(commandBuffer, vertexCount, 1, 0, 0);
    }
}

void Hair::bind(VkCommandBuffer commandBuffer) {
    VkBuffer buffers[] = {vertexBuffer};
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, buffers, offsets);

    if (hasIndexBuffer) {
        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
    }
}

std::vector<VkVertexInputBindingDescription> Hair::Vertex::getBindingDescriptions() {
    std::vector<VkVertexInputBindingDescription> bindingDescriptions(1);
    bindingDescriptions[0].binding = 0;
    bindingDescriptions[0].stride = sizeof(Vertex);
    bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription> Hair::Vertex::getAttributeDescriptions() {
    std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};

    attributeDescriptions.push_back(
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)});
    attributeDescriptions.push_back(
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
    attributeDescriptions.push_back(
        {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, direction)});

    return attributeDescriptions;
}

}  // namespace vkr
