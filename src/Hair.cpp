#include <Particle.h>
#include <imgui.h>

#include <Entity.hpp>
#include <Hair.hpp>
#include <Scene.hpp>
#include <Utils.hpp>

namespace vkr {

Hair::Hair(Device &device, const char *filename) : device{device} {
    builder.loadHairModel(filename, this->hair, dirs);
    createVertexBuffers(builder.vertices, builder.defaultSegments);
    createIndexBuffers(builder.indices, builder.defaultSegments);
}

Hair::~Hair() {
    delete[] dirs;

    vkUnmapMemory(device.device(), stagingBufferMemory);

    vkDestroyBuffer(device.device(), vertexBuffer, nullptr);
    vkFreeMemory(device.device(), vertexBufferMemory, nullptr);

    if (hasIndexBuffer) {
        vkDestroyBuffer(device.device(), indexBuffer, nullptr);
        vkFreeMemory(device.device(), indexBufferMemory, nullptr);
    }

    vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
    vkFreeMemory(device.device(), stagingBufferMemory, nullptr);
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
    defaultSegments = hairfile.GetHeader().d_segments;
    float *colorArray = hairfile.GetColorsArray();
    glm::vec3 defaultColor(hairfile.GetHeader().d_color[0], hairfile.GetHeader().d_color[1], hairfile.GetHeader().d_color[2]);

    int pointIdx = 0;
    int p1 = 0, p2 = 0, p3 = 0;
    float reductionFactor = 5.f;
    for (int i = 0; i < hairCount; i++) {
        short numSegments = segmentsArray ? segmentsArray[i] : defaultSegments;
        for (short j = 0; j < numSegments + 1; j++) {  // using lines, nPoints = nSegments + 1
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

void Hair::loadParticles(TransformComponent &transform) {
    for (auto &vertex : builder.vertices) {
        // Create particle attached to vertex
        glm::vec3 worldPos = transform.mat4() * glm::vec4(vertex.position, 1.f);

        std::shared_ptr<Particle> particle = std::make_shared<Particle>(worldPos.x, worldPos.y, worldPos.z);
        particle->setBouncing(bouncing);
        particle->setFriction(friction);
        particle->setForce(gravity);
        particle->setSize(0.02f);
        particle->setLifetime(1000);
        particle->setMass(strandMass / (builder.defaultSegments + 1));
        particle->setStiffness(stiffness);
        particle->setDamping(damping);
        particle->setAirFriction(airFriction);
        
        builder.verticesParticles.push_back(std::move(particle));

    }
    for (int i = 0; i < builder.vertices.size() - 1; i++) {
        builder.verticesParticles[i]->setDesiredLength(glm::length(builder.verticesParticles[i + 1]->getCurrentPosition() - builder.verticesParticles[i]->getCurrentPosition()));
    }
}

void Hair::createVertexBuffers(const std::vector<Vertex> &vertices, int numSegments) {
    // Destroy existing buffer
    if (vertexBuffer != VK_NULL_HANDLE) {
        vkUnmapMemory(device.device(), stagingBufferMemory);
        vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
        vkFreeMemory(device.device(), stagingBufferMemory, nullptr);

        vkDestroyBuffer(device.device(), vertexBuffer, nullptr);
        vkFreeMemory(device.device(), vertexBufferMemory, nullptr);
    }

    VkDeviceSize bufferSize = sizeof(Vertex) * static_cast<uint32_t>(numStrands * (numSegments + 1));

    device.createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        stagingBuffer, stagingBufferMemory);

    vkMapMemory(device.device(), stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), static_cast<size_t>(bufferSize));

    device.createBuffer(
        bufferSize,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

    device.copyBuffer(stagingBuffer, vertexBuffer, bufferSize);
}

void Hair::createIndexBuffers(const std::vector<uint32_t> &indices, int numSegments) {
    // Destroy existing buffer
    if (indexBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device.device(), indexBuffer, nullptr);
        vkFreeMemory(device.device(), indexBufferMemory, nullptr);
    }

    indexCount = static_cast<uint32_t>(numStrands * (numSegments + 2));
    hasIndexBuffer = indexCount > 0;

    if (!hasIndexBuffer) {
        return;
    }

    VkDeviceSize bufferSize = sizeof(indices[0]) * indexCount;

    VkBuffer l_stagingBuffer;
    VkDeviceMemory l_stagingBufferMemory;
    device.createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        l_stagingBuffer, l_stagingBufferMemory);

    void *data;
    vkMapMemory(device.device(), l_stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), static_cast<size_t>(bufferSize));
    vkUnmapMemory(device.device(), l_stagingBufferMemory);

    device.createBuffer(
        bufferSize,
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

    device.copyBuffer(l_stagingBuffer, indexBuffer, bufferSize);

    vkDestroyBuffer(device.device(), l_stagingBuffer, nullptr);
    vkFreeMemory(device.device(), l_stagingBufferMemory, nullptr);
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

void Hair::update(float dt, KinematicEntities &kinematicEntities, TransformComponent &transform) {
    static float timeSinceLastUpdate{0.f};
    timeSinceLastUpdate += dt;
    int numSteps = static_cast<int>(std::floor(timeSinceLastUpdate / builder.verticesParticles[0]->getTimeStep()));
    timeSinceLastUpdate = timeSinceLastUpdate - numSteps * builder.verticesParticles[0]->getTimeStep();

    glm::mat4 modelInv = glm::inverse(transform.mat4());

    for (int step = 0; step < numSteps; step++) {
        // Spring forces update pass
        for (int s = 0; s < numStrands; s++) {
            int offset = s * (builder.defaultSegments + 1);
            for (int i = 0; i < builder.defaultSegments /*+ 1 - 1*/; i++) {
                builder.verticesParticles[offset + i]->addSpringForce(builder.verticesParticles[offset + i + 1], gravity, i == 0);
            }
        }

        // Regular particle update and collision pass
        for (int i = 0; i < numStrands * (builder.defaultSegments + 1); i++) {
            if (!fixed || i % (builder.defaultSegments+1) != 0)
                builder.verticesParticles[i]->updateInScene(dt, numSteps, kinematicEntities, solver);
        }
    }

    for (int i = 0; i < numStrands * (builder.defaultSegments + 1); i++) {
        builder.vertices[i].position = modelInv * glm::vec4(builder.verticesParticles[i]->getCurrentPosition(), 1.f);
    }
    updateBuffers();
}

void Hair::updateBuffers() {
    VkDeviceSize bufferSize = sizeof(Vertex) * static_cast<uint32_t>(numStrands * (builder.defaultSegments + 1));

    memcpy(data, builder.vertices.data(), static_cast<size_t>(bufferSize));
    device.copyBuffer(stagingBuffer, vertexBuffer, bufferSize);
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

void Hair::renderUI() {
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Solver");
    const char *items[] = {"Euler Explicit", "Euler Semi-implicit", "Verlet"};
    ImGui::Combo("##solver", (int *)&solver, items, IM_ARRAYSIZE(items));

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Number of strands:");
    ImGui::SliderInt("##nStrands", &numStrands, 1, 500);
    if (ImGui::IsItemEdited()) {
        vkDeviceWaitIdle(device.device());
        createVertexBuffers(builder.vertices, builder.defaultSegments);
        createIndexBuffers(builder.indices, builder.defaultSegments);
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Checkbox("Fixed", &fixed);

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Strand mass");
    ImGui::SliderFloat("##hairMass", &strandMass, 0.001f, 50.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setMass(strandMass / builder.defaultSegments);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Stiffness");
    ImGui::SliderFloat("##hairStiff", &stiffness, 0.001f, 3000.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setStiffness(stiffness);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Damping");
    ImGui::SliderFloat("##hairDamp", &damping, 0.001f, 10.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setDamping(damping);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Bouncing");
    ImGui::SliderFloat("##hairBounce", &bouncing, 0.001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setBouncing(bouncing);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Friction");
    ImGui::SliderFloat("##hairFriction", &friction, 0.001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setDamping(friction);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Air Friction");
    ImGui::SliderFloat("##hairAirFriction", &airFriction, 0.99f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setAirFriction(airFriction);
        }
    }

    ImGui::Text("Force");
    ImGui::SliderFloat3("##force", (float *)&gravity, -2.f, 2.f);
}

}  // namespace vkr
