#include <Particle.h>
#include <imgui.h>

#include <Cloth.hpp>
#include <Entity.hpp>
#include <Scene.hpp>
#include <Utils.hpp>

namespace vkr {

Cloth::Cloth(Device &device, int gridSize, float cellSize) : device{device} {
    builder.createClothModel(gridSize, cellSize);
    createVertexBuffers(builder.vertices);
    createIndexBuffers(builder.indices);
}

Cloth::~Cloth() {
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

// Grows from top left to bottom right
void Cloth::Builder::createClothModel(int gridSize, float cellSize) {
    this->gridSize = gridSize;
    this->cellSize = cellSize;

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            int idx = i * gridSize + j;
            vertices.push_back(Vertex{{i * cellSize, 0, j * cellSize}, glm::vec3(1.f), glm::vec3(0.f, 1.f, 0.f), glm::vec2(i, j) / (float)gridSize});

            // 2 triangles are defined for the cell at bottom-right of each vertex.
            // No need to define them for bottom row and right column (would be outside the grid)
            if (i != gridSize - 1 && j != gridSize - 1)
                defineTrianglesInCell(i, j);

            // define neighbor particles contributing to cloth internal forces in current particle

            // Streaching
            streachParticlesIdx.push_back({});
            if (i > 0) streachParticlesIdx[idx].push_back(idx - gridSize);
            if (i < gridSize - 1) streachParticlesIdx[idx].push_back(idx + gridSize);
            if (j > 0) streachParticlesIdx[idx].push_back(idx - 1);
            if (j < gridSize - 1) streachParticlesIdx[idx].push_back(idx + 1);

            // Shearing
            shearParticlesIdx.push_back({});
            if (i > 0 && i < gridSize - 1 && j > 0 && j < gridSize - 1) {
                shearParticlesIdx[idx].push_back({idx - gridSize - 1, idx + gridSize + 1});
                shearParticlesIdx[idx].push_back({idx - gridSize + 1, idx + gridSize - 1});
            }

            // Bending
            bendParticlesIdx.push_back({});
            if (i > 1 && i < gridSize - 2)
                bendParticlesIdx[idx].push_back({idx - 2 * gridSize, idx + 2 * gridSize});
            if (j > 1 && j < gridSize - 2)
                bendParticlesIdx[idx].push_back({idx - 2, idx + 2});
        }
    }

    uint32_t vtxCount = vertices.size();
    uint32_t idxCount = indices.size();
    // Opposite faces with inverted normal
    for (uint32_t i=0; i < vtxCount; i++) {
        vertices.push_back({vertices[i].position, vertices[i].color, -vertices[i].normal, vertices[i].uv});
    }
    
    for (int i = (int)idxCount - 1; i >= 0; i--) {
        indices.push_back(indices[i] + gridSize * gridSize);
    }

    printf("Number of stored cloth points = %zd\n", vertices.size());
    printf("Number of indices = %zd\n", indices.size());
}

void Cloth::Builder::defineTrianglesInCell(int i, int j) {
    // Upper diagonal triangle
    indices.push_back(i * gridSize + j);            // Current
    indices.push_back(i * gridSize + j + 1);        // Right
    indices.push_back((i + 1) * gridSize + j + 1);  // Right-Bottom

    // Lower diagonal triangle
    indices.push_back(i * gridSize + j);            // Current
    indices.push_back((i + 1) * gridSize + j + 1);  // Right-Bottom
    indices.push_back((i + 1) * gridSize + j);      // Bottom
}

void Cloth::Builder::reset() {
    this->vertices.clear();
    this->bendParticlesIdx.clear();
    this->shearParticlesIdx.clear();
    this->streachParticlesIdx.clear();
    this->verticesParticles.clear();
}

void Cloth::loadParticles(TransformComponent &transform) {
    particleCount = builder.vertices.size() / static_cast<uint32_t>(2);

    for (int i=0; i<particleCount; i++) {
        // Create particle attached to vertex
        glm::vec3 worldPos = transform.mat4() * glm::vec4(builder.vertices[i].position, 1.f);

        std::shared_ptr<Particle> particle = std::make_shared<Particle>(worldPos.x, worldPos.y, worldPos.z);
        particle->setBouncing(bouncing);
        particle->setFriction(friction);
        particle->setForce(gravity);
        particle->setSize(0.02f);
        particle->setLifetime(1000);
        particle->setMass(clothMass / (builder.gridSize * builder.gridSize));
        particle->setStiffness(stiffness);
        particle->setDamping(damping);
        particle->setAirFriction(airFriction);

        builder.verticesParticles.push_back(std::move(particle));
    }
    clothLengths.streachLength = builder.cellSize;
    clothLengths.shearLength = 2 * sqrt(2 * builder.cellSize * builder.cellSize);
    clothLengths.bendLength = 4 * builder.cellSize;
}

void Cloth::createVertexBuffers(const std::vector<Vertex> &vertices) {
    // Destroy existing buffer
    if (vertexBuffer != VK_NULL_HANDLE) {
        vkUnmapMemory(device.device(), stagingBufferMemory);
        vkDestroyBuffer(device.device(), stagingBuffer, nullptr);
        vkFreeMemory(device.device(), stagingBufferMemory, nullptr);

        vkDestroyBuffer(device.device(), vertexBuffer, nullptr);
        vkFreeMemory(device.device(), vertexBufferMemory, nullptr);
    }

    vertexCount = static_cast<uint32_t>(vertices.size());
    VkDeviceSize bufferSize = sizeof(Vertex) * vertices.size();

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

void Cloth::createIndexBuffers(const std::vector<uint32_t> &indices) {
    // Destroy existing buffer
    if (indexBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device.device(), indexBuffer, nullptr);
        vkFreeMemory(device.device(), indexBufferMemory, nullptr);
    }

    indexCount = static_cast<uint32_t>(indices.size());
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

void Cloth::draw(VkCommandBuffer commandBuffer) {
    if (hasIndexBuffer) {
        vkCmdDrawIndexed(commandBuffer, indexCount, 1, 0, 0, 0);
    } else {
        vkCmdDraw(commandBuffer, vertexCount, 1, 0, 0);
    }
}

void Cloth::bind(VkCommandBuffer commandBuffer) {
    VkBuffer buffers[] = {vertexBuffer};
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, buffers, offsets);

    if (hasIndexBuffer) {
        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
    }
}

void Cloth::applyGravityForce() {
    for (int i = 0; i < (int)particleCount; i++) {
        builder.verticesParticles[i]->setForce(gravity);
    }
}

void Cloth::update(float dt, KinematicEntities &kinematicEntities, TransformComponent &transform) {
    // compute number of steps to perform
    static float timeSinceLastUpdate{0.f};
    timeSinceLastUpdate += dt;
    int numSteps = static_cast<int>(std::floor(timeSinceLastUpdate / builder.verticesParticles[0]->getTimeStep()));
    timeSinceLastUpdate = timeSinceLastUpdate - numSteps * builder.verticesParticles[0]->getTimeStep();

    glm::mat4 modelInv = glm::inverse(transform.mat4());
    glm::mat4 normalInv = glm::inverse(transform.normalMatrix());

    for (int step = 0; step < numSteps; step++) {
        // Spring forces update pass
        applyGravityForce();
        for (int i = 0; i < (int)particleCount; i++) {
            // Apply streaching
            for (int idx : builder.streachParticlesIdx[i]) {
                builder.verticesParticles[i]->addSpringForce(builder.verticesParticles[idx], clothLengths.streachLength);
            }

            // Apply shearing
            for (auto &idxPair : builder.shearParticlesIdx[i]) {
                builder.verticesParticles[idxPair[0]]->addSpringForce(builder.verticesParticles[idxPair[1]], clothLengths.shearLength);
            }

            // Apply bending
            for (auto &idxPair : builder.bendParticlesIdx[i]) {
                builder.verticesParticles[idxPair[0]]->addSpringForce(builder.verticesParticles[idxPair[1]], clothLengths.bendLength);
            }
        }

        // Regular particle update and collision pass
        int startingParticleIdx = (int)particleCount;  // All fixed
        if (fixed == FIXED_BY_SIDE)
            startingParticleIdx = builder.gridSize;
        else if (fixed == FIXED_BY_CORNER)
            startingParticleIdx = 1;
        else if (fixed == NOT_FIXED)
            startingParticleIdx = 0;
        for (int i = startingParticleIdx; i < (int)particleCount; i++) {
            builder.verticesParticles[i]->updateInScene(dt, numSteps, kinematicEntities, solver);
        }
    }

    for (int i = 0; i < (int)particleCount; i++) {
        builder.vertices[i].position = modelInv * glm::vec4(builder.verticesParticles[i]->getCurrentPosition(), 1.f);
    }

    // rough approximation for normal recomputation: define as cross product between neighborhood directions
    int size = builder.gridSize;
    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size - 1; j++) {
            glm::vec3 normal = glm::cross(builder.vertices[i * size + j + 1].position - builder.vertices[i * size + j - 1].position,
                                          builder.vertices[(i + 1) * size + j].position - builder.vertices[(i - 1) * size + j].position);
            builder.vertices[i * size + j].normal = glm::normalize(normal);
        }
    }
    // propagate normal in borders
    for (int i = 0; i < size; i++) {
        builder.vertices[i].normal = builder.vertices[i + size].normal;                                   // Top row
        builder.vertices[size * (size - 1) + i].normal = builder.vertices[size * (size - 2) + i].normal;  // Bottom row
        builder.vertices[i * size].normal = builder.vertices[i * size + 1].normal;                        // Left column
        builder.vertices[size * (i + 1) - 1].normal = builder.vertices[size * (i + 1) - 2].normal;        // Right column
    }

    // Update back faces info
    for (int i = 0; i < particleCount; i++) {
        builder.vertices[i + particleCount].position = builder.vertices[i].position;
        builder.vertices[i + particleCount].normal = -builder.vertices[i].normal;
    }

    updateBuffers();
}

void Cloth::updateBuffers() {
    VkDeviceSize bufferSize = sizeof(Vertex) * vertexCount;

    memcpy(data, builder.vertices.data(), static_cast<size_t>(bufferSize));
    device.copyBuffer(stagingBuffer, vertexBuffer, bufferSize);
}

std::vector<VkVertexInputBindingDescription> Cloth::Vertex::getBindingDescriptions() {
    std::vector<VkVertexInputBindingDescription> bindingDescriptions(1);
    bindingDescriptions[0].binding = 0;
    bindingDescriptions[0].stride = sizeof(Vertex);
    bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription> Cloth::Vertex::getAttributeDescriptions() {
    std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};

    attributeDescriptions.push_back(
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)});
    attributeDescriptions.push_back(
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
    attributeDescriptions.push_back(
        {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)});
    attributeDescriptions.push_back(
        {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv)});

    return attributeDescriptions;
}

void Cloth::renderUI() {
    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Solver");
    const char *items[] = {"Euler Explicit", "Euler Semi-implicit", "Verlet"};
    ImGui::Combo("##clothSolver", (int *)&solver, items, IM_ARRAYSIZE(items));

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Fix");
    if (ImGui::RadioButton("All", fixed == ALL_FIXED)) {
        fixed = ALL_FIXED;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Edge", fixed == FIXED_BY_SIDE)) {
        fixed = FIXED_BY_SIDE;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Corner", fixed == FIXED_BY_CORNER)) {
        fixed = FIXED_BY_CORNER;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("None", fixed == NOT_FIXED)) {
        fixed = NOT_FIXED;
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Cloth mass");
    ImGui::SliderFloat("##clothMass", &clothMass, 0.001f, 50.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setMass(clothMass / particleCount);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Stiffness");
    ImGui::SliderFloat("##clothStiff", &stiffness, 0.001f, 3000.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setStiffness(stiffness);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Damping");
    ImGui::SliderFloat("##clothDamp", &damping, 0.001f, 10.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setDamping(damping);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Bouncing");
    ImGui::SliderFloat("##clothBounce", &bouncing, 0.001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setBouncing(bouncing);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Friction");
    ImGui::SliderFloat("##clothFriction", &friction, 0.001f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setDamping(friction);
        }
    }

    ImGui::Separator();  // ---------------------------------
    ImGui::Text("Air Friction");
    ImGui::SliderFloat("##clothAirFriction", &airFriction, 0.99f, 1.f);
    if (ImGui::IsItemEdited()) {
        for (auto &particle : builder.verticesParticles) {
            particle->setAirFriction(airFriction);
        }
    }

    ImGui::Text("Force");
    ImGui::SliderFloat3("##force", (float *)&gravity, -2.f, 2.f);
}

}  // namespace vkr
