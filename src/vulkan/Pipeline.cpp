/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#include <Mesh.hpp>
#include <Pipeline.hpp>

// std
#include <cassert>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace vkr {

Pipeline::Pipeline(Device& device) : device{device} {}

Pipeline::~Pipeline() {
    vkDestroyShaderModule(device.device(), vertShaderModule, nullptr);
    vkDestroyShaderModule(device.device(), fragShaderModule, nullptr);
    vkDestroyPipeline(device.device(), graphicsPipeline, nullptr);
}

std::vector<char> Pipeline::readFile(const std::string& filepath) {
    std::ifstream file{filepath, std::ios::ate | std::ios::binary};

    if (!file.is_open()) {
        throw std::runtime_error("failed to open file: " + filepath);
    }

    size_t fileSize = static_cast<size_t>(file.tellg());
    std::vector<char> buffer(fileSize);

    file.seekg(0);
    file.read(buffer.data(), fileSize);

    file.close();
    return buffer;
}

std::unique_ptr<PipelineSet> Pipeline::createGraphicsPipelines(Device& device,
                                                               const std::vector<ShaderPaths>& shadersFilepaths,
                                                               const std::vector<PipelineConfigInfo>& configInfo,
                                                               std::vector<VertexInputDescriptions>& vertexInputDescriptions) {
    std::unique_ptr<PipelineSet> pipelines(new PipelineSet(std::make_shared<Pipeline>(device),
                                                           std::make_shared<Pipeline>(device),
                                                           std::make_shared<Pipeline>(device)));
    std::vector<std::shared_ptr<Pipeline>> pipelinesVector = {pipelines->meshes, pipelines->hair, pipelines->skybox};
    uint32_t numPipelines = static_cast<uint32_t>(pipelinesVector.size());

    std::vector<VkGraphicsPipelineCreateInfo> pipelinesInfo(numPipelines);
    std::vector<VkPipeline> vkPipelines(numPipelines);
    std::vector<VkPipelineVertexInputStateCreateInfo> pipelinesVertexInputInfos(numPipelines);
    std::vector<std::vector<VkPipelineShaderStageCreateInfo>> shaderStages(numPipelines);
    for (auto shaderStage : shaderStages) {
        shaderStage.resize(2);
    }

    // set info for each pipeline
    for (uint32_t i = 0; i < numPipelines; i++) {
        auto* currentPipelineConfigInfo = &configInfo[i];
        assert(
            currentPipelineConfigInfo->pipelineLayout != VK_NULL_HANDLE &&
            "Cannot create graphics pipeline: no pipelineLayout provided in configInfo");
        assert(
            currentPipelineConfigInfo->renderPass != VK_NULL_HANDLE &&
            "Cannot create graphics pipeline: no renderPass provided in configInfo");

        vkPipelines.push_back(pipelinesVector[i]->graphicsPipeline);

        auto vertCode = readFile(shadersFilepaths[i].vertFilepath);
        auto fragCode = readFile(shadersFilepaths[i].fragFilepath);

        createShaderModule(device, vertCode, &pipelinesVector[i]->vertShaderModule);
        createShaderModule(device, fragCode, &pipelinesVector[i]->fragShaderModule);

        createShaderStageInfo(pipelinesVector[i]->vertShaderModule, pipelinesVector[i]->fragShaderModule, shaderStages[i]);

        auto* vertexInputDescription = &vertexInputDescriptions[i];
        auto* vertexInputInfo = &pipelinesVertexInputInfos[i];
        vertexInputInfo->sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputInfo->vertexAttributeDescriptionCount =
            static_cast<uint32_t>(vertexInputDescription->attributeDescription.size());
        vertexInputInfo->vertexBindingDescriptionCount = static_cast<uint32_t>(vertexInputDescription->bindingDescription.size());
        vertexInputInfo->pVertexAttributeDescriptions = vertexInputDescription->attributeDescription.data();
        vertexInputInfo->pVertexBindingDescriptions = vertexInputDescription->bindingDescription.data();

        VkGraphicsPipelineCreateInfo* pipelineInfo = &pipelinesInfo[i];
        pipelineInfo->sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo->stageCount = 2;
        pipelineInfo->pStages = shaderStages[i].data();

        pipelineInfo->pVertexInputState = vertexInputInfo;
        pipelineInfo->pInputAssemblyState = &currentPipelineConfigInfo->inputAssemblyInfo;
        pipelineInfo->pViewportState = &currentPipelineConfigInfo->viewportInfo;
        pipelineInfo->pRasterizationState = &currentPipelineConfigInfo->rasterizationInfo;
        pipelineInfo->pMultisampleState = &currentPipelineConfigInfo->multisampleInfo;
        pipelineInfo->pColorBlendState = &currentPipelineConfigInfo->colorBlendInfo;
        pipelineInfo->pDepthStencilState = &currentPipelineConfigInfo->depthStencilInfo;
        pipelineInfo->pDynamicState = &currentPipelineConfigInfo->dynamicStateInfo;

        pipelineInfo->layout = currentPipelineConfigInfo->pipelineLayout;
        pipelineInfo->renderPass = currentPipelineConfigInfo->renderPass;
        pipelineInfo->subpass = currentPipelineConfigInfo->subpass;

        pipelineInfo->basePipelineIndex = -1;
        pipelineInfo->basePipelineHandle = VK_NULL_HANDLE;
    }

    if (vkCreateGraphicsPipelines(
            device.device(),
            VK_NULL_HANDLE,
            numPipelines,
            pipelinesInfo.data(),
            nullptr,
            vkPipelines.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline");
    }

    pipelines->meshes->graphicsPipeline = vkPipelines[0];
    pipelines->hair->graphicsPipeline = vkPipelines[1];
    pipelines->skybox->graphicsPipeline = vkPipelines[2];

    return pipelines;
}

void Pipeline::createShaderModule(Device& device, const std::vector<char>& code, VkShaderModule* shaderModule) {
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    if (vkCreateShaderModule(device.device(), &createInfo, nullptr, shaderModule) != VK_SUCCESS) {
        throw std::runtime_error("failed to create shader module");
    }
}

void Pipeline::createShaderStageInfo(VkShaderModule& vertShader,
                                     VkShaderModule& fragShader,
                                     std::vector<VkPipelineShaderStageCreateInfo>& shaderStages) {
    // Vertex shader
    shaderStages.push_back({});
    shaderStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    shaderStages[0].module = vertShader;
    shaderStages[0].pName = "main";
    shaderStages[0].flags = 0;
    shaderStages[0].pNext = nullptr;
    shaderStages[0].pSpecializationInfo = nullptr;

    // Fragment shader
    shaderStages.push_back({});
    shaderStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    shaderStages[1].module = fragShader;
    shaderStages[1].pName = "main";
    shaderStages[1].flags = 0;
    shaderStages[1].pNext = nullptr;
    shaderStages[1].pSpecializationInfo = nullptr;
}

void Pipeline::bind(VkCommandBuffer commandBuffer) {
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);
}

void Pipeline::defaultPipelineConfigInfo(PipelineConfigInfo& configInfo, Device& device, bool useMSAA) {
    configInfo.inputAssemblyInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    configInfo.inputAssemblyInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    configInfo.inputAssemblyInfo.primitiveRestartEnable = VK_FALSE;

    configInfo.viewportInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    configInfo.viewportInfo.viewportCount = 1;
    configInfo.viewportInfo.pViewports = nullptr;
    configInfo.viewportInfo.scissorCount = 1;
    configInfo.viewportInfo.pScissors = nullptr;

    configInfo.rasterizationInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    configInfo.rasterizationInfo.depthClampEnable = VK_FALSE;
    configInfo.rasterizationInfo.rasterizerDiscardEnable = VK_FALSE;
    configInfo.rasterizationInfo.polygonMode = VK_POLYGON_MODE_FILL;
    configInfo.rasterizationInfo.lineWidth = 1.0f;
    configInfo.rasterizationInfo.cullMode = VK_CULL_MODE_FRONT_BIT;
    configInfo.rasterizationInfo.frontFace = VK_FRONT_FACE_CLOCKWISE;
    configInfo.rasterizationInfo.depthBiasEnable = VK_FALSE;
    configInfo.rasterizationInfo.depthBiasConstantFactor = 0.0f;  // Optional
    configInfo.rasterizationInfo.depthBiasClamp = 0.0f;           // Optional
    configInfo.rasterizationInfo.depthBiasSlopeFactor = 0.0f;     // Optional

    configInfo.multisampleInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    configInfo.multisampleInfo.sampleShadingEnable = VK_FALSE;
    configInfo.multisampleInfo.rasterizationSamples = useMSAA ? device.msaaSamples() : VK_SAMPLE_COUNT_1_BIT;
    configInfo.multisampleInfo.minSampleShading = 1.0f;           // Optional
    configInfo.multisampleInfo.pSampleMask = nullptr;             // Optional
    configInfo.multisampleInfo.alphaToCoverageEnable = VK_FALSE;  // Optional
    configInfo.multisampleInfo.alphaToOneEnable = VK_FALSE;       // Optional

    configInfo.colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
        VK_COLOR_COMPONENT_A_BIT;
    configInfo.colorBlendAttachment.blendEnable = VK_FALSE;
    configInfo.colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;   // Optional
    configInfo.colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;  // Optional
    configInfo.colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;              // Optional
    configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;   // Optional
    configInfo.colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;  // Optional
    configInfo.colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;              // Optional

    configInfo.colorBlendInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    configInfo.colorBlendInfo.logicOpEnable = VK_FALSE;
    configInfo.colorBlendInfo.logicOp = VK_LOGIC_OP_COPY;  // Optional
    configInfo.colorBlendInfo.attachmentCount = 1;
    configInfo.colorBlendInfo.pAttachments = &configInfo.colorBlendAttachment;
    configInfo.colorBlendInfo.blendConstants[0] = 0.0f;  // Optional
    configInfo.colorBlendInfo.blendConstants[1] = 0.0f;  // Optional
    configInfo.colorBlendInfo.blendConstants[2] = 0.0f;  // Optional
    configInfo.colorBlendInfo.blendConstants[3] = 0.0f;  // Optional

    configInfo.depthStencilInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    configInfo.depthStencilInfo.depthTestEnable = VK_TRUE;
    configInfo.depthStencilInfo.depthWriteEnable = VK_TRUE;
    configInfo.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS;
    configInfo.depthStencilInfo.depthBoundsTestEnable = VK_FALSE;
    configInfo.depthStencilInfo.minDepthBounds = 0.0f;  // Optional
    configInfo.depthStencilInfo.maxDepthBounds = 1.0f;  // Optional
    configInfo.depthStencilInfo.stencilTestEnable = VK_FALSE;
    configInfo.depthStencilInfo.front = {};  // Optional
    configInfo.depthStencilInfo.back = {};   // Optional

    configInfo.dynamicStateEnables = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    configInfo.dynamicStateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    configInfo.dynamicStateInfo.pDynamicStates = configInfo.dynamicStateEnables.data();
    configInfo.dynamicStateInfo.dynamicStateCount =
        static_cast<uint32_t>(configInfo.dynamicStateEnables.size());
    configInfo.dynamicStateInfo.flags = 0;
}

}  // namespace vkr
