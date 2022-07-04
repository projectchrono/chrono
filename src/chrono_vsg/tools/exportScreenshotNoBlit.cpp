
#include "exportScreenshotNoBlit.h"
#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_thirdparty/stb/stb_image_resize.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

void exportScreenshotNoBlit(vsg::ref_ptr<vsg::Window> window, std::string& imageFilename) {
    auto width = window->extent2D().width;
    auto height = window->extent2D().height;

    auto device = window->getDevice();
    auto physicalDevice = window->getPhysicalDevice();
    auto swapchain = window->getSwapchain();

    // get the colour buffer image of the previous rendered frame as the current frame hasn't been rendered yet.  The 1
    // in window->imageIndex(1) means image from 1 frame ago.
    auto sourceImage = window->imageView(window->imageIndex(1))->image;

    VkFormat sourceImageFormat = swapchain->getImageFormat();
    VkFormat targetImageFormat = sourceImageFormat;

    //
    // 1) Check to see of Blit is supported.
    //
    VkFormatProperties srcFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), sourceImageFormat, &srcFormatProperties);

    VkFormatProperties destFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), VK_FORMAT_R8G8B8A8_UNORM, &destFormatProperties);

    // std::cout<<"supportsBlit = "<<supportsBlit<<std::endl;

    //
    // 2) create image to write to
    //
    auto destinationImage = vsg::Image::create();
    destinationImage->imageType = VK_IMAGE_TYPE_2D;
    destinationImage->format = targetImageFormat;
    destinationImage->extent.width = width;
    destinationImage->extent.height = height;
    destinationImage->extent.depth = 1;
    destinationImage->arrayLayers = 1;
    destinationImage->mipLevels = 1;
    destinationImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    destinationImage->samples = VK_SAMPLE_COUNT_1_BIT;
    destinationImage->tiling = VK_IMAGE_TILING_LINEAR;
    destinationImage->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    destinationImage->compile(device);

    auto deviceMemory =
        vsg::DeviceMemory::create(device, destinationImage->getMemoryRequirements(device->deviceID),
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    destinationImage->bind(deviceMemory, 0);

    //
    // 3) create command buffer and submit to graphics queue
    //
    auto commands = vsg::Commands::create();

    // 3.a) transition destinationImage to transfer destination initialLayout
    auto transitionDestinationImageToDestinationLayoutBarrier = vsg::ImageMemoryBarrier::create(
        0,                                                              // srcAccessMask
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // dstAccessMask
        VK_IMAGE_LAYOUT_UNDEFINED,                                      // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.b) transition swapChainImage from present to transfer source initialLayout
    auto transitionSourceImageToTransferSourceLayoutBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_MEMORY_READ_BIT,                                      // srcAccessMask
        VK_ACCESS_TRANSFER_READ_BIT,                                    // dstAccessMask
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionForTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                        // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                        // dstStageMask
                                     0,                                                     // dependencyFlags
                                     transitionDestinationImageToDestinationLayoutBarrier,  // barrier
                                     transitionSourceImageToTransferSourceLayoutBarrier     // barrier
        );

    commands->addChild(cmd_transitionForTransferBarrier);

    // 3.c.2) else use VkVmdCopyImage

    VkImageCopy region{};
    region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.srcSubresource.layerCount = 1;
    region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.dstSubresource.layerCount = 1;
    region.extent.width = width;
    region.extent.height = height;
    region.extent.depth = 1;

    auto copyImage = vsg::CopyImage::create();
    copyImage->srcImage = sourceImage;
    copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    copyImage->dstImage = destinationImage;
    copyImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    copyImage->regions.push_back(region);

    commands->addChild(copyImage);

    // 3.d) transition destinate image from transfer destination layout to general layout to enable mapping to image
    // DeviceMemory
    auto transitionDestinationImageToMemoryReadBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_GENERAL,                                        // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.e) transition swap chain image back to present
    auto transitionSourceImageBackToPresentBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_READ_BIT,                                    // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionFromTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                 // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                 // dstStageMask
                                     0,                                              // dependencyFlags
                                     transitionDestinationImageToMemoryReadBarrier,  // barrier
                                     transitionSourceImageBackToPresentBarrier       // barrier
        );

    commands->addChild(cmd_transitionFromTransferBarrier);

    auto fence = vsg::Fence::create(device);
    auto queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
    auto commandPool = vsg::CommandPool::create(device, queueFamilyIndex);
    auto queue = device->getQueue(queueFamilyIndex);

    vsg::submitCommandsToQueue(commandPool, fence, 100000000000, queue,
                               [&](vsg::CommandBuffer& commandBuffer) { commands->record(commandBuffer); });

    //
    // 4) map image and copy
    //
    VkImageSubresource subResource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout subResourceLayout;
    vkGetImageSubresourceLayout(*device, destinationImage->vk(device->deviceID), &subResource, &subResourceLayout);

    // Map the buffer memory and assign as a vec4Array2D that will automatically unmap itself on destruction.
    auto imageData = vsg::MappedData<vsg::ubvec4Array2D>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                 vsg::Data::Layout{targetImageFormat}, width,
                                                                 height);  // deviceMemory, offset, flags and dimensions
    // unsigned char* pixels = new unsigned char[nPixelBytes];
    unsigned char* data = (unsigned char*)imageData->dataPointer();
    data += subResourceLayout.offset;
    // If source is BGR (destination is always RGB) and we can't use blit (which does automatic conversion), we'll have
    // to manually swizzle color components
    bool colorSwizzle = false;
    // Check if source is BGR
    // Note: Not complete, only contains most common and basic BGR surface formats for demonstration purposes
    std::vector<VkFormat> formatsBGR = {VK_FORMAT_B8G8R8A8_SRGB, VK_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_B8G8R8A8_SNORM};
    colorSwizzle = (std::find(formatsBGR.begin(), formatsBGR.end(), swapchain->getImageFormat()) != formatsBGR.end());

    std::ostringstream oss(std::ios::out | std::ios::binary);
    for (uint32_t y = 0; y < height; y++) {
        unsigned int* row = (unsigned int*)data;
        for (uint32_t x = 0; x < width; x++) {
            if (colorSwizzle) {
                oss.put(*((char*)row + 2));
                oss.put(*((char*)row + 1));
                oss.put(*((char*)row));
            } else {
                oss.put(*((char*)row));
                oss.put(*((char*)row + 1));
                oss.put(*((char*)row + 2));
            }
            row++;
        }
        data += subResourceLayout.rowPitch;
    }
    // which file format?
    size_t dotPos = imageFilename.find_last_of(".");
    string format;
    if (dotPos != string::npos)
        format = imageFilename.substr(dotPos + 1);
    else
        format = "unknown";
    std::transform(format.begin(), format.end(), format.begin(), [](unsigned char c) { return std::tolower(c); });

    if ((format.compare("png") == 0)) {
        int ans = stbi_write_png(imageFilename.c_str(), width, height, 3, oss.str().c_str(), 0);
    } else if ((format.compare("tga") == 0)) {
        int ans = stbi_write_tga(imageFilename.c_str(), width, height, 3, oss.str().c_str());
    } else if ((format.compare("jpg") == 0) || (format.compare("jpeg") == 0)) {
        int ans = stbi_write_jpg(imageFilename.c_str(), width, height, 3, oss.str().c_str(), 95);
    } else if ((format.compare("bmp") == 0)) {
        int ans = stbi_write_bmp(imageFilename.c_str(), width, height, 3, oss.str().c_str());
    } else {
        cout << "No screen capture written due to unknown image format. Use png, tga, jpg or bmp!" << endl;
    }
}
