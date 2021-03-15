#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_vsg/tools/ChVSGScreenshotHandler.h"

using namespace chrono::vsg3d;

ChVSGScreenshotHandler::ChVSGScreenshotHandler(vsg::ref_ptr<vsg::Event> in_event) : event(in_event) {}

void ChVSGScreenshotHandler::printInfo(vsg::ref_ptr<vsg::Window> window) {
    auto device = window->getDevice();
    auto physicalDevice = window->getPhysicalDevice();
    auto swapchain = window->getSwapchain();
    std::cout << "\nNeed to take screenshot " << window << std::endl;
    std::cout << "    device = " << device << std::endl;
    std::cout << "    physicalDevice = " << physicalDevice << std::endl;
    std::cout << "    swapchain = " << swapchain << std::endl;
    std::cout << "        swapchain->getImageFormat() = " << swapchain->getImageFormat() << std::endl;
    std::cout << "        swapchain->getExtent() = " << swapchain->getExtent().width << ", "
              << swapchain->getExtent().height << std::endl;

    for (auto& imageView : swapchain->getImageViews()) {
        std::cout << "        imageview = " << imageView << std::endl;
    }

    std::cout << "    numFrames() = " << window->numFrames() << std::endl;
    for (size_t i = 0; i < window->numFrames(); ++i) {
        std::cout << "        imageview[" << i << "] = " << window->imageView(i) << std::endl;
        std::cout << "        framebuffer[" << i << "] = " << window->framebuffer(i) << std::endl;
    }

    std::cout << "    surfaceFormat() = " << window->surfaceFormat().format << ", "
              << window->surfaceFormat().colorSpace << std::endl;
    std::cout << "    depthFormat() = " << window->depthFormat() << std::endl;
}

void ChVSGScreenshotHandler::screenshot_image(vsg::ref_ptr<vsg::Window> window) {
    // printInfo(window);

    do_image_capture = false;

    if (eventDebugTest && event && event->status() == VK_EVENT_RESET) {
        std::cout << "event->status() == VK_EVENT_RESET" << std::endl;
        // manually wait for the event to be signaled
        while (event->status() == VK_EVENT_RESET) {
            std::cout << "w";
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << std::endl;
    }

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

    bool supportsBlit = ((srcFormatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) != 0) &&
                        ((destFormatProperties.linearTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0);

    if (supportsBlit) {
        // we can automatically convert the image format when blit, so take advantage of it to ensure RGBA
        targetImageFormat = VK_FORMAT_R8G8B8A8_UNORM;
    }

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
    // 3) create command buffer and submit to graphcis queue
    //
    auto commands = vsg::Commands::create();

    if (event) {
        commands->addChild(
            vsg::WaitEvents::create(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, event));
        commands->addChild(vsg::ResetEvent::create(event, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT));
    }

    // 3.a) tranisistion destinationImage to transfer destination initialLayout
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

    if (supportsBlit) {
        // 3.c.1) if blit using VkCmdBliImage
        VkImageBlit region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.srcOffsets[0] = VkOffset3D{0, 0, 0};
        region.srcOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 0};
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.dstOffsets[0] = VkOffset3D{0, 0, 0};
        region.dstOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 0};

        auto blitImage = vsg::BlitImage::create();
        blitImage->srcImage = sourceImage;
        blitImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        blitImage->dstImage = destinationImage;
        blitImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        blitImage->regions.push_back(region);
        blitImage->filter = VK_FILTER_NEAREST;

        commands->addChild(blitImage);
    } else {
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
    }

    // 3.d) tranisition destinate image from transder destination layout to general laytout to enable mapping to image
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

    // 3.e) transition sawp chain image back to present
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

    vsg::submitCommandsToQueue(device, commandPool, fence, 100000000000, queue,
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

    // vsg::Path outputFilename("screenshot.vsgt");
    // vsg::write(imageData, outputFilename);

    // following issue has been found on a iMac with retina display
    if (width > 1600) {
        std::cout << "Could not make the desired screenshot due to STB limitations." << std::endl;
        std::cout << "Framebuffer width = " << width << " pixels." << std::endl;
        std::cout << "Please decrease window width setting to 800!" << std::endl;
        return;
    }
    unsigned char* data = reinterpret_cast<unsigned char*>(imageData->dataPointer());
    const size_t channels = 3;  // we use RGB, RGBA seems to cause output problems
    size_t pixelsDataSize = width * height * channels;
    unsigned char* pixels = new unsigned char[pixelsDataSize];
    if (pixels) {
        size_t k = 0;
        for (size_t j = 0; j < imageData->dataSize(); j += 4) {
            // we have to swap RGB width BGR for output
            pixels[k++] = data[j + 2];
            pixels[k++] = data[j + 1];
            pixels[k++] = data[j];
        }

        // int ans1 = stbi_write_bmp("screenshot.bmp", width, height, 3, pixels);
        int ans2 = stbi_write_png("screenshot.png", width, height, channels, pixels, 0);
        // int ans3 = stbi_write_jpg("screenshot.jpg", width, height, 3, pixels, 100);
        // int ans4 = stbi_write_tga("screenshot.tga", width, height, 3, pixels);

        delete[] pixels;
    } else {
        std::cout << "Not enough memory to make a screenshot! Decrease window size!" << std::endl;
    }
}

void ChVSGScreenshotHandler::screenshot_depth(vsg::ref_ptr<vsg::Window> window) {
    do_depth_capture = false;

    // printInfo(window);
    if (eventDebugTest && event && event->status() == VK_EVENT_RESET) {
        std::cout << "event->status() == VK_EVENT_RESET" << std::endl;
        // manually wait for the event to be signaled
        while (event->status() == VK_EVENT_RESET) {
            std::cout << "w";
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << std::endl;
    }

    auto width = window->extent2D().width;
    auto height = window->extent2D().height;

    auto device = window->getDevice();
    auto physicalDevice = window->getPhysicalDevice();

    vsg::ref_ptr<vsg::Image> sourceImage(window->getDepthImage());

    VkFormat sourceImageFormat = window->depthFormat();
    VkFormat targetImageFormat = sourceImageFormat;

    auto memoryRequirements = sourceImage->getMemoryRequirements(device->deviceID);

    // 1. create buffer to copy to.
    VkDeviceSize bufferSize = memoryRequirements.size;
    auto destinationBuffer =
        vsg::createBufferAndMemory(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_SHARING_MODE_EXCLUSIVE,
                                   VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                                       VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
    auto destinationMemory = destinationBuffer->getDeviceMemory(device->deviceID);

    VkImageAspectFlags imageAspectFlags = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;

    // 2.a) tranition depth image for reading
    auto commands = vsg::Commands::create();

    if (event) {
        commands->addChild(
            vsg::WaitEvents::create(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, event));
        commands->addChild(vsg::ResetEvent::create(event, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT));
    }

    auto transitionSourceImageToTransferSourceLayoutBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,  // srcAccessMask
        VK_ACCESS_TRANSFER_READ_BIT,                                                                 // dstAccessMask
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,                                            // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                                                        // newLayout
        VK_QUEUE_FAMILY_IGNORED,                               // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                               // dstQueueFamilyIndex
        sourceImage,                                           // image
        VkImageSubresourceRange{imageAspectFlags, 0, 1, 0, 1}  // subresourceRange
    );

    auto transitionDestinationBufferToTransferWriteBarrier =
        vsg::BufferMemoryBarrier::create(VK_ACCESS_MEMORY_READ_BIT,     // srcAccessMask
                                         VK_ACCESS_TRANSFER_WRITE_BIT,  // dstAccessMask
                                         VK_QUEUE_FAMILY_IGNORED,       // srcQueueFamilyIndex
                                         VK_QUEUE_FAMILY_IGNORED,       // dstQueueFamilyIndex
                                         destinationBuffer,             // buffer
                                         0,                             // offset
                                         bufferSize                     // size
        );

    auto cmd_transitionSourceImageToTransferSourceLayoutBarrier = vsg::PipelineBarrier::create(
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,  // srcStageMask
        VK_PIPELINE_STAGE_TRANSFER_BIT,                                                             // dstStageMask
        0,                                                                                          // dependencyFlags
        transitionSourceImageToTransferSourceLayoutBarrier,                                         // barrier
        transitionDestinationBufferToTransferWriteBarrier                                           // barrier
    );
    commands->addChild(cmd_transitionSourceImageToTransferSourceLayoutBarrier);

    // 2.b) copy image to buffer
    {
        VkBufferImageCopy region{};
        region.bufferOffset;
        region.bufferRowLength = width;  // need to figure out actual row length from somewhere...
        region.bufferImageHeight = height;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = VkOffset3D{0, 0, 0};
        region.imageExtent = VkExtent3D{width, height, 1};

        auto copyImage = vsg::CopyImageToBuffer::create();
        copyImage->srcImage = sourceImage;
        copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        copyImage->dstBuffer = destinationBuffer;
        copyImage->regions.push_back(region);

        commands->addChild(copyImage);
    }

    // 2.c) transition dpeth image back for rendering
    auto transitionSourceImageBackToPresentBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_READ_BIT,                                                                 // srcAccessMask
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,  // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                                                        // oldLayout
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,                                            // newLayout
        VK_QUEUE_FAMILY_IGNORED,                               // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                               // dstQueueFamilyIndex
        sourceImage,                                           // image
        VkImageSubresourceRange{imageAspectFlags, 0, 1, 0, 1}  // subresourceRange
    );

    auto transitionDestinationBufferToMemoryReadBarrier =
        vsg::BufferMemoryBarrier::create(VK_ACCESS_TRANSFER_WRITE_BIT,  // srcAccessMask
                                         VK_ACCESS_MEMORY_READ_BIT,     // dstAccessMask
                                         VK_QUEUE_FAMILY_IGNORED,       // srcQueueFamilyIndex
                                         VK_QUEUE_FAMILY_IGNORED,       // dstQueueFamilyIndex
                                         destinationBuffer,             // buffer
                                         0,                             // offset
                                         bufferSize                     // size
        );

    auto cmd_transitionSourceImageBackToPresentBarrier = vsg::PipelineBarrier::create(
        VK_PIPELINE_STAGE_TRANSFER_BIT,                                                             // srcStageMask
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,  // dstStageMask
        0,                                                                                          // dependencyFlags
        transitionSourceImageBackToPresentBarrier,                                                  // barrier
        transitionDestinationBufferToMemoryReadBarrier                                              // barrier
    );

    commands->addChild(cmd_transitionSourceImageBackToPresentBarrier);

    auto fence = vsg::Fence::create(device);
    auto queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
    auto commandPool = vsg::CommandPool::create(device, queueFamilyIndex);
    auto queue = device->getQueue(queueFamilyIndex);

    vsg::submitCommandsToQueue(device, commandPool, fence, 100000000000, queue,
                               [&](vsg::CommandBuffer& commandBuffer) { commands->record(commandBuffer); });

    // 3. map buffer and copy data.

    // Map the buffer memory and assign as a vec4Array2D that will automatically unmap itself on destruction.
    if (targetImageFormat == VK_FORMAT_D32_SFLOAT || targetImageFormat == VK_FORMAT_D32_SFLOAT_S8_UINT) {
        auto imageData =
            vsg::MappedData<vsg::floatArray2D>::create(destinationMemory, 0, 0, vsg::Data::Layout{targetImageFormat},
                                                       width, height);  // deviceMemory, offset, flags and dimensions

        size_t num_set_depth = 0;
        size_t num_unset_depth = 0;
        for (auto& value : *imageData) {
            if (value == 1.0f)
                ++num_unset_depth;
            else
                ++num_set_depth;
        }

        std::cout << "num_unset_depth = " << num_unset_depth << std::endl;
        std::cout << "num_set_depth = " << num_set_depth << std::endl;

        vsg::Path outputFilename("depth.vsgt");
        vsg::write(imageData, outputFilename);
    } else {
        auto imageData =
            vsg::MappedData<vsg::uintArray2D>::create(destinationMemory, 0, 0, vsg::Data::Layout{targetImageFormat},
                                                      width, height);  // deviceMemory, offset, flags and dimensions

        vsg::Path outputFilename("depth.vsgt");
        vsg::write(imageData, outputFilename);
    }
}
