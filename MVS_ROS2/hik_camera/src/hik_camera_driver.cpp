#include "hik_camera/hik_camera_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>


static size_t getBitsPerPixel(unsigned int pixelType) {
    switch (pixelType) {
        case PixelType_Gvsp_Mono8:
        case PixelType_Gvsp_RGB8_Packed:
        case PixelType_Gvsp_BGR8_Packed:
            return 8;
        case PixelType_Gvsp_Mono10:
        case PixelType_Gvsp_Mono12:
            return 16;
        case PixelType_Gvsp_Mono10_Packed:
            return 12;  
        case PixelType_Gvsp_Mono12_Packed:
            return 12;
        default:
            return 8;
    }
}

rcl_interfaces::msg::SetParametersResult HikCameraDriver::onParameterChange(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "exposure_time") {
            double exp = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exp);
            // 省略错误处理...
        } else if (param.get_name() == "gain") {
            double gain = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
            // 省略错误处理...
        } else if (param.get_name() == "frame_rate") {
            double fps = param.as_double();
            if (fps > 0) {
                MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
                MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
            } else {
                MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
            }
        } else if (param.get_name() == "pixel_format") {
            std::string fmt = param.as_string();
            int pixelFormat;
            if (fmt == "Mono8") {
                pixelFormat = PixelType_Gvsp_Mono8;
            } else if (fmt == "RGB8") {
                pixelFormat = PixelType_Gvsp_RGB8_Packed;
            } else if (fmt == "BGR8") {
                pixelFormat = PixelType_Gvsp_BGR8_Packed;
            } else {
                result.successful = false;
                result.reason = "Unsupported pixel format: " + fmt;
                return result;
            }
            int nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", pixelFormat);
            if (nRet != MV_OK) {
                result.successful = false;
                result.reason = "Failed to set PixelFormat";
                return result;
            } else {
                RCLCPP_INFO(node_->get_logger(), "Set PixelFormat to %s. Restart the camera driver to apply.", fmt.c_str());
            }
        }
    }

    return result;
}

HikCameraDriver::HikCameraDriver(std::shared_ptr<rclcpp::Node> node)
    : node_(node) // 初始化 node_
{
    node_->declare_parameter("image_topic", "image_raw");
    std::string image_topic = node_->get_parameter("image_topic").as_string();

    // 使用 ImageTransport 创建 Publisher（发布到自定义 topic）
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertise(image_topic, 10); // 队列大小 10

    // 打印日志
    RCLCPP_INFO(node_->get_logger(), "Publishing images to topic: '%s'", image_topic.c_str());
    
    node_->declare_parameter<std::string>("camera_ip", "");
    node_->declare_parameter<std::string>("camera_sn", "");
    
    // 声明重连间隔参数（单位：秒）
    node_->declare_parameter("reconnect_interval", 5); // 默认 5 秒
    int interval_sec = node_->get_parameter("reconnect_interval").as_int();

    // 声明相机控制参数（带默认值）
    node_->declare_parameter("exposure_time", 10000.0);      // 微秒
    node_->declare_parameter("gain", 0.0);                   // dB 或 无单位
    node_->declare_parameter("frame_rate", -1.0);            // -1 表示不限制
    node_->declare_parameter("pixel_format", "Mono8");       // 支持的格式字符串

    // 创建定时器，定期检查连接状态
    reconnect_timer_ = node_->create_wall_timer(
        std::chrono::seconds(interval_sec),
        std::bind(&HikCameraDriver::reconnectTimerCallback, this)
    );

    // 注册参数回调
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&HikCameraDriver::onParameterChange, this, std::placeholders::_1)
    );
}

HikCameraDriver::~HikCameraDriver() {
    // 移除参数回调
    if (param_callback_handle_) {
        node_->remove_on_set_parameters_callback(param_callback_handle_.get());
    }
    
    // 停止相机采集
    closeDevice(); 
}

bool HikCameraDriver::initialize() {
    RCLCPP_INFO(node_->get_logger(), "Initializing HikCameraDriver...");

    //  先清理旧资源
    if (camera_handle_ != nullptr) {
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    camera_ip_ = node_->get_parameter("camera_ip").as_string();
    camera_sn_ = node_->get_parameter("camera_sn").as_string();

    if (camera_ip_.empty() && camera_sn_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Must specify 'camera_ip' or 'camera_sn'");
        return false;
    }

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof(device_list));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (nRet != MV_OK || device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(node_->get_logger(), "No devices found");
        return false;
    }

    // 根据 IP 或 SN 选择设备
    bool found = false;
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO* pDevInfo = device_list.pDeviceInfo[i];

    std::string sn;
    std::string ip;

    if (pDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        // 获取 IP
        unsigned int ip_num = pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        ip = std::to_string((ip_num >> 24) & 0xFF) + "." +
             std::to_string((ip_num >> 16) & 0xFF) + "." +
             std::to_string((ip_num >> 8)  & 0xFF) + "." +
             std::to_string(ip_num & 0xFF);

        // 获取 SN
        sn = std::string((char*)pDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
    }
    else if (pDevInfo->nTLayerType == MV_USB_DEVICE) {
        // USB 设备获取 SN
        sn = std::string((char*)pDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        // USB 没有传统 IP，可以留空或设为 "usb"
        // 这里只考虑了USB接口，因为根据现有条件只能测试USB接口的效果
    }
    else {
        continue; // 忽略其他类型
    }

    // 统一匹配逻辑
    if ((!camera_ip_.empty() && ip == camera_ip_) ||
        (!camera_sn_.empty() && sn == camera_sn_)) {
        memcpy(&device_info_, pDevInfo, sizeof(MV_CC_DEVICE_INFO));
        found = true;
        break;
    }
}

    if (!found) {
        RCLCPP_ERROR(node_->get_logger(), "Camera not found: ip=%s, sn=%s", 
                     camera_ip_.c_str(), camera_sn_.c_str());
        return false;
    }

    // 创建句柄
    nRet = MV_CC_CreateHandle(&camera_handle_, &device_info_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Create handle failed: %d", nRet);
        return false;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Open device failed: %d", nRet);
        return false;
    }
    is_connected_ = true;// 若成功打开设备则标记为已连接，并发布日志
    RCLCPP_INFO(node_->get_logger(), "Camera opened successfully");

    // 确保没有在采集
    bool is_grabbing = false;
    nRet = MV_CC_GetBoolValue(camera_handle_, "IsGrabbing", &is_grabbing);
    if (nRet == MV_OK && is_grabbing) {
        RCLCPP_WARN(node_->get_logger(), "Camera was already grabbing. Stopping...");
        MV_CC_StopGrabbing(camera_handle_);
    }

    // 设置默认图像尺寸（根据相机型号调整）
    nRet = MV_CC_SetIntValue(camera_handle_, "Width", 640);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set Width: %d", nRet);
    }

    nRet = MV_CC_SetIntValue(camera_handle_, "Height", 480);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set Height: %d", nRet);
    }

    std::string pixel_format_str = node_->get_parameter("pixel_format").as_string();
    int pixelFormat;

    if (pixel_format_str == "Mono8") {
        pixelFormat = PixelType_Gvsp_Mono8;
    } else if (pixel_format_str == "RGB8") {
        pixelFormat = PixelType_Gvsp_RGB8_Packed;
    } else if (pixel_format_str == "BGR8") {
        pixelFormat = PixelType_Gvsp_BGR8_Packed;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported pixel format: %s", pixel_format_str.c_str());
        return false;
    }
   
    // 设置像素格式（必须支持）
    nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", pixelFormat);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set PixelFormat: %d", nRet);
    }

    // 设置为连续采集模式
    nRet = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set AcquisitionMode to Continuous");
    }

    // 不限制帧率
    nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to Disable AcquisitionFrameRate");
    }

    RCLCPP_INFO(node_->get_logger(), "Camera initialized");
    return true;
}

bool HikCameraDriver::startCapture() {
    RCLCPP_INFO(node_->get_logger(), "Starting capture...");

    if (!camera_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Camera handle is null");
        return false;
    }

    // 先注册回调函数
    int nRet = MV_CC_RegisterImageCallBackEx(camera_handle_, onImageCallback, this);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Register callback failed: %d", nRet);
        return false;
    }

    // 然后启动采集
    is_capturing_.store(true);
    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Start grabbing failed: %d", nRet);
        is_capturing_.store(false);
        return false;
    }

    // 等待 IsGrabbing 为 true，但不阻塞太久
    const int max_wait_ms = 500;
    const int check_interval_ms = 10;
    int elapsed = 0;
    while (elapsed < max_wait_ms) {
        bool is_grabbing = false;
        int nRet = MV_CC_GetBoolValue(camera_handle_, "IsGrabbing", &is_grabbing);
        if (nRet == MV_OK && is_grabbing) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
        elapsed += check_interval_ms;
    }

    RCLCPP_WARN(node_->get_logger(), "Timeout waiting for IsGrabbing, but capture may still be working.");
    return true; // 仍然返回 true
}

void HikCameraDriver::stopCapture() {
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);
    }
    RCLCPP_INFO(node_->get_logger(), "Capture stopped");
    is_capturing_.store(false); 
}

void HikCameraDriver::closeDevice() {
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    is_connected_.store(false);
    is_capturing_.store(false);
    RCLCPP_INFO(node_->get_logger(), "Device closed");
}

void __stdcall HikCameraDriver::onImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser){
    HikCameraDriver* driver = static_cast<HikCameraDriver*>(pUser);
    if (!driver || !driver->is_capturing_.load() || !pData || !pFrameInfo) {
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("HikCamera"), "Image received: %dx%d, format=%u", 
                 pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->enPixelType);

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = driver->node_->now();
    msg->header.frame_id = "camera_frame";
    msg->height = pFrameInfo->nHeight;
    msg->width = pFrameInfo->nWidth;

    // ... encoding 设置 ...

    msg->is_bigendian = 0;
    size_t bits_per_pixel = getBitsPerPixel(pFrameInfo->enPixelType);
    size_t bytes_per_pixel = (bits_per_pixel + 7) / 8;
    msg->step = pFrameInfo->nWidth * bytes_per_pixel;

    size_t data_size = pFrameInfo->nFrameLen;  // 使用 SDK 提供的长度
    msg->data.resize(data_size);
    memcpy(msg->data.data(), pData, data_size);  // 安全复制

    driver->image_pub_.publish(std::move(msg));
}

void HikCameraDriver::reconnectTimerCallback() {
    if (is_capturing_.load()) {
        retry_count_ = 0;
        return;
    }

    RCLCPP_WARN(node_->get_logger(), "Camera not capturing. Attempting reconnect... (attempt %d/%d)", 
                retry_count_ + 1, max_retries_);

    // 确保完全关闭
    stopCapture();   // 停止采集
    closeDevice();   // 关闭设备并销毁句柄

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 短暂延迟

    if (initialize()) {
        if (startCapture()) {
            RCLCPP_INFO(node_->get_logger(), "Reconnected and capturing successfully!");
            retry_count_ = 0;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Initialized but failed to start capture");
            incrementAndCheckRetry();
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Initialization failed during reconnect");
        incrementAndCheckRetry();
    }
}

// 辅助函数：递增并判断是否达到上限
void HikCameraDriver::incrementAndCheckRetry() {
    retry_count_++;
    if (retry_count_ >= max_retries_) {
        RCLCPP_ERROR(node_->get_logger(), "Max retries (%d) reached. Closing device to reset state.", max_retries_);
        closeDevice();  // 主动关闭设备，释放资源
        retry_count_ = 0;  // 重置，等待下次触发

        // 停止重连定时器，防止下次触发
        if (reconnect_timer_) {
            reconnect_timer_->cancel();
            RCLCPP_INFO(node_->get_logger(), "Reconnect timer canceled.");
        }
        rclcpp::shutdown();
        std::exit(EXIT_FAILURE);
    }
}

void HikCameraDriver::printCameraInfo() {
    if (!is_connected_) return;

    MVCC_INTVALUE int_value;
    MVCC_ENUMVALUE enum_value;
    MVCC_FLOATVALUE float_value;

    // 获取宽度
    memset(&int_value, 0, sizeof(MVCC_INTVALUE));
    int nRet = MV_CC_GetIntValue(camera_handle_, "Width", &int_value);
    if (nRet == MV_OK) {
        RCLCPP_INFO(node_->get_logger(), "Current Width: %ld", int_value.nCurValue);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to get Width: %d", nRet);
    }
    
    // 获取像素格式
    memset(&enum_value, 0, sizeof(MVCC_ENUMVALUE));
    nRet = MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &enum_value);
    if (nRet == MV_OK) {
        RCLCPP_INFO(node_->get_logger(), "Current PixelFormat value: %u", enum_value.nCurValue);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to get PixelFormat: %d", nRet);
    }
  
}
