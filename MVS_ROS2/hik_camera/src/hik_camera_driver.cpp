#include "hik_camera/hik_camera_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>
using namespace std::chrono_literals;


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
            if (nRet != MV_OK) {
                result.successful = false;
                result.reason = "Failed to set ExposureTime: " + std::to_string(exp) + " (error code: " + std::to_string(nRet) + ")";
                RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                return result;
            }
            RCLCPP_INFO(node_->get_logger(), "Successfully set ExposureTime to %.2f", exp);

        } else if (param.get_name() == "gain") {
            double gain = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
            if (nRet != MV_OK) {
                result.successful = false;
                result.reason = "Failed to set Gain: " + std::to_string(gain) + " (error code: " + std::to_string(nRet) + ")";
                RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                return result;
            }
            RCLCPP_INFO(node_->get_logger(), "Successfully set Gain to %.2f", gain);

        } else if (param.get_name() == "frame_rate") {
            double fps = param.as_double();
            int nRet;

            if (fps > 0) {
                nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
                if (nRet != MV_OK) {
                    result.successful = false;
                    result.reason = "Failed to enable AcquisitionFrameRate (error: " + std::to_string(nRet) + ")";
                    RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                    return result;
                }

                nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
                if (nRet != MV_OK) {
                    result.successful = false;
                    result.reason = "Failed to set AcquisitionFrameRate to " + std::to_string(fps) + " (error: " + std::to_string(nRet) + ")";
                    RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                    return result;
                }
                RCLCPP_INFO(node_->get_logger(), "Successfully set Frame Rate to %.2f fps", fps);
            } else {
                nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
                if (nRet != MV_OK) {
                    result.successful = false;
                    result.reason = "Failed to disable AcquisitionFrameRate (error: " + std::to_string(nRet) + ")";
                    RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                    return result;
                }
                RCLCPP_INFO(node_->get_logger(), "Frame Rate control disabled.");
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
                RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                return result;
            }

            int nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", pixelFormat);
            if (nRet != MV_OK) {
                result.successful = false;
                result.reason = "Failed to set PixelFormat to " + fmt + " (error: " + std::to_string(nRet) + ")";
                RCLCPP_ERROR(node_->get_logger(), "%s", result.reason.c_str());
                return result;
            }
            RCLCPP_INFO(node_->get_logger(), "Set PixelFormat to %s. Restart the camera driver to apply.", fmt.c_str());
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
    node_->declare_parameter("exposure_time", 5000.0);      // 微秒
    node_->declare_parameter("gain", 10.0);                   // dB 或 无单位
    node_->declare_parameter("frame_rate", 40.0);            // -1 表示不限制
    node_->declare_parameter("pixel_format", "BGR8");       // 支持的格式字符串

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
    MV_CC_DEVICE_INFO_LIST device_list={0};
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (nRet != MV_OK || device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(node_->get_logger(), "No devices found");
        return false;
    }
    
    MV_CC_DEVICE_INFO* selected_dev_info = nullptr;
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

    nRet = MV_CC_SetBoolValue(camera_handle_, "CounterEventSource", true);
    if (nRet != MV_OK) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set CounterEventSource for timestamp");
    }

    // 启用时间戳（确保 Timestamp 已启用）
    nRet = MV_CC_SetBoolValue(camera_handle_, "GevTimestampControlMode", 1); // 或类似参数

    // 不限制帧率
    // nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
    // if (nRet != MV_OK) {
    //     RCLCPP_WARN(node_->get_logger(), "Failed to Disable AcquisitionFrameRate");
    // }

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

void __stdcall HikCameraDriver::onImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    HikCameraDriver* driver = static_cast<HikCameraDriver*>(pUser);
    if (!driver || !driver->is_capturing_.load() || !pData || !pFrameInfo) {
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("HikCamera"), "Image received: %dx%d, format=%u", 
                 pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->enPixelType);

    // 获取硬件时间戳（单位：纳秒）
    uint64_t current_timestamp_ns = pFrameInfo-> nDevTimeStampLow;
    if (current_timestamp_ns == 0) {
        RCLCPP_WARN(driver->node_->get_logger(), "Received frame with zero hardware timestamp.");
        return;
    }

    // 获取上次时间戳和帧计数
    uint64_t last_ts = driver->last_timestamp_ns_.load();
    int64_t last_log_time = driver->last_print_time_ns_.load();

    // 更新帧计数
    int frame_cnt = ++driver->frame_count_;

    double estimated_fps = 0.0;

    // 只有当已有上一帧时才计算 delta
    if (last_ts != 0) {
        int64_t delta_ns = static_cast<int64_t>(current_timestamp_ns - last_ts);
        if (delta_ns > 0) {
            estimated_fps = 1e9 / static_cast<double>(delta_ns); // FPS ≈ 1 / Δt
        }
    }

    // 更新最后时间戳
    driver->last_timestamp_ns_.store(current_timestamp_ns);

    // 判断是否达到日志输出间隔（使用硬件时间）
    if (current_timestamp_ns - last_log_time >= LOG_INTERVAL_NS) {
        // 尝试原子地更新打印时间，防止多线程冲突
        if (driver->last_print_time_ns_.compare_exchange_strong(last_log_time, current_timestamp_ns)) {
            // 重置帧计数用于下一周期平均帧率估算（可选）
            int fps_avg = frame_cnt - 1; // 因为这一秒已经过去，总帧数约为本周期内收到的帧

            RCLCPP_INFO(driver->node_->get_logger(),
                " Camera FPS | HW-Timestamp Based | Instant: %.2f Hz | Avg over last sec: %d Hz",
                estimated_fps/10, fps_avg/10);

            // 可选：重置帧计数器用于下个周期
            driver->frame_count_.store(1); // 当前帧计入下一周期
        }
    }

    // 创建并发布 ROS 图像消息
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = driver->node_->now(); // ROS 时间仍用于 ROS 系统同步
    msg->header.frame_id = "camera_frame";
    msg->height = pFrameInfo->nHeight;
    msg->width = pFrameInfo->nWidth;

    switch (pFrameInfo->enPixelType) {
        case PixelType_Gvsp_Mono8:
            msg->encoding = sensor_msgs::image_encodings::MONO8;
            break;
        case PixelType_Gvsp_Mono10:
        case PixelType_Gvsp_Mono12:
            msg->encoding = sensor_msgs::image_encodings::MONO16;
            break;
        case PixelType_Gvsp_Mono10_Packed:
        case PixelType_Gvsp_Mono12_Packed:
            msg->encoding = "mono12p"; // 或 "mono10p"
            break;
        case PixelType_Gvsp_RGB8_Packed:
            msg->encoding = sensor_msgs::image_encodings::RGB8;
            break;
        case PixelType_Gvsp_BGR8_Packed:
            msg->encoding = sensor_msgs::image_encodings::BGR8;
            break;
        default:
            RCLCPP_WARN(driver->node_->get_logger(), "Unknown pixel format: %u, using MONO8 as fallback", 
                        static_cast<unsigned int>(pFrameInfo->enPixelType));
            msg->encoding = sensor_msgs::image_encodings::MONO8;
            break;
    }

    msg->is_bigendian = 0;
    size_t bits_per_pixel = getBitsPerPixel(pFrameInfo->enPixelType);
    size_t bytes_per_pixel = (bits_per_pixel + 7) / 8;
    msg->step = pFrameInfo->nWidth * bytes_per_pixel;

    size_t data_size = pFrameInfo->nFrameLen;
    msg->data.resize(data_size);
    memcpy(msg->data.data(), pData, data_size);

    driver->image_pub_.publish(std::move(msg));
}

void HikCameraDriver::reconnectTimerCallback() {
    static int status_counter = 0;
    status_counter++;

    //  每 N 次打印一次状态，避免日志爆炸
    RCLCPP_INFO(node_->get_logger(), 
            " Status Check [count=%d] connected_flag=%d, capturing=%d, handle=%p", 
            status_counter, is_connected_.load(), is_capturing_.load(), camera_handle_);

    //  关键：使用 SDK 查询真实连接状态
    bool sdk_connected = isDeviceConnected();

    if (sdk_connected) {
        // 设备物理连接正常
        retry_count_ = 0;  // 重置重试计数
        return;
    }

    //  物理断开！开始重连流程
    RCLCPP_WARN(node_->get_logger(), " Camera physically disconnected! Starting reconnection attempt %d/%d", 
                retry_count_ + 1, max_retries_);

    //  确保停止采集并释放资源
    stopCapture();
    closeDevice();  // 内部会设置 is_connected_.store(false)

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (initialize()) {
        if (startCapture()) {
            RCLCPP_INFO(node_->get_logger(), " Reconnected and capturing successfully!");
            retry_count_ = 0;
        } else {
            RCLCPP_ERROR(node_->get_logger(), " Start capture failed after re-initialization");
            incrementAndCheckRetry();
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), " Re-initialization failed");
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

bool HikCameraDriver::isDeviceConnected() {
    if (!camera_handle_) {
        RCLCPP_DEBUG(node_->get_logger(), "Camera handle is null. Not connected.");
        return false;
    }

    int nRet = MV_CC_IsDeviceConnected(camera_handle_);
    bool connected = (nRet == 1);

    if (connected) {
        RCLCPP_DEBUG(node_->get_logger(), "Camera is connected.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "Camera disconnected or error: %d", nRet);
    }

    return connected;
}
// void HikCameraDriver::printCameraInfo() {
//     if (!is_connected_) return;

//     MVCC_INTVALUE int_value;
//     MVCC_ENUMVALUE enum_value;
//     MVCC_FLOATVALUE float_value;

//     // 获取宽度
//     memset(&int_value, 0, sizeof(MVCC_INTVALUE));
//     int nRet = MV_CC_GetIntValue(camera_handle_, "Width", &int_value);
//     if (nRet == MV_OK) {
//         RCLCPP_INFO(node_->get_logger(), "Current Width: %ld", int_value.nCurValue);
//     } else {
//         RCLCPP_WARN(node_->get_logger(), "Failed to get Width: %d", nRet);
//     }
    
//     // 获取像素格式
//     memset(&enum_value, 0, sizeof(MVCC_ENUMVALUE));
//     nRet = MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &enum_value);
//     if (nRet == MV_OK) {
//         RCLCPP_INFO(node_->get_logger(), "Current PixelFormat value: %u", enum_value.nCurValue);
//     } else {
//         RCLCPP_WARN(node_->get_logger(), "Failed to get PixelFormat: %d", nRet);
//     }
  


// void HikCameraDriver::checkAndUpdateFps() {
//     if (camera_handle_ == nullptr) return;

//     MVCC_FLOATVALUE resulting, acquisition;
//     int ret1 = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &resulting);
//     int ret2 = MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &acquisition);

//     if (ret1 == MV_OK && ret2 == MV_OK) {
//         RCLCPP_INFO(node_->get_logger(), 
//             " FPS | Set: %.2f | Actual: %.2f", 
//             acquisition.fCurValue, resulting.fCurValue);
//     } else {
//         RCLCPP_WARN(node_->get_logger(), 
//             " Failed to get FPS: Resulting=%d, Acquisition=%d", ret1, ret2);
//     }
