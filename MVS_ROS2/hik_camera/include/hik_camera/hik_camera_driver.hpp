#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

// 海康 SDK 头文件
#include "/home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK/include/MvCameraControl.h"

/**
 * @brief 海康相机驱动类
 * 提供初始化、采集、重连、参数设置等功能
 */
class HikCameraDriver {
public:
    /**
     * @brief 构造函数
     * @param node ROS 2 节点共享指针
     */
    explicit HikCameraDriver(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief 析构函数
     * 自动清理资源
     */
    ~HikCameraDriver();

    /**
     * @brief 初始化相机（打开设备、设置参数）
     * @return 成功返回 true
     */
    bool initialize();

    /**
     * @brief 开始图像采集
     * @return 成功返回 true
     */
    bool startCapture();

    /**
     * @brief 停止图像采集（不关闭设备）
     */
    void stopCapture();

    /**
     * @brief 关闭设备（释放连接，但不销毁句柄）
     */
    void closeDevice();

private:
    // ========================================
    // 回调函数（Callback Functions）
    // ========================================

    /// 图像回调函数（静态，供 SDK 调用）
    static void __stdcall onImageCallback(
        unsigned char* pData,
        MV_FRAME_OUT_INFO_EX* pFrameInfo,
        void* pUser
    );

    /// 参数设置回调
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters
    );

    /// 重连定时器回调
    void reconnectTimerCallback();

    /// 辅助函数：递增重试次数并判断是否达到上限
    void incrementAndCheckRetry();


    // ========================================
    // 成员变量（Member Variables）
    // ========================================

    // ROS 相关
    std::shared_ptr<rclcpp::Node> node_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 相机连接与状态
    std::atomic<bool> is_connected_{false};   ///< 设备是否已打开
    std::atomic<bool> is_capturing_{false};   ///< 是否正在采集图像

    // 重试机制
    int retry_count_ = 0;                     ///< 当前重试次数
    const int max_retries_ = 10;              ///< 最大重试次数

    // 海康 SDK 相关
    void* camera_handle_ = nullptr;           ///< 相机句柄
    MV_CC_DEVICE_INFO device_info_{};         ///< 设备信息
    std::string camera_ip_;                   ///< 目标相机 IP
    std::string camera_sn_;                   ///< 目标相机 SN
    void printCameraInfo();
};