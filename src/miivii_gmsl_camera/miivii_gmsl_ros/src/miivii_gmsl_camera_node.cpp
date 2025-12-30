#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"
#include <ros/ros.h>
#include <csignal>
#include <thread>
#include <atomic>

std::shared_ptr<miivii_gmsl::MiiviiGmslCamera> node = nullptr;
std::atomic<bool> quit{false};
std::atomic<bool> request{false};

void signal_handler(int signal) {
    ROS_INFO("Received SIGINT, shutting down...");
    request = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "miivii_gmsl_camera");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    node = std::make_shared<miivii_gmsl::MiiviiGmslCamera>(nh, pnh);
    
    signal(SIGINT, signal_handler);
    
    // 获取发布频率
    double publish_rate = 30.0; // 默认值
    pnh.param("publish_rate", publish_rate, 30.0);
    auto loop_duration = std::chrono::microseconds(static_cast<int64_t>(1000000 / publish_rate));
    
    ROS_INFO("Starting main loop at %.1f Hz", publish_rate);
    
    auto last_time = std::chrono::steady_clock::now();
    uint64_t loop_count = 0;
    
    // 主循环
    while (!quit && ros::ok()) {
        if (request) {
            ROS_INFO("Shutdown requested");
            quit = true;
            break;
        }
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time);
        
        // 精确控制调用频率
        if (elapsed >= loop_duration) {
            try {
                // 使用try-catch避免异常导致程序退出
                node->timer_callback();
                loop_count++;
                
                // 每100次循环打印一次状态
                if (loop_count % 100 == 0) {
                    ROS_DEBUG("Main loop running, count: %lu", loop_count);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in timer_callback: %s", e.what());
                // 短暂休息后继续
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            last_time = current_time;
        }
        
        // 处理ROS消息（非阻塞）
        ros::spinOnce();
        
        // 避免忙等待
        auto time_to_wait = loop_duration - std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - last_time);
        if (time_to_wait > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(time_to_wait);
        }
    }
    
    ROS_INFO("Main loop exited after %lu iterations", loop_count);
    ros::shutdown();
    node = nullptr;
    return 0;
}
