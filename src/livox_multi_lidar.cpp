#include <ros/ros.h>
#include <ros/package.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <fstream>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>

class LivoxLidarMerger {
public:
    LivoxLidarMerger(ros::NodeHandle& nh) : nh_(nh) {
        // 获取包路径
        std::string package_path = ros::package::getPath("fast_livo");
        log_dir_ = package_path + "/Log/launch_log";
        
        // 创建Log目录（如果不存在）
        mkdir(log_dir_.c_str(), 0777);
        
        // 生成带时间戳的日志文件名
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << log_dir_ << "/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
        log_file_path_ = ss.str();
        
        // 打开日志文件
        log_file_.open(log_file_path_, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
        } else {
            ROS_INFO("Log file created: %s", log_file_path_.c_str());
        }
        
        // 获取参数：三个输入话题和一个输出话题
        nh_.param<std::string>("output_topic", output_topic_, "/livox/multi_lidar");
        nh_.param<std::string>("input_topic_159", input_topic_159_, "/livox/lidar_192_168_1_159");
        nh_.param<std::string>("input_topic_160", input_topic_160_, "/livox/lidar_192_168_1_160");
        nh_.param<std::string>("input_topic_161", input_topic_161_, "/livox/lidar_192_168_1_161");
        
        // 创建ROS发布者，用于发布合并后的点云数据，队列长度为10
        merged_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 10);
        
        // 创建订阅者
        sub_159_ = nh_.subscribe(input_topic_159_, 10, &LivoxLidarMerger::lidar159Callback, this);
        sub_160_ = nh_.subscribe(input_topic_160_, 10, &LivoxLidarMerger::lidar160Callback, this);
        sub_161_ = nh_.subscribe(input_topic_161_, 10, &LivoxLidarMerger::lidar161Callback, this);
        
        // 初始化标志位和缓存
        has_159_data_ = false;
        has_160_data_ = false;
        has_161_data_ = false;
        
        writeLog("Livox Multi Lidar initialized");
        writeLog("Input topics: " + input_topic_159_ + ", " + input_topic_160_ + ", " + input_topic_161_);
        writeLog("Output topic: " + output_topic_);
    }
    
    ~LivoxLidarMerger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    void writeLog(const std::string& message) {
        if (log_file_.is_open()) {
            // 添加时间戳
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
            
            log_file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") 
                     << "." << std::setfill('0') << std::setw(3) << ms.count() 
                     << " - " << message << std::endl;
            log_file_.flush();
        }
    }
    
    // 合并并发布数据
    void mergeAndPublish() {
        if (has_159_data_ && has_160_data_ && has_161_data_) {
            // 创建合并后的消息
            livox_ros_driver::CustomMsg merged_msg;
            
            // 复制头部信息（使用159的头部信息）
            merged_msg.header = msg_159_->header;
            merged_msg.timebase = msg_159_->timebase;
            
            // 计算160、161的timebase相对于159的偏移
            uint64_t timebase_offset_160 = msg_160_->timebase - msg_159_->timebase;
            uint64_t timebase_offset_161 = msg_161_->timebase - msg_159_->timebase;
            
            writeLog("Timebase offsets: 160=" + std::to_string(timebase_offset_160) + 
                    ", 161=" + std::to_string(timebase_offset_161));

            // 计算总点数
            int total_points = msg_159_->point_num + msg_160_->point_num + msg_161_->point_num;
            merged_msg.point_num = total_points;
            
            // 预分配空间
            merged_msg.points.reserve(total_points);
            
            // 复制159的点（不需要时间偏移，保持原始时间）
            merged_msg.points.insert(merged_msg.points.end(), msg_159_->points.begin(), msg_159_->points.end());
            
            // 优化：先插入再批量修改
            size_t start_160 = merged_msg.points.size();
            merged_msg.points.insert(merged_msg.points.end(), msg_160_->points.begin(), msg_160_->points.end());
            
            // 批量修改160的时间戳（仍需遍历，但减少了复制操作）
            for (size_t i = start_160; i < merged_msg.points.size(); ++i) {
                merged_msg.points[i].offset_time += timebase_offset_160;
            }
            
            // 对161做同样处理
            size_t start_161 = merged_msg.points.size();
            merged_msg.points.insert(merged_msg.points.end(), msg_161_->points.begin(), msg_161_->points.end());
            
            for (size_t i = start_161; i < merged_msg.points.size(); ++i) {
                merged_msg.points[i].offset_time += timebase_offset_161;
            }
            
            // 发布合并后的消息
            merged_pub_.publish(merged_msg);
            writeLog("Published merged message with " + std::to_string(total_points) + " points (" +
                    std::to_string(msg_159_->point_num) + " from 159, " +
                    std::to_string(msg_160_->point_num) + " from 160 (offset+" + std::to_string(timebase_offset_160) + "), " +
                    std::to_string(msg_161_->point_num) + " from 161 (offset+" + std::to_string(timebase_offset_161) + "))");
            
            // 重置标志位和缓存
            has_159_data_ = false;
            has_160_data_ = false;
            has_161_data_ = false;
            msg_159_.reset();
            msg_160_.reset();
            msg_161_.reset();
        }
    }

    void lidar159Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // 保存159的数据
        msg_159_ = msg;
        has_159_data_ = true;
        writeLog("Received 159 LiDAR message with " + std::to_string(msg->point_num) + " points");
        
        // 尝试合并发布
        // mergeAndPublish();
    }
    
    void lidar160Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // 如果已经收到159的数据，才处理160的数据
        if (has_159_data_) {
            msg_160_ = msg;
            has_160_data_ = true;
            writeLog("Received 160 LiDAR message with " + std::to_string(msg->point_num) + " points");
            
            // 尝试合并发布
            // mergeAndPublish();
        } else {
            writeLog("Received 160 LiDAR out of sequence, waiting for 159 data first");
        }
    }
    
    void lidar161Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // 如果已经收到159和160的数据，才处理161的数据
        if (has_159_data_ && has_160_data_) {
            msg_161_ = msg;
            has_161_data_ = true;
            writeLog("Received 161 LiDAR message with " + std::to_string(msg->point_num) + " points");
            
            // 尝试合并发布
            mergeAndPublish();
        } else {
            writeLog("Received 161 LiDAR out of sequence, waiting for 159 and 160 data first");
        }
    }
    
    ros::NodeHandle nh_;
    ros::Publisher merged_pub_;
    ros::Subscriber sub_159_, sub_160_, sub_161_;
    
    std::string output_topic_;
    std::string input_topic_159_, input_topic_160_, input_topic_161_;
    std::string log_dir_;
    std::string log_file_path_;
    std::ofstream log_file_; // 日志文件流
    
    // 用于存储接收到的消息
    livox_ros_driver::CustomMsg::ConstPtr msg_159_;
    livox_ros_driver::CustomMsg::ConstPtr msg_160_;
    livox_ros_driver::CustomMsg::ConstPtr msg_161_;
    
    // 标志位，表示是否已收到各个激光雷达的数据
    bool has_159_data_;
    bool has_160_data_;
    bool has_161_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_multi_lidar");
    ros::NodeHandle nh;
    
    LivoxLidarMerger merger(nh);
    
    ros::spin();
    return 0;
}