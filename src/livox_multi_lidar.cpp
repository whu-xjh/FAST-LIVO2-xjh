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
        sub_159_ = nh_.subscribe(input_topic_159_, 100, &LivoxLidarMerger::lidar159Callback, this);
        sub_160_ = nh_.subscribe(input_topic_160_, 100, &LivoxLidarMerger::lidar160Callback, this);
        sub_161_ = nh_.subscribe(input_topic_161_, 100, &LivoxLidarMerger::lidar161Callback, this);

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
    
    // 合并并发布数据（基于时间戳排序的新版本）
    void mergeAndPublish() {
        if (has_159_data_ && has_160_data_ && has_161_data_) {
            writeLog("Starting timestamp-based merging");
            
            size_t total_points = msg_159_->point_num + msg_160_->point_num + msg_161_->point_num;

            std::vector<std::tuple<uint64_t, int, size_t>> sorted_indices;  // (time, source_id, point_index)
            sorted_indices.reserve(total_points);
            
            size_t idx = 0;
            for (const auto& point : msg_159_->points) {
                uint64_t time = msg_159_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 159, idx++);
            }
            idx = 0;
            for (const auto& point : msg_160_->points) {
                uint64_t time = msg_160_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 160, idx++);
            }
            idx = 0;
            for (const auto& point : msg_161_->points) {
                uint64_t time = msg_161_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 161, idx++); 
            }

            // 排序
            std::sort(sorted_indices.begin(), sorted_indices.end());

            // 创建合并后的消息
            livox_ros_driver::CustomMsg merged_msg;
            if (msg_159_->timebase < msg_160_->timebase && msg_159_->timebase < msg_161_->timebase) {
                merged_msg.header = msg_159_->header;  // 使用159的头部信息
                merged_msg.timebase = msg_159_->timebase; // 159
            } else if (msg_160_->timebase < msg_159_->timebase && msg_160_->timebase < msg_161_->timebase) {
                merged_msg.header = msg_160_->header;  // 使用160的头部信息
                merged_msg.timebase = msg_160_->timebase; // 160
            } else {
                merged_msg.header = msg_161_->header;  // 使用161的头部信息
                merged_msg.timebase = msg_161_->timebase; // 161
            }
            merged_msg.point_num = total_points;
            merged_msg.points.reserve(total_points);

            for (const auto& [time, source_id, point_idx] : sorted_indices) {
                livox_ros_driver::CustomPoint adjusted_point;
                if (source_id == 159) adjusted_point = msg_159_->points[point_idx];
                else if (source_id == 160) adjusted_point = msg_160_->points[point_idx];
                else if (source_id == 161) adjusted_point = msg_161_->points[point_idx];
                else continue; // 不应该发生
                adjusted_point.offset_time = time - merged_msg.timebase;
                merged_msg.points.push_back(adjusted_point);
            }

            /*  Another version of merging topics based on time alignment
                // 创建时间戳点对结构
                struct TimestampedPoint {
                    uint64_t absolute_time;  // 绝对时间戳
                    livox_ros_driver::CustomPoint point;
                    int source_id;  // 159, 160, or 161
                    
                    // 用于排序的比较运算符
                    bool operator<(const TimestampedPoint& other) const {
                        return absolute_time < other.absolute_time;
                    }
                };
                
                // 计算总点数并创建容器
                int total_points = msg_159_->point_num + msg_160_->point_num + msg_161_->point_num;
                std::vector<TimestampedPoint> timestamped_points;
                timestamped_points.reserve(total_points);
                
                // 处理159的点云（作为参考时间基准）
                for (const auto& point : msg_159_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_159_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 159;
                    timestamped_points.push_back(tp);
                }
                
                // 处理160的点云（转换到绝对时间）
                for (const auto& point : msg_160_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_160_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 160;
                    timestamped_points.push_back(tp);
                }
                
                // 处理161的点云（转换到绝对时间）
                for (const auto& point : msg_161_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_161_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 161;
                    timestamped_points.push_back(tp);
                }
                
                // 按时间戳排序
                std::sort(timestamped_points.begin(), timestamped_points.end());
                
                // 创建合并后的消息
                livox_ros_driver::CustomMsg merged_msg;
                merged_msg.header = msg_159_->header;  // 使用159的头部信息
                merged_msg.timebase = msg_159_->timebase;
                merged_msg.point_num = total_points;
                merged_msg.points.reserve(total_points);
                
                if (msg_159_->timebase < msg_160_->timebase && msg_159_->timebase < msg_161_->timebase) {
                    merged_msg.timebase = msg_159_->timebase; // 159
                } else if (msg_160_->timebase < msg_159_->timebase && msg_160_->timebase < msg_161_->timebase) {
                    merged_msg.timebase = msg_160_->timebase; // 160
                } else {
                    merged_msg.timebase = msg_161_->timebase; // 161
                }

                // 将排序后的点转换为相对时间
                for (const auto& tp : timestamped_points) {
                    livox_ros_driver::CustomPoint adjusted_point = tp.point;
                    
                    // 计算相对于timebase的偏移时间
                    if (tp.absolute_time >= merged_msg.timebase) {
                        adjusted_point.offset_time = tp.absolute_time - merged_msg.timebase;
                    } else {
                        // // 处理时间戳小于基准的情况（理论上不应该发生）
                        // writeLog("Warning: Point timestamp " + std::to_string(tp.absolute_time) + 
                        //         " is before reference timebase " + std::to_string(merged_msg.timebase));
                        continue; // 跳过这个点
                    }
                    
                    merged_msg.points.push_back(adjusted_point);
                }
            */

            // 发布合并后的消息
            merged_pub_.publish(merged_msg);
            
            // 统计信息
            uint64_t min_time = std::get<0>(sorted_indices.front());
            uint64_t max_time = std::get<0>(sorted_indices.back());
            double time_span = (max_time - min_time) / 1000000000.0; // 转换为秒
            
            writeLog("Published timestamp-sorted merged message with " + std::to_string(total_points) + " points");
            writeLog("Time span: " + std::to_string(time_span) + " seconds");
            writeLog("Points per source: 159=" + std::to_string(msg_159_->point_num) + 
                    ", 160=" + std::to_string(msg_160_->point_num) + 
                    ", 161=" + std::to_string(msg_161_->point_num));
            
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