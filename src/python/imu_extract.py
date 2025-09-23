#!/usr/bin/env python3
import rosbag
import pandas as pd
import numpy as np
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import os
from datetime import datetime

def extract_imu_data_from_bag(bag_path, output_dir="./imu_data"):
    """从rosbag中提取两个IMU节点的数据"""
    
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 定义话题名称
    internal_imu_topic = "/livox/imu_192_168_1_159"
    external_imu_topic = "/gps/imu"
    
    # 存储数据的列表
    internal_imu_data = []
    external_imu_data = []
    
    print(f"开始处理rosbag: {bag_path}")
    
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            # 获取bag信息
            bag_info = bag.get_type_and_topic_info()
            topics = bag_info[1]
            
            print("可用的话题:")
            for topic, info in topics.items():
                if 'imu' in topic.lower():
                    print(f"  {topic}: {info.message_count} msgs ({info.msg_type})")
            
            # 检查话题是否存在
            if internal_imu_topic not in topics:
                print(f"警告: {internal_imu_topic} 不存在于bag中")
            if external_imu_topic not in topics:
                print(f"警告: {external_imu_topic} 不存在于bag中")
            
            # 读取内部IMU数据
            print(f"\n读取 {internal_imu_topic} 数据...")
            for topic, msg, t in bag.read_messages(topics=[internal_imu_topic]):
                imu_data = {
                    'timestamp': t.to_sec(),
                    'ros_time': t.to_nsec(),
                    'header_stamp': msg.header.stamp.to_sec(),
                    'frame_id': msg.header.frame_id,
                    # 线性加速度
                    'linear_acc_x': msg.linear_acceleration.x,
                    'linear_acc_y': msg.linear_acceleration.y,
                    'linear_acc_z': msg.linear_acceleration.z,
                    # 角速度
                    'angular_vel_x': msg.angular_velocity.x,
                    'angular_vel_y': msg.angular_velocity.y,
                    'angular_vel_z': msg.angular_velocity.z,
                    # 姿态四元数
                    'orientation_x': msg.orientation.x,
                    'orientation_y': msg.orientation.y,
                    'orientation_z': msg.orientation.z,
                    'orientation_w': msg.orientation.w,
                    # 协方差矩阵（可选）
                    'linear_acc_cov': list(msg.linear_acceleration_covariance),
                    'angular_vel_cov': list(msg.angular_velocity_covariance),
                    'orientation_cov': list(msg.orientation_covariance)
                }
                internal_imu_data.append(imu_data)
            
            # 读取外部IMU数据
            print(f"读取 {external_imu_topic} 数据...")
            for topic, msg, t in bag.read_messages(topics=[external_imu_topic]):
                imu_data = {
                    'timestamp': t.to_sec(),
                    'ros_time': t.to_nsec(),
                    'header_stamp': msg.header.stamp.to_sec(),
                    'frame_id': msg.header.frame_id,
                    # 线性加速度
                    'linear_acc_x': msg.linear_acceleration.x,
                    'linear_acc_y': msg.linear_acceleration.y,
                    'linear_acc_z': msg.linear_acceleration.z,
                    # 角速度
                    'angular_vel_x': msg.angular_velocity.x,
                    'angular_vel_y': msg.angular_velocity.y,
                    'angular_vel_z': msg.angular_velocity.z,
                    # 姿态四元数
                    'orientation_x': msg.orientation.x,
                    'orientation_y': msg.orientation.y,
                    'orientation_z': msg.orientation.z,
                    'orientation_w': msg.orientation.w,
                    # 协方差矩阵
                    'linear_acc_cov': list(msg.linear_acceleration_covariance),
                    'angular_vel_cov': list(msg.angular_velocity_covariance),
                    'orientation_cov': list(msg.orientation_covariance)
                }
                external_imu_data.append(imu_data)
    
    except Exception as e:
        print(f"读取bag文件时出错: {e}")
        return None, None
    
    print(f"\n数据提取完成:")
    print(f"内部IMU数据点: {len(internal_imu_data)}")
    print(f"外部IMU数据点: {len(external_imu_data)}")
    
    # 转换为DataFrame
    if internal_imu_data:
        internal_df = pd.DataFrame(internal_imu_data)
        internal_csv_path = os.path.join(output_dir, "livox_imu_data.csv")
        internal_df.to_csv(internal_csv_path, index=False)
        print(f"内部IMU数据已保存至: {internal_csv_path}")
    
    if external_imu_data:
        external_df = pd.DataFrame(external_imu_data)
        external_csv_path = os.path.join(output_dir, "gps_imu_data.csv")
        external_df.to_csv(external_csv_path, index=False)
        print(f"外部IMU数据已保存至: {external_csv_path}")
    
    return internal_imu_data, external_imu_data

def plot_imu_comparison(internal_data, external_data, output_dir="./imu_data"):
    """绘制两个IMU数据的对比图"""
    
    if not internal_data or not external_data:
        print("数据不足，无法绘制对比图")
        return
    
    # 转换为numpy数组便于处理
    internal_df = pd.DataFrame(internal_data)
    external_df = pd.DataFrame(external_data)
    
    # 创建对比图
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('IMU数据对比 (Livox vs GPS)', fontsize=16)
    
    # 加速度对比
    axes[0, 0].plot(internal_df['timestamp'], internal_df['linear_acc_x'], 'b-', label='Livox IMU', alpha=0.7)
    axes[0, 0].plot(external_df['timestamp'], external_df['linear_acc_x'], 'r-', label='GPS IMU', alpha=0.7)
    axes[0, 0].set_title('线性加速度 X')
    axes[0, 0].set_ylabel('加速度 (m/s²)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(internal_df['timestamp'], internal_df['linear_acc_y'], 'b-', label='Livox IMU', alpha=0.7)
    axes[0, 1].plot(external_df['timestamp'], external_df['linear_acc_y'], 'r-', label='GPS IMU', alpha=0.7)
    axes[0, 1].set_title('线性加速度 Y')
    axes[0, 1].set_ylabel('加速度 (m/s²)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # 角速度对比
    axes[1, 0].plot(internal_df['timestamp'], internal_df['angular_vel_x'], 'b-', label='Livox IMU', alpha=0.7)
    axes[1, 0].plot(external_df['timestamp'], external_df['angular_vel_x'], 'r-', label='GPS IMU', alpha=0.7)
    axes[1, 0].set_title('角速度 X')
    axes[1, 0].set_ylabel('角速度 (rad/s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(internal_df['timestamp'], internal_df['angular_vel_y'], 'b-', label='Livox IMU', alpha=0.7)
    axes[1, 1].plot(external_df['timestamp'], external_df['angular_vel_y'], 'r-', label='GPS IMU', alpha=0.7)
    axes[1, 1].set_title('角速度 Y')
    axes[1, 1].set_ylabel('角速度 (rad/s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # 加速度幅值对比
    internal_acc_mag = np.sqrt(internal_df['linear_acc_x']**2 + 
                              internal_df['linear_acc_y']**2 + 
                              internal_df['linear_acc_z']**2)
    external_acc_mag = np.sqrt(external_df['linear_acc_x']**2 + 
                              external_df['linear_acc_y']**2 + 
                              external_df['linear_acc_z']**2)
    
    axes[2, 0].plot(internal_df['timestamp'], internal_acc_mag, 'b-', label='Livox IMU', alpha=0.7)
    axes[2, 0].plot(external_df['timestamp'], external_acc_mag, 'r-', label='GPS IMU', alpha=0.7)
    axes[2, 0].set_title('加速度幅值')
    axes[2, 0].set_ylabel('加速度幅值 (m/s²)')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # 角速度幅值对比
    internal_gyro_mag = np.sqrt(internal_df['angular_vel_x']**2 + 
                               internal_df['angular_vel_y']**2 + 
                               internal_df['angular_vel_z']**2)
    external_gyro_mag = np.sqrt(external_df['angular_vel_x']**2 + 
                               external_df['angular_vel_y']**2 + 
                               external_df['angular_vel_z']**2)
    
    axes[2, 1].plot(internal_df['timestamp'], internal_gyro_mag, 'b-', label='Livox IMU', alpha=0.7)
    axes[2, 1].plot(external_df['timestamp'], external_gyro_mag, 'r-', label='GPS IMU', alpha=0.7)
    axes[2, 1].set_title('角速度幅值')
    axes[2, 1].set_ylabel('角速度幅值 (rad/s)')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    
    # 保存图像
    plot_path = os.path.join(output_dir, "imu_comparison.png")
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"对比图已保存至: {plot_path}")
    plt.show()

def analyze_imu_synchronization(internal_data, external_data):
    """分析两个IMU数据的同步情况"""
    
    if not internal_data or not external_data:
        print("数据不足，无法分析同步性")
        return
    
    internal_df = pd.DataFrame(internal_data)
    external_df = pd.DataFrame(external_data)
    
    print("\n=== IMU数据同步分析 ===")
    print(f"Livox IMU:")
    print(f"  数据点数: {len(internal_df)}")
    print(f"  起始时间: {internal_df['timestamp'].min():.6f}")
    print(f"  结束时间: {internal_df['timestamp'].max():.6f}")
    print(f"  持续时间: {internal_df['timestamp'].max() - internal_df['timestamp'].min():.2f}秒")
    print(f"  平均频率: {len(internal_df) / (internal_df['timestamp'].max() - internal_df['timestamp'].min()):.1f} Hz")
    
    print(f"\nGPS IMU:")
    print(f"  数据点数: {len(external_df)}")
    print(f"  起始时间: {external_df['timestamp'].min():.6f}")
    print(f"  结束时间: {external_df['timestamp'].max():.6f}")
    print(f"  持续时间: {external_df['timestamp'].max() - external_df['timestamp'].min():.2f}秒")
    print(f"  平均频率: {len(external_df) / (external_df['timestamp'].max() - external_df['timestamp'].min()):.1f} Hz")
    
    # 时间差分析
    time_offset = internal_df['timestamp'].min() - external_df['timestamp'].min()
    print(f"\n时间偏移: {time_offset:.6f}秒")

def main():
    # 设置参数
    bag_file_path = "/media/xjh/Extreme SSD/data/shanxi/car/rosbag/road_2025-09-12-12-06-27.bag"  # 默认路径

    output_directory = "./imu_analysis_output"
    
    # 提取数据
    internal_data, external_data = extract_imu_data_from_bag(bag_file_path, output_directory)
    
    if internal_data or external_data:
        # 分析同步性
        analyze_imu_synchronization(internal_data, external_data)
        
        # 绘制对比图
        if internal_data and external_data:
            plot_imu_comparison(internal_data, external_data, output_directory)
        
        print(f"\n所有结果已保存到: {output_directory}")
    else:
        print("未能提取到任何数据")

if __name__ == "__main__":
    main()