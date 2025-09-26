#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 全局发布者
ros::Publisher pub_custom_msg;

// 回调函数，将PointCloud2转换为CustomMsg
void pcl2ToCustomMsgCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS点云消息转换为PCL格式 (这里使用带强度的点)
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    livox_ros_driver::CustomMsg custom_msg;

    // 设置CustomMsg头部信息
    custom_msg.point_num = pcl_cloud.points.size();
    custom_msg.timebase = cloud_msg->header.stamp.toNSec();
    custom_msg.header = cloud_msg->header;

    // 逐点转换
    custom_msg.points.reserve(pcl_cloud.points.size());
    for (const auto& pt : pcl_cloud.points) {
        livox_ros_driver::CustomPoint cp;
        cp.x = pt.x;
        cp.y = pt.y;
        cp.z = pt.z;
        cp.reflectivity = pt.intensity;  // 用intensity映射reflectivity
        cp.line = 0;                     // 如果没有线号信息，设置为0，或根据需要修改
        cp.offset_time = 0;              // 时间偏移无具体来源，先填0，后续可改进
        custom_msg.points.push_back(cp);
    }

    // 发布CustomMsg
    pub_custom_msg.publish(custom_msg);

    ROS_INFO_STREAM("Converted PointCloud2 (" << pcl_cloud.points.size() << " pts) to CustomMsg");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl2_to_custommsg_node");
    ros::NodeHandle nh;

    // 订阅PointCloud2点云话题（根据你的bag包或系统调整topic名称）
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, pcl2ToCustomMsgCallback);

    // 发布转换后的CustomMsg话题
    pub_custom_msg = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar_custommsg", 10);

    ROS_INFO("pcl2_to_custommsg_node started");

    ros::spin();
    return 0;
}

