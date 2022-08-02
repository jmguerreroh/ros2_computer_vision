/*
# Copyright (c) 2022 José Miguel Guerrero Hernández
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);
cv::Mat image_processing(const cv::Mat in_image);


class ComputerVisionSubscriber : public rclcpp::Node
{
  public:
    ComputerVisionSubscriber()
    : Node("opencv_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/head_front_camera/rgb/image_raw", qos, std::bind(&ComputerVisionSubscriber::topic_callback, this, std::placeholders::_1));
    
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "cv_image", qos);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {     
      // Convert ROS Image to CV Image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_raw =  cv_ptr->image;

      // Image processing
      cv::Mat cv_image = image_processing(image_raw);

      // Convert OpenCV Image to ROS Image
      cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);
      sensor_msgs::msg::Image out_image; // >> message to be sent
      img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

      // Publish the data
      publisher_ -> publish(out_image);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class PCLSubscriber : public rclcpp::Node
{
  public:
    PCLSubscriber()
    : Node("pcl_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

      subscription_3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_front_camera/depth_registered/points", qos, std::bind(&PCLSubscriber::topic_callback_3d, this, std::placeholders::_1));
    
      publisher_3d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_points", qos);
    }

  private:
    void topic_callback_3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {    
     /* // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      pcl::fromROSMsg(*msg, point_cloud);     

      pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud = pcl_processing(point_cloud);
      
      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(pcl_pointcloud, output);
      output.header = msg->header;

      // Publish the data
      publisher_3d_ -> publish(output);*/
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_3d_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3d_;
};


/**
  TO-DO
*/
cv::Mat image_processing(const cv::Mat in_image) 
{
  // Create output image
  cv::Mat out_image = in_image;

  // You must to return a 3-channels image to show it in ROS, so do it with 1-channel images
  //cv::cvtColor(out_image, out_image, cv::COLOR_GRAY2BGR);
  return out_image;
}

/**
  TO-DO
*/
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  out_pointcloud = in_pointcloud;
  return out_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto cv_node = std::make_shared<ComputerVisionSubscriber>();
  auto pcl_node = std::make_shared<PCLSubscriber>();
  exec.add_node(cv_node);
  exec.add_node(pcl_node);
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}