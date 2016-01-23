#include <ros/ros.h>
#include <string>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>

using namespace std;
using namespace pcl::search;

namespace merge {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void receiveTopPointCloud(const sensor_msgs::PointCloud2& pc);
  void receiveSidePointCloud(const sensor_msgs::PointCloud2& pc);
  bool performMerge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res);

 private:
  ros::Publisher mergedPublish;
  pcl::PointCloud<pcl::PointXYZ> sideCloud;
  pcl::PointCloud<pcl::PointXYZ> topCloud;
  pcl::PointCloud<pcl::PointXYZ> resultCloud;
  ros::NodeHandle pnh_;
  ros::Subscriber sub;
  ros::ServiceServer service;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  sub = pnh_.subscribe("project_top", 10, &Node::receiveTopPointCloud, this);
  sub = pnh_.subscribe("project_side", 10, &Node::receiveSidePointCloud, this);
  service = pnh_.advertiseService("finish", &Node::performMerge, this);
  mergedPublish = pnh_.advertise<sensor_msgs::PointCloud2>("merged", 10);
  ROS_INFO("init");
}

void Node::receiveTopPointCloud(const sensor_msgs::PointCloud2& pc)
{
  pcl::fromROSMsg(pc, topCloud);
}

void Node::receiveSidePointCloud(const sensor_msgs::PointCloud2& pc)
{
  pcl::fromROSMsg(pc, sideCloud);
}

bool Node::performMerge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;   
  icp.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(&sideCloud));
  icp.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(&topCloud));
  icp.align(resultCloud);
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(resultCloud, cloud);
  mergedPublish.publish(cloud);
  return true;
}

}  // namespace merge


int main(int argc, char** argv) {

  ros::init(argc, argv, "merge");
  ros::NodeHandle pnh("~");
  merge::Node node(pnh);
  ros::spin();
  return 0;
}
