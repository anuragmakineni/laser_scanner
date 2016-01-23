#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

using namespace std;

namespace merge {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void receivePointCloud(const std_msgs::String::ConstPtr& msg);
  bool performMerge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res);

 private:
  ros::NodeHandle pnh_;
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
  ros::Subscriber sub;
  ros::ServiceServer service;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  sub = pnh_.subscribe("project", 10, &Node::receivePointCloud, this);
  service = pnh_.advertiseService("finish", &Node::performMerge, this);
  ROS_INFO("init");
}

void Node::receivePointCloud(const std_msgs::String::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

}

bool Node::performMerge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("sdkjf");
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
