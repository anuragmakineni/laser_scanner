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

 private:
  ros::NodeHandle pnh_;
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
}

}  // namespace merge

void receivePointCloud(const std_msgs::String::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  
}

bool performMerge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("sdkjf");
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge");
  ros::NodeHandle pnh("~");
  merge::Node node(pnh);
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("project", 1000, receivePointCloud);
  ros::ServiceServer service = pnh.advertiseService("finish", performMerge);
  ros::spin();
  return 0;
}
