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
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  ros::Subscriber sub = pnh_.subscribe("project", 10, &Node::receivePointCloud, this);
  ros::ServiceServer service = pnh_.advertiseService("finish", &Node::performMerge, this);
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
  
  printf("test\n"); 
  fflush(stdout);
ROS_WARN("JKALDSFJKJFASFJSA");
  ros::init(argc, argv, "merge");

  ros::NodeHandle pnh("~");
  merge::Node node(pnh);
  printf("test"); 
  ROS_INFO("started merge node");
  ros::spin();
  return 0;
}
