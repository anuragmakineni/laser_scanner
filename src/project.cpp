#include <ros/ros.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

namespace project {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void laser_1_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void laser_2_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber laser_1_sub;
  ros::Subscriber laser_2_sub;
  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  laser_1_sub = pnh_.subscribe("laser/scan1", 10, &Node::laser_1_cb, this);
  laser_2_sub = pnh_.subscribe("laser/scan2", 10, &Node::laser_2_cb, this);
  ROS_INFO("init");
}

void Node::laser_1_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  ROS_INFO("laser 1 cb");

  if(!listener_.waitForTransform("/laser1", "/world", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/world", *scan_in, cloud, listener_);

}

void Node::laser_2_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform("/laser2", "/world", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/world", *scan_in, cloud, listener_);

  ROS_INFO("laser 2 cb");
}


}  // namespace project


int main(int argc, char** argv) {

  ros::init(argc, argv, "project");
  ros::NodeHandle pnh("~");
  project::Node node(pnh);
  ros::spin();
  return 0;
}
