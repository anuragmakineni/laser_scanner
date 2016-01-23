#include <ros/ros.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

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
  ros::Publisher pc_1_pub_;
  ros::Publisher pc_2_pub_;

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  laser_1_sub = pnh_.subscribe("laser/scan1", 10, &Node::laser_1_cb, this);
  laser_2_sub = pnh_.subscribe("laser/scan2", 10, &Node::laser_2_cb, this);

  pc_1_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("project_side", 10);
  pc_2_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("project_top", 10);


  ROS_INFO("init");
}

void Node::laser_1_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

  if(!listener_.waitForTransform("/laser1", "/world", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/world", *scan_in, cloud, listener_);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);

  cloud1 += pcl_cloud;

  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(cloud1, cloud_out);
  cloud_out.header = cloud.header;

  pc_1_pub_.publish(cloud_out);
}

void Node::laser_2_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform("/laser2", "/world", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/world", *scan_in, cloud, listener_);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);

  cloud2 += pcl_cloud;

  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(cloud2, cloud_out);
  cloud_out.header = cloud.header;

  pc_2_pub_.publish(cloud_out);
}


}  // namespace project


int main(int argc, char** argv) {

  ros::init(argc, argv, "project");
  ros::NodeHandle pnh("~");
  project::Node node(pnh);
  ros::spin();
  return 0;
}
