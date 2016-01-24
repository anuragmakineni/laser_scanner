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
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

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
  ros::Subscriber sub_top;
  ros::Subscriber sub_side;
  ros::ServiceServer service;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  sub_top = pnh_.subscribe("project_top", 10, &Node::receiveTopPointCloud, this);
  sub_side = pnh_.subscribe("project_side", 10, &Node::receiveSidePointCloud, this);
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
  icp.setInputSource(sideCloud.makeShared());
  icp.setInputTarget(topCloud.makeShared());
  icp.align(resultCloud);
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(resultCloud, cloud2);
  mergedPublish.publish(cloud2);

  //save as PLY
  pcl::io::savePLYFileASCII("out.ply",resultCloud);

  return true;


  // // STL Export
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&resultCloud);
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // tree->setInputCloud (cloud);
  // n.setInputCloud (cloud);
  // n.setSearchMethod (tree);
  // n.setKSearch (20);
  // n.compute (*normals);
  // //* normals should not contain the point normals + surface curvatures

  // // Concatenate the XYZ and normal fields*
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  // pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  // //* cloud_with_normals = cloud + normals

  // // Create search tree*
  // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  // tree2->setInputCloud (cloud_with_normals);

  // // Initialize objects
  // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  // pcl::PolygonMesh triangles;

  // // Set the maximum distance between connected points (maximum edge length)
  // gp3.setSearchRadius (0.025);

  // // Set typical values for the parameters
  // gp3.setMu (2.5);
  // gp3.setMaximumNearestNeighbors (100);
  // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  // gp3.setMinimumAngle(M_PI/18); // 10 degrees
  // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  // gp3.setNormalConsistency(false);

  // // Get result
  // gp3.setInputCloud (cloud_with_normals);
  // gp3.setSearchMethod (tree2);
  // gp3.reconstruct (triangles);
  // pcl::io::savePolygonFileSTL("out.stl", triangles);
  // return true;
}

}  // namespace merge


int main(int argc, char** argv) {

  ros::init(argc, argv, "merge");
  ros::NodeHandle pnh("~");
  merge::Node node(pnh);
  ros::spin();
  return 0;
}
