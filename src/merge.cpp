#include <ros/ros.h>

namespace merge {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);

 private:
  ros::NodeHandle pnh_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
}



}  // namespace merge

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge");
  ros::NodeHandle pnh("~");
  merge::Node node(pnh);
  ros::spin();
  return 0;
}
