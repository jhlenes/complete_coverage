#include <coverage_binn/coverage_binn.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "coverage_binn");

  ros::NodeHandle n;

  CoverageBinn coverageBinnNode;

  ros::spin();

  return 0;
}
