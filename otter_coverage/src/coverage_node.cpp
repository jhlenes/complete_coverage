#include <coverage/coverage.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "coverage_node");

  otter_coverage::Coverage coverage;

  ros::spin();

  return(0);
}
