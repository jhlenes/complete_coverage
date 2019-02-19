#include <simple_dubins_path/simple_dubins_path.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_dubins_path_node");

  otter_coverage::SimpleDubinsPath simpleDubinsPath;

  ros::spin();

  return (0);
}
