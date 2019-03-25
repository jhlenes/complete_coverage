#ifndef SIMPLE_DUBINS_PATH_H_
#define SIMPLE_DUBINS_PATH_H_

#include <coverage_boustrophedon/DubinInput.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace otter_coverage
{

class SimpleDubinsPath
{
public:
  SimpleDubinsPath();
  SimpleDubinsPath(double turningRadius, double pathResolution);
  ~SimpleDubinsPath();

  bool makePath(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path);

  bool getTargetHeading(double x_q, double y_q, double theta_q, double x_n,
                        double y_n, double& yawTarget);

private:
  enum Dir
  {
    Left = 1,
    Right = -1
  };

  void onGoal(const geometry_msgs::PoseStamped& goal);
  void onInput(const coverage_boustrophedon::DubinInput& input);

  Dir turningDirection(double x_q, double y_q, double theta_q, double x_n,
                       double y_n);
  void turningCenter(double x_q, double y_q, double theta_q, double& x_cr,
                     double& y_cr, Dir dir);
  void tangentLine(double x_n, double y_n, double x_cr, double y_cr,
                   double& beta1, double& beta2);
  void tangentPoint(double x_q, double y_q, double x_n, double y_n, double x_cr,
                    double y_cr, double beta1, double beta2, Dir dir,
                    double& x_lc, double& y_lc);
  void generatePath(double x_q, double y_q, double x_n, double y_n, double x_cr,
                    double y_cr, double x_lc, double y_lc, Dir dir,
                    const geometry_msgs::PoseStamped& goal,
                    nav_msgs::Path& path);
  void generateStraightPath(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            nav_msgs::Path& path);

  double m_turningRadius;
  double m_pathResolution;
  ros::Publisher m_pathPub;
};

} // namespace otter_coverage

#endif
