#include <guidance/guidance.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

#include <usv_msgs/SpeedCourse.h>

namespace otter_coverage
{

Guidance::Guidance()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  m_maxSpeed = nhP.param("max_speed", 1.5);
  m_maxSpeedTurn = nhP.param("max_speed_turn", 0.6);
  m_minSpeed = nhP.param("min_speed", 0.4);

  ros::Subscriber dubinsPathSub =
      nh.subscribe("simple_dubins_path", 1000, &Guidance::newPath, this);

  m_controllerPub =
      nh.advertise<usv_msgs::SpeedCourse>("speed_heading", 1000);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // Get the pose of the robot in the map frame
    geometry_msgs::TransformStamped tfStamped;
    try
    {
      tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                           ros::Duration(0.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Transform from map to base_link not found: %s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    double x = tfStamped.transform.translation.x;
    double y = tfStamped.transform.translation.y;
    double psi = tf2::getYaw(tfStamped.transform.rotation);

    followPath(x, y, psi);

    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newPath(const nav_msgs::Path& path) { m_path = path; }

void Guidance::followPath(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{

  // Finished?
  if (m_path.poses.size() <= 1)
  {
    usv_msgs::SpeedCourse msg;
    msg.speed = 0.0;
    msg.course = psi;
    m_controllerPub.publish(msg);
    return;
  }

  // Identify closest point on path
  std::vector<geometry_msgs::PoseStamped>::iterator closest;
  double minDist = std::numeric_limits<double>::max();
  for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
    {
      minDist = dist;
      closest = it;
    }
  }

  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  m_path.poses.erase(m_path.poses.begin(), closest);

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e =
      (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
      delta_min;
  // if turning => small lookahead distance
  bool isTurning = false;
  if ((closest + 1) != m_path.poses.end())
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
      isTurning = true;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate error in heading
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)
    u = m_maxSpeedTurn;

  // Publish speed and course to controller
  usv_msgs::SpeedCourse msg;
  msg.speed = u;
  msg.course = chi_d;
  m_controllerPub.publish(msg);

  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);

}

} // namespace otter_coverage
