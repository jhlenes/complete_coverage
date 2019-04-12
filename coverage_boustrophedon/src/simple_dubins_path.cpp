#include <nav_msgs/Path.h>
#include <simple_dubins_path/simple_dubins_path.h>
#include <tf/tf.h>

#include <cmath>

namespace otter_coverage
{

const double epsilon = std::numeric_limits<double>::epsilon();

SimpleDubinsPath::SimpleDubinsPath()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  m_turningRadius = private_nh.param("turning_radius", 1.0);
  m_pathResolution = private_nh.param("path_resolution", 0.05);

  ros::Subscriber inputSub = nh.subscribe("simple_dubins_path/input", 1000,
                                          &SimpleDubinsPath::onInput, this);

  ros::Subscriber sub = nh.subscribe("simple_dubins_path/goal", 1000,
                                     &SimpleDubinsPath::onGoal, this);

  m_pathPub = nh.advertise<nav_msgs::Path>("simple_dubins_path", 1000);

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

SimpleDubinsPath::SimpleDubinsPath(double turningRadius, double pathResolution)
    : m_turningRadius(turningRadius), m_pathResolution(pathResolution)
{
}

SimpleDubinsPath::~SimpleDubinsPath() {}

void SimpleDubinsPath::onGoal(const geometry_msgs::PoseStamped& goal)
{
  ROS_ERROR("simple_dubins_path/goal is disabled.");
  return;
  static geometry_msgs::PoseStamped start;
  static geometry_msgs::PoseStamped end;
  static int counter = 0;

  ++counter;
  if (counter == 1)
  {
    ROS_INFO("Received start pose");
    start = goal;
  }
  else if (counter == 2)
  {
    ROS_INFO("Received end pose");
    end = goal;

    nav_msgs::Path path;
    makePath(start, end, path);
    m_pathPub.publish(path);

    counter = 0;
  }
}

void SimpleDubinsPath::onInput(const coverage_boustrophedon::DubinInput& input)
{
  nav_msgs::Path path;
  makePath(input.start, input.end, path);
  m_pathPub.publish(path);
}

SimpleDubinsPath::Dir SimpleDubinsPath::turningDirection(double x_q, double y_q,
                                                         double theta_q,
                                                         double x_n, double y_n)
// Note: Prefers right turns when doing 180 turn (or going straight)
{
  Dir turningDirection = Right;

  // Rotate x-axis theta_q, and check if the new y-coordinate is on the left
  // (positive)
  if (-(x_n - x_q) * sin(theta_q) + (y_n - y_q) * cos(theta_q) > 0)
  {
    turningDirection = Left;
  }
  return turningDirection;
}

void SimpleDubinsPath::turningCenter(double x_q, double y_q, double theta_q,
                                     double& x_cr, double& y_cr, Dir dir)
// Note: For compliance with previous assumption,
// must prefer right turns when doing 180 turn (or going straight)
{
  double x_cr1 = x_q + sin(theta_q) * m_turningRadius;
  double y_cr1 = y_q - cos(theta_q) * m_turningRadius;
  double x_cr2 = x_q - sin(theta_q) * m_turningRadius;
  double y_cr2 = y_q + cos(theta_q) * m_turningRadius;

  // Rotate x-axis theta_q, and check if the new y-coordinate of cr1 is on the
  // left (positive)
  if (-(x_cr1 - x_q) * sin(theta_q) + (y_cr1 - y_q) * cos(theta_q) > 0)
  // cr1 is left turning point
  {
    if (dir == Left)
    {
      x_cr = x_cr1;
      y_cr = y_cr1;
    }
    else // dir == Right
    {
      x_cr = x_cr2;
      y_cr = y_cr2;
    }
  }
  else
  // cr1 is right turning point
  {
    if (dir == Right)
    {
      x_cr = x_cr1;
      y_cr = y_cr1;
    }
    else // dir == Left
    {
      x_cr = x_cr2;
      y_cr = y_cr2;
    }
  }
}

void SimpleDubinsPath::tangentLine(double x_n, double y_n, double x_cr,
                                   double y_cr, double& beta1, double& beta2)
{
  double a = (x_cr - x_n);
  double b = (y_n - y_cr);
  beta1 = 0;
  beta2 = 0;
  if (std::abs(b + m_turningRadius) < epsilon)
  {
    beta1 = 2 * atan((a - std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b - m_turningRadius));
    beta2 = 2 * atan((a + std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b - m_turningRadius));
  }
  else
  {
    beta1 = 2 * atan((a + std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b + m_turningRadius));
    beta2 = 2 * atan((a - std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b + m_turningRadius));
  }
  // force beta in [0, pi)
  if (beta1 < 0)
  {
    beta1 = beta1 + M_PI;
  }
  if (beta2 < 0)
  {
    beta2 = beta2 + M_PI;
  }
}

void SimpleDubinsPath::tangentPoint(double x_q, double y_q, double x_n,
                                    double y_n, double x_cr, double y_cr,
                                    double beta1, double beta2, Dir dir,
                                    double& x_lc, double& y_lc)
{
  // Circle-line intersection with circle in origin
  // http://mathworld.wolfram.com/Circle-LineIntersection.html
  double x2 = x_n - x_cr; // move line to origin
  double y2 = y_n - y_cr; // move line to origin

  // (x_lc1, y_lc1)
  double x1 = (x_n + cos(beta1)) - x_cr; // move line to origin
  double y1 = (y_n + sin(beta1)) - y_cr; // move line to origin
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr = std::sqrt(dx * dx + dy * dy);
  double D = x1 * y2 - x2 * y1;

  double x_lc1 = D * dy / (dr * dr);
  double y_lc1 = -D * dx / (dr * dr);
  x_lc1 = x_lc1 + x_cr; // move back from origin
  y_lc1 = y_lc1 + y_cr; // move back from origin

  // (x_lc2, y_lc2)
  x1 = (x_n + cos(beta2)) - x_cr; // move line to origin
  y1 = (y_n + sin(beta2)) - y_cr; // move line to origin
  dx = x2 - x1;
  dy = y2 - y1;
  dr = std::sqrt(dx * dx + dy * dy);
  D = x1 * y2 - x2 * y1;

  double x_lc2 = D * dy / (dr * dr);
  double y_lc2 = -D * dx / (dr * dr);
  x_lc2 = x_lc2 + x_cr; // move back from origin
  y_lc2 = y_lc2 + y_cr; // move back from origin

  // Find the first tangent point encountered along the direction of rotation
  double v_head[2] = {x_q - x_cr, y_q - y_cr};
  double v_lc1[2] = {x_lc1 - x_cr, y_lc1 - y_cr};
  double v_lc2[2] = {x_lc2 - x_cr, y_lc2 - y_cr};

  x1 = v_head[0];
  y1 = v_head[1];
  x2 = v_lc1[0];
  y2 = v_lc1[1];
  double dot = x1 * x2 + y1 * y2; // dot product
  double det = x1 * y2 - y1 * x2; // determinant
  double angle1 = std::atan2(det, dot);
  if (angle1 < 0)
    angle1 += 2 * M_PI; // wrap to [0, 2*pi]

  x2 = v_lc2[0];
  y2 = v_lc2[1];
  dot = x1 * x2 + y1 * y2; // dot product
  det = x1 * y2 - y1 * x2; // determinant
  double angle2 = std::atan2(det, dot);
  if (angle2 < 0)
    angle2 += 2 * M_PI; // wrap to [0, 2*pi]

  x_lc = x_lc2;
  y_lc = y_lc2;
  double angle = angle2;
  if (dir == Left)
  {
    if (angle1 < angle2)
    {
      x_lc = x_lc1;
      y_lc = y_lc1;
      angle = angle1;
    }
  }
  else if (dir == Right)
  {
    if (angle1 > angle2 || abs(angle1) < epsilon) // angle == 0 is best
    {
      x_lc = x_lc1;
      y_lc = y_lc1;
      angle = angle1;
    }
  }
}

void SimpleDubinsPath::generatePath(double x_q, double y_q, double x_n,
                                    double y_n, double x_cr, double y_cr,
                                    double x_lc, double y_lc, Dir dir,
                                    const geometry_msgs::PoseStamped& goal,
                                    nav_msgs::Path& path)
{
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses.clear();

  // find circle segment to follow
  double startAngle = std::atan2(y_q - y_cr, x_q - x_cr);
  if (startAngle < 0)
    startAngle += 2 * M_PI; // wrap to [0, 2*pi]
  double stopAngle = std::atan2(y_lc - y_cr, x_lc - x_cr);
  if (stopAngle < 0)
    stopAngle += 2 * M_PI; // wrap to [0, 2*pi]

  if (dir == Left && stopAngle < startAngle)
  {
    stopAngle += 2 * M_PI;
  }
  else if (dir == Right && stopAngle > startAngle)
  {
    stopAngle -= 2 * M_PI;
  }

  // generate circle segment
  double angleIncrement = m_pathResolution / m_turningRadius;
  for (double i = startAngle; std::abs(i - stopAngle) > 2 * angleIncrement;
       i += dir * angleIncrement)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = x_cr + cos(i) * m_turningRadius;
    point.pose.position.y = y_cr + sin(i) * m_turningRadius;
    tf2::Quaternion q;
    q.setRPY(0, 0, i + ((dir == Right) ? -M_PI_2 : M_PI_2));
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    path.poses.push_back(point);
  }

  double dx = x_n - x_lc;
  double dy = y_n - y_lc;
  double dx_norm = dx / std::sqrt(dx * dx + dy * dy);
  double dy_norm = dy / std::sqrt(dx * dx + dy * dy);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(y_n - y_lc, x_n - x_lc));

  // generate straight line segment
  for (double i = 0;
       std::fabs(i * m_pathResolution * dx_norm - dx) > 2 * m_pathResolution ||
       std::fabs(i * m_pathResolution * dy_norm - dy) > 2 * m_pathResolution;
       ++i)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = x_lc + i * m_pathResolution * dx_norm;
    point.pose.position.y = y_lc + i * m_pathResolution * dy_norm;
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    path.poses.push_back(point);
  }

  path.poses.push_back(goal);
}

void SimpleDubinsPath::generateStraightPath(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path)
{
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses.clear();

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double dx_norm = dx / std::sqrt(dx * dx + dy * dy);
  double dy_norm = dy / std::sqrt(dx * dx + dy * dy);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(dy, dx));

  // generate straight line segment
  for (double i = 0;
       std::fabs(i * m_pathResolution * dx_norm - dx) > 2 * m_pathResolution ||
       std::fabs(i * m_pathResolution * dy_norm - dy) > 2 * m_pathResolution;
       ++i)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = start.pose.position.x + i * m_pathResolution * dx_norm;
    point.pose.position.y = start.pose.position.y + i * m_pathResolution * dy_norm;
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    path.poses.push_back(point);
  }
}

bool SimpleDubinsPath::makePath(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                nav_msgs::Path& path)
{
  // Initial configuration
  double x_q = start.pose.position.x;
  double y_q = start.pose.position.y;
  double theta_q = tf::getYaw(start.pose.orientation);

  // Target position
  double x_n = goal.pose.position.x;
  double y_n = goal.pose.position.y;

  Dir dir = turningDirection(x_q, y_q, theta_q, x_n, y_n);

  // Find the center of the turning circle
  double x_cr;
  double y_cr;
  turningCenter(x_q, y_q, theta_q, x_cr, y_cr, dir);

  // Perform check
  if (std::sqrt(std::pow(x_n - x_cr, 2) + std::pow(y_n - y_cr, 2)) <
      m_turningRadius)
  {
    //ROS_WARN("Target not reachable with simple Dubin's path. Generating "
    //         "straight line instead.");
    generateStraightPath(start, goal, path);
    return true;
  }

  // Find angle of tangent line from target to turning circle
  double beta1;
  double beta2;
  tangentLine(x_n, y_n, x_cr, y_cr, beta1, beta2);

  // Find tangent point
  double x_lc;
  double y_lc;
  tangentPoint(x_q, y_q, x_n, y_n, x_cr, y_cr, beta1, beta2, dir, x_lc, y_lc);

  // Generate path
  generatePath(x_q, y_q, x_n, y_n, x_cr, y_cr, x_lc, y_lc, dir, goal, path);

  return true;
}

bool SimpleDubinsPath::getTargetHeading(double x_q, double y_q, double theta_q,
                                        double x_n, double y_n,
                                        double& yawTarget)
{
  Dir dir = turningDirection(x_q, y_q, theta_q, x_n, y_n);

  // Find the center of the turning circle
  double x_cr, y_cr;
  turningCenter(x_q, y_q, theta_q, x_cr, y_cr, dir);

  // Is target reachable?
  if (std::sqrt(std::pow(x_n - x_cr, 2) + std::pow(y_n - y_cr, 2)) <
      m_turningRadius)
  {
    return false;
  }

  // Find angle of tangent line from target to turning circle
  double beta1, beta2;
  tangentLine(x_n, y_n, x_cr, y_cr, beta1, beta2);

  // Find tangent point
  double x_lc, y_lc;
  tangentPoint(x_q, y_q, x_n, y_n, x_cr, y_cr, beta1, beta2, dir, x_lc, y_lc);

  yawTarget = std::atan2(y_n - y_lc, x_n - x_lc);
  return true;
}

} // namespace otter_coverage
