#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace otter_coverage {

    class Guidance {
        public:
            Guidance();
            ~Guidance();
        private:
            void newWaypoint(const geometry_msgs::PoseStamped &waypoint);
            void followPath(double x, double y, double psi);

            int currentWp;
            nav_msgs::Path waypoints;

    };

}

#endif
