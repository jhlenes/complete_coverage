#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace otter_coverage {

    class Guidance {
        public:
            Guidance();
            ~Guidance();
        private:
            nav_msgs::Path path;


    };

}

#endif
