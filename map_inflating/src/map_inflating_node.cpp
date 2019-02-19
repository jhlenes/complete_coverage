#include <map_inflating/map_inflating.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "map_inflating_node");

    MapProcessor mapProcessor;

    ros::spin();

    return(0);
}
