#include <map_processing/map_processing.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "map_processing_node");

    otter_coverage::MapProcessor mapProcessor;

    ros::spin();

    return(0);
}
