#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Image.h"
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core/core.hpp"
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
////#include <pcl/filters/filter.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl_ros/impl/transforms.hpp>

#include <vector>
#include <iostream>

#include "ImageSubscriberAndPublisher_node/ImageSubscriberAndPublisher.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "imageSubscriberAndPublisher_node");
	ROS_INFO("Start to run imageSubscriberAndPublisher_node with node name %s", ros::this_node::getName().c_str());

	ros::NodeHandle node;
	ros::NodeHandle nodeLocal("~");

	ImageSubscriberAndPublisher imageSubscriberAndPublisher(&node,&nodeLocal);

	ros::spin();

	return 0;

}
