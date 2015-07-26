#pragma once

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

//#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
////#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
//#include <pcl/common/conversions.h>
//#include <pcl/common/common_headers.h>
//#include <pcl/point_traits.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>
//#include <pcl/visualization/point_cloud_color_handlers.h>


//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl_ros/impl/transforms.hpp>

#include "ImageSubscriberAndPublisher_node/ImageSubscriberAndPublisher.h"


struct Parameter{
  //Parameters
	std::string imageTopicName;

	//Function to load the parameters
	void loadParameter(ros::NodeHandle* localNodeHandlePtr);

	//Adapt the structure such that it will work with new-operator
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




class ImageSubscriberAndPublisher{
private:
	ros::NodeHandle* nodeHandlePtr_;         //pointer to the node handle
	ros::NodeHandle* localNodeHandlePtr_;    //pointer to the local node handle
  Parameter params;                        //struct of the needed parameters

  ros::Subscriber imageSubscriber_;
  ros::Publisher imagePublisher_;

public:
	ImageSubscriberAndPublisher(ros::NodeHandle* nodeHandlePtr,ros::NodeHandle* localNodeHandlePtr);
	~ImageSubscriberAndPublisher();

	//Callback-Functions:
	void edgeExtractionCb(const sensor_msgs::ImagePtr& rosMsg);


  //Adapt the structure such that it will work with new-operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
