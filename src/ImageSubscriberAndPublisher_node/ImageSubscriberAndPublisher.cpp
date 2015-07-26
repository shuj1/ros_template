#include "ImageSubscriberAndPublisher_node/ImageSubscriberAndPublisher.h"

//Acquire parameters from yaml file
void Parameter::loadParameter(ros::NodeHandle* localNodeHandlePtr)
{
	localNodeHandlePtr->getParam("imageTopicName",imageTopicName);
};


//Class
ImageSubscriberAndPublisher::ImageSubscriberAndPublisher(ros::NodeHandle* nodeHandlePtr,ros::NodeHandle* localNodeHandlePtr){

	nodeHandlePtr_=nodeHandlePtr;
	localNodeHandlePtr_=localNodeHandlePtr;

	//Load the parameters
	params.loadParameter(localNodeHandlePtr);

  //Initialize ROS subscribers
	imageSubscriber_ = nodeHandlePtr_->subscribe("/usb_cam/image_raw", 1, &ImageSubscriberAndPublisher::edgeExtractionCb,this);
	imagePublisher_= nodeHandlePtr_->advertise<sensor_msgs::Image>("edgeImage",1);

};

ImageSubscriberAndPublisher::~ImageSubscriberAndPublisher(){
};

//Call-back function for the subscriber
void ImageSubscriberAndPublisher::edgeExtractionCb(const sensor_msgs::ImagePtr& rosMsg){

	ROS_INFO("Received new image and extracting edges in it.\n");

	//1) Acquire a new image and its pose
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(rosMsg,sensor_msgs::image_encodings::MONO8);

  //2) Extract edges
  cv::Mat edgeImage;
  cv::Canny(cv_ptr->image, edgeImage, 50, 200, 3);

  //3) Publish
	cv_bridge::CvImage edgeImage_msg;
	edgeImage_msg.header.stamp = ros::Time::now();
	//edgeImage_msg.header.frame_id = 1;
	edgeImage_msg.encoding = sensor_msgs::image_encodings::MONO8;
	edgeImage_msg.image    = edgeImage;
	imagePublisher_.publish(edgeImage_msg.toImageMsg());


};
