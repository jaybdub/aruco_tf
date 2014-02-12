// This node detects Aruco markers on a sensor_msgs/image_raw topic, 
// and broadcasts the markers as tf's relative to the camera frame.
//
// The naming scheme used for the markers is /aruco/marker<marker id>
// 

//For cout
#include <iostream>

//For aruco::MarkerDetector, aruco::Marker, and aruco::CameraParameters
#include <aruco/aruco.h>

//For ros node handle/subscribers
#include <ros/ros.h>

//For getting camera info
#include <sensor_msgs/CameraInfo.h>

//For subscribing to the image topic
#include <image_transport/image_transport.h>

//For broadcasting the markers as ros::tf's
#include <tf/transform_broadcaster.h>

//For converting from ROS to OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//For opencv operations (such as draw) and class types
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//For setting the tf name.
//#include <string>

//Globals
static const std::string OPENCV_WINDOW = "Image window";
cv::Size image_size;
cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);;
cv::Mat distortion = cv::Mat::zeros(4, 1, CV_32F);
const float marker_size = 0.088f;

//Function declarations
tf::StampedTransform markerToStampedTransform(aruco::Marker m);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void cameraInfoCb(const sensor_msgs::CameraInfo& msg);

//*** Functions ***

//imageCb is called when an image is received from the /image_raw topic.  
//It detects Aruco markers in an image, broadcasts the markers as TFs
//and draws the markers on the image in an OpenCV window.
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //ArUco object for detecting fiducial ID markers
  aruco::MarkerDetector MDetector;
    
  //Vector to contain detected aruco markers
  vector<aruco::Marker> Markers;
  
  //Detect aruco markers
  MDetector.detect(cv_ptr->image,Markers,camera_matrix,distortion,marker_size);

  //Broadcast each detected marker as a tf, and draw it on the image.
  for (int i=0;i<Markers.size();i++) {
    br.sendTransform(markerToStampedTransform(Markers[i]));
    Markers[i].draw(cv_ptr->image,cv::Scalar(0,0,255),2);  
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
  
};

//Converts an aruco::Marker to a tf::StampedTransform.  It assigns
//a child_tf_name of "marker<id>" and a parent tf of "world"
tf::StampedTransform markerToStampedTransform(aruco::Marker m) {
  tf::Transform transform;
  // Convert and set the translation
  float dx = m.Tvec.ptr<float>(0)[0];
  float dy = m.Tvec.ptr<float>(0)[1];
  float dz = m.Tvec.ptr<float>(0)[2];
  transform.setOrigin( tf::Vector3(dx,dy,dz) );
  
  // Convert and set the rotation.
  float rx = m.Rvec.ptr<float>(0)[0];
  float ry = m.Rvec.ptr<float>(0)[1];
  float rz = m.Rvec.ptr<float>(0)[2];
  float angle_magnitude = sqrt(rx*rx+ry*ry+rz*rz);
  float kx = rx/angle_magnitude;
  float ky = ry/angle_magnitude;
  float kz = rz/angle_magnitude;
  transform.setRotation( tf::Quaternion(tf::Vector3(kx, ky, kz), angle_magnitude) );
    
  // Name and timestamp
  char tf_name[10] = "";
  sprintf(tf_name,"marker%d",m.id);
  return tf::StampedTransform(transform, ros::Time::now(), "world", tf_name);
}

// cameraInfoCb takes a sensor_msgs::CameraInfo message, and 
// converts the calibration data stored in the message into 
// a format that the ArUco algorithm can use.  It operates
// on the global variables 'image_size', 'camera_matrix', and 'distortion'
void cameraInfoCb(const sensor_msgs::CameraInfo& msg) {
  //Convert ROS Camera Info msg to format used by Aruco
  // see http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // for ROS types and & aruco's cameraparameters.h constructors for aruco types

  //Update the image size
  image_size.width = msg.width;
  image_size.height = msg.height;
  cout << "\nWidth: " << image_size.width;
  cout << "\nHeight: " << image_size.height;
  
  //Convert the intrinsic camera parameters in the ROS msg into a 3x3 cv::Mat.
  //In the ROS message the parameters are stored in a row-major float array.
  for(int row=0;row<3;row++){
    for(int col=0;col<3;col++){
      camera_matrix.at<float>(row,col) = msg.K[row*3+col];
    }
  }
  //cout << camera_matrix;
  //Convert the distortion parameters to a 4x1 cv:Mat, if the msg.D exists
  //and is at least 4 units long.
  if(msg.D.size() >= 4){
    for(int i=0;i<4;i++){
      distortion.at<float>(i,0) = msg.D.at(i);
    } 
  }
  cout << distortion;
}

int main(int argc, char** argv) {

  //ROS node handle
  ros::init(argc, argv, "aruco_tf_broadcaster");
  ros::NodeHandle nh;

  //Camera Info Subscriber (used for calibration)
  ros::Subscriber cam_info_sub = nh.subscribe("camera_info", 100, &cameraInfoCb);

  //Image subscriber (used to get raw image data)
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  cv::namedWindow(OPENCV_WINDOW);
  image_sub = it.subscribe("image_raw", 1, &imageCb);
  ros::spin();
  return 0;
}
