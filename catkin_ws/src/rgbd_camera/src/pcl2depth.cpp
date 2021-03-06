/**********************************
Author: David Chen
Date: 2019/03/12 
Last update: 2019/03/12
Point Cloud to Depth Image
Subscribe: 
  /camera/depth_registered/points      (sensor_msgs/PointCloud2)
  /X1/rgbd_camera/depth/points
  /X1/points
Publish:
  /obstacle_marker      (visualization_msgs/MarkerArray)
  /obstacle_marker_line (visualization_msgs/MarkerArray)
  /cluster_result       (sensor_msgs/PointCloud2)
***********************************/ 
#include <iostream>
#include <vector>
#include <array>
#include <time.h>
#include <string>
#include <math.h>
//Ros Lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
//TF lib
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

//OpenCV lib
#include <opencv/cv.h>

#define PI 3.14159
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
bool is_LIDAR = true;

class PCL2Depth{
private:
  string node_name;

  // For point cloud declare variables
  string frame_id;
  tf::StampedTransform transformStamped;
  Eigen::Affine3d transform_eigen;

  // Image
  sensor_msgs::ImagePtr img_msg;
  cv_bridge::CvImage cvIMG;

  // Counter
  int counts;

  // for ROS
  ros::NodeHandle nh;
  ros::Subscriber sub_cloud;
  ros::Publisher  pub_cloud;
  ros::Publisher  pub_image;

  // Camera information
  float fx = 614.4776611328125;
  float fy = 614.2581176757812;
  float cx = 320.2654724121094;
  float cy = 249.4024658203125;

public:
  PCL2Depth(ros::NodeHandle&);
  void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);
  void get_img_coordinate(float*, float*, float);
  cv::Mat pointcloud_to_image(const PointCloudXYZRGB::Ptr, PointCloudXYZRGB::Ptr, cv::Mat);
  void pcl_preprocess(PointCloudXYZRGB::Ptr);
};

PCL2Depth::PCL2Depth(ros::NodeHandle &n){
  nh = n;
  counts = 0;
  node_name = ros::this_node::getName();


  // Publisher
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/ppp", 1);
  pub_image = nh.advertise<sensor_msgs::Image> ("/dp_img", 1);

  // Subscriber
  sub_cloud = nh.subscribe("/X1/points", 1, &PCL2Depth::cbCloud, this);
}

void PCL2Depth::get_img_coordinate(float* x, float* y,float z){
	*x = (*x * fx)/z + cx;
	*y = (*y * fy)/z + cy;
	return;
}

void PCL2Depth::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  frame_id = cloud_msg->header.frame_id;
  counts++;
  //return if no cloud data
  if ((cloud_msg->width * cloud_msg->height) == 0 || counts % 3 == 0)
  {
    counts = 0;
    return ;
  }
  const clock_t t_start = clock();
  
  // transfer ros msg to point cloud
  PointCloudXYZ::Ptr cloud_XYZ(new PointCloudXYZ);
  PointCloudXYZRGB::Ptr cloud_XYZRGB(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_in(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);
  if(is_LIDAR){
  	pcl::fromROSMsg (*cloud_msg, *cloud_XYZ);
  	copyPointCloud(*cloud_XYZ, *cloud_in);
  }
  else{
  	pcl::fromROSMsg (*cloud_msg, *cloud_XYZRGB);
  	copyPointCloud(*cloud_XYZRGB, *cloud_in);
  }
  copyPointCloud(*cloud_in, *cloud_out);

  // Remove out of range points and robot points
  //pcl_preprocess(cloud_out);

  cv::Mat depth_image(480, 640, CV_16UC1, cv::Scalar(0, 0, 0));
  cv::Mat result = pointcloud_to_image(cloud_in, cloud_out, depth_image);
  clock_t t_end = clock();
  //cout << "PointCloud preprocess time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;

  // PUblish image
  //std::cout << result.cols << ',' << result.rows << std::endl;
  cvIMG.header = cloud_msg->header;
  cvIMG.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  cvIMG.image = result;
  //img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_image).toImageMsg();
  pub_image.publish(cvIMG.toImageMsg());

  // Publish point cloud
  sensor_msgs::PointCloud2 pcl_output;
  pcl::toROSMsg(*cloud_out, pcl_output);
  pcl_output.header = cloud_msg->header;
  //pcl_output.header.frame_id = source_frame;
  pub_cloud.publish(pcl_output);
}

cv::Mat PCL2Depth::pointcloud_to_image(const PointCloudXYZRGB::Ptr cloud_in, PointCloudXYZRGB::Ptr cloud_out, cv::Mat depth_image){
  //depth_image.setTo(cv::Scalar(0, 0, 0));
  
  for(int i = 0; i < cloud_in->points.size(); i++){
  	float* x;
  	float* y;
  	float z;

  	if(is_LIDAR){
  	  x = new float(cloud_in->points[i].y);
  	  y = new float(cloud_in->points[i].z);
  	  z = cloud_in->points[i].x;
  	  if(z < 0){
  	  	continue;
  	  }
  	  get_img_coordinate(x, y, z);
  	  if (int(*x) < 640 && int(*y) < 480 && int(*x) >= 0 && int(*y) >= 0){
  	  	cloud_out->points[i].r = 255;
    	cloud_out->points[i].g = 255;
    	cloud_out->points[i].b = 0;
  		depth_image.at<unsigned short int>(int(*y), int(*x)) = z;
  	  }
	}

	else{
	  x = new float(cloud_in->points[i].x);
  	  y = new float(cloud_in->points[i].y);
  	  z = cloud_in->points[i].z;
  	  /*if (std::isnan(*x) || std::isnan(*y) ||std::isnan(z)){
  	  	std::cout << *x << ',' << *y << "," << z << std::endl;
  	  }*/
  	  if(z < 0){
  	  	continue;
  	  }
  	  get_img_coordinate(x, y, z);
  	  if (int(*x) < 640 && int(*y) < 480 && int(*x) >= 0 && int(*y) >= 0){
  		depth_image.at<unsigned short int>(int(*y), int(*x)) = z*1000.;
  	  }
	}
  	
  	free(x);
  	free(y);
  }
  return depth_image;
}

void PCL2Depth::pcl_preprocess(PointCloudXYZRGB::Ptr cloud_out){
  int num = 0;
  for (int i=0 ; i <  cloud_out->points.size() ; i++)
  {
    if (cloud_out->points[i].x > 0){
    	cloud_out->points[i].r = 255;
    	cloud_out->points[i].g = 255;
    	cloud_out->points[i].b = 0;
    }
  }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "PCL2Depth");
  ros::NodeHandle nh("~");
  PCL2Depth pn(nh);
  ros::spin ();
  return 0;
}