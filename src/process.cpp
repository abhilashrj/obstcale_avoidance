#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <fstream>
#include <vector>

typedef pcl::PointXYZRGB Point;
typedef sensor_msgs::PointCloud2 PCloud2;

ros::Publisher pub_intermediate;

void process_cloud(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
  float min_z=0.0, max_z=5.0, voxel_size=0.012;
  pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>), 
                         ds_cloud(new pcl::PointCloud<Point>),
                         filtered_cloud(new pcl::PointCloud<Point>);
  PCloud2::Ptr intermediate_cloud(new PCloud2);

  pcl::ModelCoefficients::Ptr coefficients_floor(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_floor(new pcl::PointIndices);


  pcl::PassThrough<Point> pass_filter;
  pcl::VoxelGrid<Point> voxel;

  pcl::SACSegmentation<Point> seg;

  fromROSMsg(*input_cloud, *cloud_in);

  // Filter for depth
  pass_filter.setInputCloud(cloud_in);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z, max_z);
  pass_filter.filter(*filtered_cloud);

  

  // Downsampling of the PCL
  voxel.setInputCloud(filtered_cloud);
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setDownsampleAllData(true);
  voxel.filter(*ds_cloud);

  // Segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.005);

  seg.setInputCloud(ds_cloud);
  seg.segment(*inliers_floor, *coefficients_floor);

  if (inliers_floor->indices.size() == 0)
  {
    ROS_ERROR("Couldn't estimate a planar model from dataset!");
    return;
  }

  for (size_t i = 0; i < inliers_floor->indices.size(); ++i)
  {
    ds_cloud->points[inliers_floor->indices[i]].r = 255;
    ds_cloud->points[inliers_floor->indices[i]].g = 0;
    ds_cloud->points[inliers_floor->indices[i]].b = 0;
  }

  toROSMsg(*ds_cloud, *intermediate_cloud);
  intermediate_cloud->header.frame_id = "kinect2_ir_optical_frame";   
  pub_intermediate.publish(intermediate_cloud);
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_node");
  ros::NodeHandle nh;
  ros::Subscriber sub;

  pub_intermediate = nh.advertise<PCloud2>("/intermediate", 1, true);
  sub = nh.subscribe("/kinect2/sd/points", 1, process_cloud);
  while (true)
  {
    ros::spinOnce();
  }
  
  
}