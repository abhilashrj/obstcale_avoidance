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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <fstream>
#include <vector>

typedef pcl::PointXYZRGB Point;
typedef sensor_msgs::PointCloud2 PCloud2;

ros::Publisher pub_intermediate;
float min_z, max_z, voxel_size,seg_dist_tr, plane_perc;

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}
class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

std::vector<Color> colors;






void createColors(){
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        for (int i=0;i<20;i++){
            while (r<70 && g < 70 && b < 70){
                r = rand()%(255);
                g = rand()%(255);
                b = rand()%(255);
            }
            Color c(r,g,b);
            r = 0;
            g = 0;
            b = 0;
            colors.push_back(c);
        }
    }





void process_cloud(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
  
 

  pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>), 
                         ds_cloud(new pcl::PointCloud<Point>),
                         filtered_cloud(new pcl::PointCloud<Point>),
                          cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<Point> non_plane_cloud;

  PCloud2::Ptr intermediate_cloud(new PCloud2);

  pcl::ModelCoefficients::Ptr coefficients_floor(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_floor(new pcl::PointIndices);

  pcl::PassThrough<Point> pass_filter;
  pcl::VoxelGrid<Point> voxel;

  pcl::SACSegmentation<Point> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<Point> extract;


  fromROSMsg(*input_cloud, *cloud_in);
  std::cout << min_z << max_z << voxel_size << seg_dist_tr;

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

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(seg_dist_tr);
  seg.setMaxIterations(100);

 // New segmentation
    int original_size = ds_cloud->height*ds_cloud->width;
    int n_planes = 0;
    while (ds_cloud->height*ds_cloud->width > original_size*plane_perc){

        // Fit a plane
        seg.setInputCloud(ds_cloud);
        seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size() == 0)
            break;


        for (int i=0;i<inliers->indices.size();i++){

            // Get Point
            pcl::PointXYZRGB pt = ds_cloud->points[inliers->indices[i]];

            // Copy point to new cloud
            pcl::PointXYZRGB pt_color;
            pt_color.x = pt.x;
            pt_color.y = pt.y;
            pt_color.z = pt.z;
            uint32_t rgb;
            rgb = colors[n_planes].getColor();
            pt_color.rgb = *reinterpret_cast<float*>(&rgb);
            cloud_pub->points.push_back(pt_color);

        }

        // Extract inliers
        extract.setInputCloud(ds_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(non_plane_cloud);
        ds_cloud->swap(non_plane_cloud);

        // Display infor
        ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
                "kinect",n_planes,
                 coefficients->values[0],(coefficients->values[1]>=0?"+":""),
                 coefficients->values[1],(coefficients->values[2]>=0?"+":""),
                 coefficients->values[2],(coefficients->values[3]>=0?"+":""),
                 coefficients->values[3],
                 inliers->indices.size(),original_size);
        ROS_INFO("%s: poitns left in cloud %i","Kinect",ds_cloud->width*ds_cloud->height);

        // Nest iteration
        n_planes++;
    }


  
  toROSMsg(non_plane_cloud, *intermediate_cloud);
  intermediate_cloud->header.frame_id = "kinect2_ir_optical_frame";   
  pub_intermediate.publish(intermediate_cloud);
 
  
  // // Segmentation

 


  // seg.setInputCloud(ds_cloud);
  // seg.segment(*inliers_floor, *coefficients_floor);

  // if (inliers_floor->indices.size() == 0)
  // {
  //   ROS_ERROR("Couldn't estimate a planar model from dataset!");
  //   return;
  // }

  // for (size_t i = 0; i < inliers_floor->indices.size(); ++i)
  // {
  //   ds_cloud->points[inliers_floor->indices[i]].r = 255;
  //   ds_cloud->points[inliers_floor->indices[i]].g = 0;
  //   ds_cloud->points[inliers_floor->indices[i]].b = 0;
  // }

  
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_node");
  ros::NodeHandle nh;
  ros::Subscriber sub;


  createColors();
  nh.getParam("/kinect_node/min_z", min_z);
  nh.getParam("/kinect_node/max_z", max_z);
  nh.getParam("/kinect_node/voxel_size", voxel_size);
  nh.getParam("/kinect_node/seg_dist_tr", seg_dist_tr);
  nh.getParam("/kinect_node/plane_perc", plane_perc);


  pub_intermediate = nh.advertise<PCloud2>("/intermediate", 1, true);
  sub = nh.subscribe("/kinect2/sd/points", 1, process_cloud);
  while (true)
  {
    ros::spinOnce();
  }
  
  
}