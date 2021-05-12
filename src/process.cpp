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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include "std_msgs/String.h"
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


 #include <cv_bridge/cv_bridge.h>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>

typedef pcl::PointXYZRGB Point;
typedef sensor_msgs::PointCloud2 PCloud2;

ros::Publisher pub_intermediate, ros_image_pub;
image_transport::Publisher image_pub_;

ros::Subscriber sub; 
float min_z, max_z, voxel_size,seg_dist_tr, plane_perc;
int min_cluster_size, max_cluster_size;
cv::RNG rng(12345);

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
        for (int i=0;i<300000;i++){
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


void convert_point_cloud_to_opencvimage(pcl::PointCloud<Point>::Ptr point_cloud){
  float centre_x = 944.6798;
  float centre_y = 677.7635;
  float focal_x = 0.74;
  float focal_y = 0.77;
  int width = 512,  height =424;
  cv::Mat cv_image = cv::Mat(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

  for (int i=0; i<point_cloud->points.size();i++){
        float z = point_cloud->points[i].z*1000.0;
        float u = (point_cloud->points[i].x*1000.0*focal_x) / z;
        float v = (point_cloud->points[i].y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

    if (pixel_pos_x > (width-1)){
      pixel_pos_x = width -1;
    }
    if (pixel_pos_y > (height-1)){
      pixel_pos_y = height-1;
    }
    cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
  }

  cv_image.convertTo(cv_image, CV_16UC1);
  
  sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
  image_pub_.publish(output_image);
  std::cout << "Published images";
}


void process_cloud(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
  


  pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>), 
                         ds_cloud(new pcl::PointCloud<Point>),
                         filtered_cloud(new pcl::PointCloud<Point>),
                          cloud_pub(new pcl::PointCloud<Point>),
                        cloud_cluster(new pcl::PointCloud<Point>);
  pcl::PointCloud<Point> cloud_temp;

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

  // Filter for depth
  pass_filter.setInputCloud(cloud_in);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z, max_z);
  pass_filter.filter(*filtered_cloud);

  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(filtered_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*filtered_cloud);
  

  // Downsampling of the PCL
  voxel.setInputCloud(filtered_cloud);
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setDownsampleAllData(true);
  voxel.filter(*ds_cloud);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(seg_dist_tr);
  seg.setMaxIterations(1000);

 // New segmentation
    int original_size = ds_cloud->height*ds_cloud->width;
    int n_planes = 0;
    int count = 0;
    while (ds_cloud->height*ds_cloud->width > original_size*plane_perc){
       
        // count += 1;
        // if(count == 4)
        //     break;
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
        extract.filter(cloud_temp);
        ds_cloud->swap(cloud_temp);

        // Display infor
        // ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
        //         "kinect",n_planes,
        //          coefficients->values[0],(coefficients->values[1]>=0?"+":""),
        //          coefficients->values[1],(coefficients->values[2]>=0?"+":""),
        //          coefficients->values[2],(coefficients->values[3]>=0?"+":""),
        //          coefficients->values[3],
        //          inliers->indices.size(),original_size);
        // ROS_INFO("%s: poitns left in cloud %i","Kinect",ds_cloud->width*ds_cloud->height);

        // Nest iteration
        n_planes++;
    }

      toROSMsg(*ds_cloud, *intermediate_cloud);
      intermediate_cloud->header.frame_id = "kinect2_ir_optical_frame";   
      pub_intermediate.publish(intermediate_cloud);
  
 
  

  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(ds_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(ds_cloud);
  ec.extract(cluster_indices);
  createColors();


  float centre_x = cam_info->K[2];
  float centre_y = cam_info->K[5];
  float focal_x = cam_info->K[0];
  float focal_y = cam_info->K[4];
  int width = cam_info->width,  height = cam_info->height;
 
  // std::cout <<  centre_x << " " << centre_y << " "   << focal_x << " "  << focal_y << " "  << width  << " "  << height;
                //  255.141          207.707               365.013              0                512 424 
  cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat canvas = cv::Mat::zeros(width,height,CV_8UC1);
  cv::Mat median_blur_canvas =  cv::Mat::zeros(width,height,CV_8UC1);
  int j=0;
  float depth[width][height];
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    for (const auto& idx : it->indices){

        // Get Point
        pcl::PointXYZRGB pt = ds_cloud->points[idx];

        // Copy point to new cloud
        pcl::PointXYZRGB pt_color;
        pt_color.x = pt.x;
        pt_color.y = pt.y;
        pt_color.z = pt.z;
        uint32_t rgb;
        rgb = colors[j].getColor();
        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
        cloud_cluster->points.push_back(pt_color);

        float z = pt.z*1000.0;
        float u = (pt.x*1000.0*focal_x) / z;
        float v = (pt.y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);
        if (pixel_pos_x > (cv_ptr->image.cols-1)){
          pixel_pos_x = cv_ptr->image.cols-1;
        }
      if (pixel_pos_y > (cv_ptr->image.rows-1)){
        pixel_pos_y = cv_ptr->image.rows-1;
      }
      if(pixel_pos_y < 0){
        pixel_pos_y = 0;
      }
      if(pixel_pos_x < 0){
        pixel_pos_x = 0;
      }
        // std::cout << pixel_pos_x << pixel_pos_x << "pixel pos";
        cv_ptr->image.at<cv::Vec3b>(pixel_pos_y,pixel_pos_x)[0] = 255; 
			  cv_ptr->image.at<cv::Vec3b>(pixel_pos_y,pixel_pos_x)[1] = 255;  
			  cv_ptr->image.at<cv::Vec3b>(pixel_pos_y,pixel_pos_x)[2] = 255;  
        canvas.at<unsigned char>(pixel_pos_y, pixel_pos_x) = 255;
        depth[pixel_pos_y][pixel_pos_x] = z;
    }
    j++;
  }
  // for(int i=0;i<width;i++){
  //   for(int j=0;j<height;j++){
  //     std::cout << depth[i][j] << " ";
  //   }
  //   std::cout << "\n";
  // }

  toROSMsg(*cloud_cluster, *intermediate_cloud);
  intermediate_cloud->header.frame_id = "kinect2_ir_optical_frame";   
  pub_intermediate.publish(intermediate_cloud);
  
  cv::GaussianBlur(canvas, median_blur_canvas,  cv::Size(15, 15), 0, 0 );
 
  image_pub_.publish(cv_ptr->toImageMsg());
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(median_blur_canvas, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  std::vector<cv::Moments> mu(contours.size());
  std::vector<cv::Point2f> mc(contours.size());
  cv::Mat drawing = cv::Mat::zeros(cv_ptr->image.size(), CV_8UC3);
  
  cv_bridge::CvImagePtr depth_ptr;
    try
    {
      depth_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  

  for(int i = 0; i< contours.size(); i++)
  {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
    mu[i] = moments(contours[i], true);
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
    drawing.at<uchar>(int(mu[i].m10/mu[i].m00) , int(mu[i].m01/mu[i].m00)) = 0;

    //Calculating distance between cluster center and image center
    int ox = int(mu[i].m10/mu[i].m00);
    int oy = int(mu[i].m01/mu[i].m00);
    float depth_o = depth_ptr->image.at<float>(ox, oy);

    float depth_c = depth_ptr->image.at<float>(int(centre_x), int(centre_y));

    float vx = (ox - centre_x)*depth_o; 
    float vy = (oy - centre_y)*depth_o;
    float vz =  depth_o - depth_c ;


    float distance = sqrt(vx*vx + vy*vy + vz*vz);
    std::cout << "Distance: " << depth_o << " " <<  depth_c << " "  << ox << " "  << oy << " "  << centre_x << " " << centre_y << "\n";
    7.94645e-34 4.93612e-33                                             39           269               255.141         207.707

  } 
  
  



cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg; 
std_msgs::Header header; 
header.stamp = ros::Time::now(); 
img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, drawing);
//img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, median_blur_canvas);
img_bridge.toImageMsg(img_msg); 
ros_image_pub.publish(img_msg);

//image_pub_.publish(cv_ptr->toImageMsg());
//convert_point_cloud_to_opencvimage(cloud_cluster);

 
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
  nh.getParam("/kinect_node/min_cluster_size", min_cluster_size);
  nh.getParam("/kinect_node/max_cluster_size", max_cluster_size);
  pub_intermediate = nh.advertise<PCloud2>("/intermediate", 1, true);
  ros_image_pub = nh.advertise<sensor_msgs::Image>("/countours", 1, true);
  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);


  //sub = nh.subscribe("/kinect2/sd/points", 1, process_cloud);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/kinect2/sd/image_depth_rect", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/kinect2/sd/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/kinect2/sd/points", 1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sync(image_sub, info_sub, pc_sub, 10);
  sync.registerCallback(boost::bind(&process_cloud, _1, _2, _3));




  while (true)
  {
    ros::spinOnce();
  }
  
  
}

