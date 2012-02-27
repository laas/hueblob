#include <ros/ros.h>
#include <ros/console.h>

// Image transport.
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


// Message filters.
# include <message_filters/subscriber.h>
# include <message_filters/sync_policies/exact_time.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <message_filters/synchronizer.h>

// Msgs
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <hueblob/RoiStamped.h>
#include <sensor_msgs/image_encodings.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include "pcl/filters/statistical_outlier_removal.h"

#include "cv.h"
#include "highgui.h"

#include <cv_bridge/cv_bridge.h>


namespace
{

  inline void projectTo3d(float u, float v, float disparity,
                          const stereo_msgs::DisparityImage &disparity_image,
                          const sensor_msgs::CameraInfo &camera_info,
                          float &x, float &y, float &z
                          )
  {

    float fx = camera_info.P[0*4+0];
    float fy = camera_info.P[1*4+1];
    float cx = camera_info.P[0*4+2];
    float cy = camera_info.P[1*4+2];
    // float Tx = camera_info.P[0*4+3];
    // float Ty = camera_info.P[1*4+3];

    z = disparity_image.f * disparity_image.T / disparity;
    x = ( (u - cx ) / fx );
    x*= z;
    y = ( (v - cy ) / fy );
    y*= z;
  }

  //! Check if a disparity image as a valid value at given image coordinates
  inline bool hasDisparityValue(const stereo_msgs::DisparityImage
                                &disparity_image, unsigned int h, unsigned int w)
  {
    if (h >= disparity_image.image.height || w >= disparity_image.image.width)
      return false;
    float val;
    memcpy(&val, &(disparity_image.image.data.at(
                                                 h*disparity_image.image.step
                                                 + sizeof(float)*w )), sizeof(float));
    if (std::isnan(val) || std::isinf(val)) return false;
    if (val < disparity_image.min_disparity || val >
        disparity_image.max_disparity) return false;
    return true;
  }
  //! Get the value from an image at given image coordinates as a float
  inline void getPoint(const sensor_msgs::Image &image, unsigned int h,
                       unsigned int w, float &val)
  {
    ROS_ASSERT(h<image.height && w<image.width);
    memcpy(&val, &(image.data.at( h*image.step + sizeof(float)*w )),
           sizeof(float));
  }


  void get3dCloud(const stereo_msgs::DisparityImage &disparity_image,
                  const sensor_msgs::CameraInfo &camera_info,
                  const sensor_msgs::Image &image,
                  const hueblob::RoiStamped & roi_stamped,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
                  )
  {
    namespace enc = sensor_msgs::image_encodings;
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
      cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }


    const sensor_msgs::RegionOfInterest roi = roi_stamped.roi;
    for (unsigned i = roi.y_offset; i < roi.y_offset + roi.height; ++i)
      for (unsigned j = roi.x_offset; j < roi.x_offset + roi.width; ++j)
        {
          if (!hasDisparityValue(disparity_image, i, j))
            continue;
          float disparity, x, y, z;

          getPoint(disparity_image.image, i, j, disparity);
          ROS_ASSERT(disparity_image.max_disparity != 0.0);
          if (disparity == 0)
            continue;
          projectTo3d(j, i, disparity,  disparity_image,
                      camera_info, x, y, z);
          pcl::PointXYZRGB p;
          p.x = x;
          p.y = y;
          p.z = z;
          cv::Vec3b rgb = cv_ptr->image.at<cv::Vec3b>(i,j);
          p.r = rgb[2];
          p.g = rgb[1];
          p.b = rgb[0];
          //ROS_INFO_STREAM(rgb[0]);
          cloud->points.push_back(p);
          cloud->header = roi_stamped.header;
        }
  }

} // end of anonymous namespace.




using namespace std;
class Projector
{
public:
  Projector();
  virtual ~Projector(){};

private:
  void callback(const sensor_msgs::CameraInfoConstPtr& info,
                const sensor_msgs::ImageConstPtr& image,
                const stereo_msgs::DisparityImageConstPtr& disparity,
                const hueblob::RoiStampedConstPtr& box
                );

  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::CameraInfo,
                                                           sensor_msgs::Image,
                                                           stereo_msgs::DisparityImage,
                                                           hueblob::RoiStamped
                                                           > ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;


  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ApproximateSync sync_;
  message_filters::Subscriber<hueblob::RoiStamped> roi_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;
  image_transport::SubscriberFilter image_sub_;
  ros::Publisher cloud_pub_, cloud_filtered_pub_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_;

};


Projector::Projector()
  : nh_("blob2CloudProjector"),
    it_(nh_),
    sync_(50),
    roi_sub_(),
    camera_info_sub_(),
    disparity_sub_(),
    sor_()
{
  string blob2d_topic, disparity_topic, cloud_topic, \
    camera_info_topic, image_topic, cloud_filtered_topic;
  ros::param::param<string>("~blob2d", blob2d_topic, "blobs/rose/blob2d");
  ros::param::param<string>("~disparity", disparity_topic, "disparity");
  ros::param::param<string>("~camera_info", camera_info_topic, "left/camera_info");
  ros::param::param<string>("~image", image_topic, "left/image_rect_color");
  ros::param::param<string>("~cloud", cloud_topic, "blobs/rose/points");



  sor_.setMeanK (50);
  sor_.setStddevMulThresh (1.0);

  blob2d_topic      = ros::names::resolve(blob2d_topic);
  disparity_topic   = ros::names::resolve(disparity_topic);
  cloud_topic       = ros::names::resolve(cloud_topic);
  camera_info_topic = ros::names::resolve(camera_info_topic);
  image_topic       = ros::names::resolve(image_topic);
  cloud_filtered_topic = ros::names::append(cloud_topic, "_filtered");

  cloud_pub_  = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (cloud_topic, 1);
  cloud_filtered_pub_  = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (cloud_filtered_topic, 1);


  roi_sub_.subscribe(nh_, blob2d_topic, 10);
  camera_info_sub_.subscribe(nh_, camera_info_topic, 10);
  image_sub_.subscribe(it_, image_topic, 10);
  disparity_sub_.subscribe(nh_, disparity_topic, 10);

  sync_.connectInput(camera_info_sub_, image_sub_, disparity_sub_, roi_sub_);
  sync_.registerCallback(boost::bind(&Projector::callback,
                                     this, _1, _2, _3, _4));

  ROS_INFO_STREAM(endl<< "Listening to:"
                  << "\n\t* " << blob2d_topic
                  << "\n\t* " << disparity_topic
                  << "\n\t* " << camera_info_topic
                  << endl
                  << "Publishing to:"
                  << "\n\t* " << cloud_topic);

}

void Projector::callback(const sensor_msgs::CameraInfoConstPtr& info,
                         const sensor_msgs::ImageConstPtr& image,
                         const stereo_msgs::DisparityImageConstPtr& disparity,
                         const hueblob::RoiStampedConstPtr& roi_stamped
                )
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  //ROS_INFO_STREAM(box->x << " " << box->y << " " << box->width << " " << box->height);

  get3dCloud(*disparity, *info, *image,
             *roi_stamped,
             cloud);
  sor_.setInputCloud (cloud);
  sor_.filter (*cloud_filtered);

  cloud_pub_.publish(cloud);
  cloud_filtered_pub_.publish(cloud_filtered);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blob2CloudProjector");
  Projector p;
  ros::spin();
}
