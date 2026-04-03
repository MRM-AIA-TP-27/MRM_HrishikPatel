#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/d430/depth/points", 10,
      std::bind(&ObstacleDetectionNode::pointCloudBufferCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ObstacleDetectionNode::processPointCloud, this));
  }

private:
  sensor_msgs::msg::PointCloud2::SharedPtr latest_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  pcl::PassThrough<pcl::PointXYZ> pass;

  void pointCloudBufferCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_msg_ = msg;
  }

  void processPointCloud()
  {
    if (!latest_msg_) {
      RCLCPP_WARN(this->get_logger(), "No point cloud received yet.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*latest_msg_, *cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.05, 1.0); // Ignore anything below 5cm and above 1 meter
    pass.filter(*cloud);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5.0); // Ignore everything further than 3 meters
    pass.filter(*cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*cloud_voxel);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_voxel);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_voxel);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_ground);
    if (cloud_no_ground->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No points left after ground removal!");
      return;  // Skip the clustering
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_no_ground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(15000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_ground);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& indices : cluster_indices) {
        for (const auto& idx : indices.indices) {
            clustered_cloud->points.push_back(cloud_no_ground->points[idx]);
        }
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header = latest_msg_->header;
    pub_->publish(output);

    pcl::PointXYZ nearest_point;
    double min_distance = std::numeric_limits<double>::max();
    bool obstacle_within_range = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    int cluster_id = 0;

    for (const auto& indices : cluster_indices)
    {
      uint8_t r = rand() % 256;
      uint8_t g = rand() % 256;
      uint8_t b = rand() % 256;

      for (const auto& idx : indices.indices)
      {
        pcl::PointXYZRGB pt;
        pt.x = cloud_no_ground->points[idx].x;
        pt.y = cloud_no_ground->points[idx].y;
        pt.z = cloud_no_ground->points[idx].z;
        pt.r = r;
        pt.g = g;
        pt.b = b;
        clustered_cloud_rgb->points.push_back(pt);
      }

      cluster_id++;
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    RCLCPP_INFO(this->get_logger(), "Detected %ld obstacles", cluster_indices.size());
    pcl::toROSMsg(*clustered_cloud, output);
    output.header = latest_msg_->header;
    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}