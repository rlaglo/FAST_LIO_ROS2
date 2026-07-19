#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
    std::uint16_t, ring,
    ring)(float, time, time))

class GazeboPointCloudConverter : public rclcpp::Node
{
public:
  GazeboPointCloudConverter()
  : Node("gazebo_pointcloud_converter")
  {
    declare_parameter<std::string>("common.input_cloud_topic", "/lidar/points");
    declare_parameter<std::string>("common.lid_topic", "/lio/raw_points");
    declare_parameter<double>("simulator.scan_rate", 10.0);
    declare_parameter<double>("simulator.min_vertical_angle_deg", -15.0);
    declare_parameter<double>("simulator.max_vertical_angle_deg", 15.0);
    declare_parameter<double>("simulator.max_range", 10.0);

    input_topic_ = get_parameter("common.input_cloud_topic").as_string();
    output_topic_ = get_parameter("common.lid_topic").as_string();
    scan_rate_ = get_parameter("simulator.scan_rate").as_double();
    min_vertical_angle_deg_ =
      get_parameter("simulator.min_vertical_angle_deg").as_double();
    max_vertical_angle_deg_ =
      get_parameter("simulator.max_vertical_angle_deg").as_double();
    max_range_ = get_parameter("simulator.max_range").as_double();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GazeboPointCloudConverter::convert, this, std::placeholders::_1));
  }

private:
  void convert(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*message, input);

    if (
      input.empty() || input.width < 2 || input.height < 2 || scan_rate_ <= 0.0 ||
      max_range_ <= 0.0 || min_vertical_angle_deg_ >= max_vertical_angle_deg_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid organized cloud or LiDAR parameters (width=%u, height=%u)",
        input.width, input.height);
      return;
    }

    pcl::PointCloud<PointXYZIRT> output;
    output.resize(input.size());
    output.width = input.width;
    output.height = input.height;
    output.is_dense = true;

    const double scan_time = 1.0 / scan_rate_;
    const std::size_t ring_count = input.height;
    const std::size_t column_count = input.width;
    const double min_vertical_angle = pcl::deg2rad(min_vertical_angle_deg_);
    const double vertical_fov =
      pcl::deg2rad(max_vertical_angle_deg_) - min_vertical_angle;
    const double clearing_range = max_range_ + 0.01;

    for (std::size_t index = 0; index < input.size(); ++index) {
      const auto & source = input[index];
      auto & target = output[index];
      const std::size_t ring = index / column_count;
      const std::size_t column = index % column_count;

      target.x = source.x;
      target.y = source.y;
      target.z = source.z;
      target.intensity = 1.0F;
      target.ring = static_cast<std::uint16_t>(ring);
      target.time = static_cast<float>(
        scan_time * static_cast<double>(column) / static_cast<double>(column_count));

      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
        const double horizontal_angle =
          -M_PI + 2.0 * M_PI * static_cast<double>(column) /
          static_cast<double>(column_count - 1);
        const double vertical_angle =
          min_vertical_angle + vertical_fov * static_cast<double>(ring) /
          static_cast<double>(ring_count - 1);

        target.x = static_cast<float>(
          clearing_range * std::cos(vertical_angle) * std::cos(horizontal_angle));
        target.y = static_cast<float>(
          clearing_range * std::cos(vertical_angle) * std::sin(horizontal_angle));
        target.z = static_cast<float>(clearing_range * std::sin(vertical_angle));
        target.intensity = -1.0F;
      }
    }

    sensor_msgs::msg::PointCloud2 converted;
    pcl::toROSMsg(output, converted);
    converted.header = message->header;
    publisher_->publish(converted);
  }

  std::string input_topic_;
  std::string output_topic_;
  double scan_rate_;
  double min_vertical_angle_deg_;
  double max_vertical_angle_deg_;
  double max_range_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPointCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
