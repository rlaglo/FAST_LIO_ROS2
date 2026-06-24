#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <string>   
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h> 


// ring 만드는 법
// ring = floor((angle + FOV/2) / vertical_resolution);

// time 만드는 법: 반복문 시작시간 ~~ hz 만큼 균등분배
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)
// T* ptr; 
// ptr은 주소값, *ptr은 객체 자체, &ptr은 ptr의 주소값: &는 원본값을 가리키는 거임
// &a = x라고 하면, a를 x의 별명으로 정하는 거임, 주소값은 아님
// 실제 객체면 .으로 접근, 포인터면 ->로 접근
// Iterate over XYZ

class PcdConverter : public rclcpp::Node
{
public:
    PcdConverter() : Node("pcdconverter_node")
    {
        this->declare_parameter<std::string>("input_cloud_topic", "/lidar/points");
        this->declare_parameter<std::string>("lid_topic", "/lio/raw_points");

        this->get_parameter("input_cloud_topic", in_topic_);
        this->get_parameter("lid_topic", out_topic_);

        pub_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, 10);
        sub_ =  this->create_subscription<sensor_msgs::msg::PointCloud2>(in_topic_,10,std::bind(&PcdConverter::callbackFcn,this,std::placeholders::_1));
    }
private:
    std::string out_topic_;
    std::string in_topic_;
    void callbackFcn(const sensor_msgs::msg::PointCloud2::SharedPtr msg) // sharedptr는 주소값. msg는 주소값
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>()); // Ptr도 주소값. cloud는 주소값
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<PointXYZIRT>::Ptr output(new pcl::PointCloud<PointXYZIRT>());
        output->points.resize(cloud->points.size());
        const float scan_time = 0.1; //hz의 역수였나 내 sdf에서는 10 hz
        const int num_rings = 16; // vertical sample
        //const double hor_sample = 1800;
        const double min_angle_rad = -0.2618;   // -15도
        const double max_angle_rad = 0.2618;    // 15도
        const double v_fov_rad = max_angle_rad - min_angle_rad;
        uint32_t width = cloud->width; //즉 horizontal sample
        // vertical = 1800, height = 16
        // gz에서 넘어온 거 width=horizontal sample, height=vertical sample로 돼 있다고 함
        for (size_t i = 0; i< cloud->points.size();i++)
        {
            auto &p = cloud->points[i];
            auto &q = output->points[i];
            
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            q.intensity = 1.0;
            
            // NaN 체크 (필수 예외 처리) 좋다고 함
            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
                q.ring = 0;
                q.time = 0.0;
                continue;
            }

            //float angle std::atan2(p.z, std::sqrt(p.x*p.x + p.y*p.y));
            //q.ring = static_cast<uint16_t>(std::max(0,std::min(num_rings-1,ring)));
            q.ring = static_cast<uint16_t>(i / width);

            // t는 1800등분 되는 건데, 
            int col_idx = i % width; // 이건 0~1800 0~1800 0~1800 반복함
            q.time = scan_time * (static_cast<double>(col_idx) / static_cast<double>(width));
        }
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*output, out_msg);
        out_msg.header.stamp = msg->header.stamp;
        out_msg.header.frame_id = msg->header.frame_id;
        pub_->publish(out_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdConverter>());
  rclcpp::shutdown();
  return 0;
}