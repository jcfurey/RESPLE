// Synthetic data injector for sanitizing RESPLE's data path.
//
// Publishes (all from one C++ node — no rclpy needed):
//   * static TF  base_link <- os_lidar  and  base_link <- os_imu (identity)
//   * /imu/data  at 100 Hz: stationary IMU (gravity on +Z) so gravity-init runs
//   * /points    at 10 Hz: a static "room" point cloud (6 planes), zero motion,
//                with x,y,z,intensity,t fields (t = relative ns offset in scan)
//
// Zero motion + a static scene means the estimator stays near the origin and
// the worker runs its full pipeline every scan (deskew -> findCorresp parallel
// k-NN -> IEKF -> mapIncremental Add_Points -> lasermapFovSegment
// Delete_Point_Boxes -> spline update). Run RESPLE under TSan/ASan alongside
// this to sanitize that data-flow concurrency.

#include <chrono>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

class Injector : public rclcpp::Node
{
public:
  Injector() : Node("data_injector"), rng_(42)
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points", rclcpp::SensorDataQoS());
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publishStaticTf("base_link", "os_lidar");
    publishStaticTf("base_link", "os_imu");

    buildScene();

    imu_timer_ = create_wall_timer(10ms, std::bind(&Injector::imuTick, this));   // 100 Hz
    pc_timer_ = create_wall_timer(100ms, std::bind(&Injector::pcTick, this));     // 10 Hz
    RCLCPP_INFO(get_logger(), "data_injector up: %zu scene points, IMU 100Hz, cloud 10Hz", scene_.size());
  }

private:
  struct P { float x, y, z; };

  void publishStaticTf(const std::string& parent, const std::string& child)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.rotation.w = 1.0;  // identity
    static_tf_->sendTransform(t);
  }

  // A 10x10x4 m room: floor, ceiling, 4 walls. Dense enough for plane fits.
  void buildScene()
  {
    std::uniform_real_distribution<float> j(-0.01f, 0.01f);
    auto add = [&](float x, float y, float z) { scene_.push_back({x + j(rng_), y + j(rng_), z + j(rng_)}); };
    for (float a = -5; a <= 5; a += 0.25f)
      for (float b = -5; b <= 5; b += 0.25f) {
        add(a, b, 0.0f);   // floor
        add(a, b, 4.0f);   // ceiling
      }
    for (float a = -5; a <= 5; a += 0.25f)
      for (float c = 0; c <= 4; c += 0.25f) {
        add(a, -5.0f, c);  // wall -y
        add(a, 5.0f, c);   // wall +y
        add(-5.0f, a, c);  // wall -x
        add(5.0f, a, c);   // wall +x
      }
  }

  void imuTick()
  {
    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = "os_imu";
    std::normal_distribution<double> n(0.0, 0.01);
    m.linear_acceleration.x = n(rng_);
    m.linear_acceleration.y = n(rng_);
    m.linear_acceleration.z = 9.81 + n(rng_);  // gravity on +Z, stationary
    m.angular_velocity.x = n(rng_);
    m.angular_velocity.y = n(rng_);
    m.angular_velocity.z = n(rng_);
    imu_pub_->publish(m);
  }

  void pcTick()
  {
    sensor_msgs::msg::PointCloud2 c;
    c.header.stamp = now();
    c.header.frame_id = "os_lidar";
    c.height = 1;
    c.width = scene_.size();
    c.is_bigendian = false;
    c.is_dense = true;
    // fields: x,y,z (float32), intensity (float32), t (uint32) -> point_step 20
    auto addField = [&](const char* name, uint32_t off, uint8_t dtype) {
      sensor_msgs::msg::PointField f;
      f.name = name; f.offset = off; f.datatype = dtype; f.count = 1;
      c.fields.push_back(f);
    };
    addField("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    addField("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    addField("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    addField("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    addField("t", 16, sensor_msgs::msg::PointField::UINT32);
    c.point_step = 20;
    c.row_step = c.point_step * c.width;
    c.data.resize(c.row_step);
    const uint32_t scan_ns = 100000000u;  // 100 ms scan
    for (size_t i = 0; i < scene_.size(); ++i) {
      uint8_t* p = c.data.data() + i * c.point_step;
      float xyz[4] = {scene_[i].x, scene_[i].y, scene_[i].z, 100.0f};
      std::memcpy(p, xyz, 16);
      uint32_t t = static_cast<uint32_t>((double)i / scene_.size() * scan_ns);
      std::memcpy(p + 16, &t, 4);
    }
    pc_pub_->publish(c);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr imu_timer_, pc_timer_;
  std::vector<P> scene_;
  std::mt19937 rng_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Injector>());
  rclcpp::shutdown();
  return 0;
}
