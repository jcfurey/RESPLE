#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <omp.h>

#include "utils/eigen_utils.hpp"

// NOTE: NUM_OF_THREAD and NUM_MATCH_POINTS are now ROS parameters in RESPLE node
// They were previously global variables but are now configurable per-node instance
// int NUM_OF_THREAD = 5;
// int NUM_MATCH_POINTS = 5;

struct ImuData {
    int64_t time_ns;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Eigen::Matrix<double, 6, 24> H;
    Eigen::Matrix<double, 6, 1> imu_itp;
    ImuData(){}
    ImuData(const int64_t s, const Eigen::Vector3d& w, const Eigen::Vector3d& a)
      : time_ns(s), gyro(w), accel(a), H(Eigen::Matrix<double, 6, 24>::Zero()), imu_itp(Eigen::Matrix<double, 6, 1>::Zero()) {}

    ImuData(const ImuData& other) : time_ns(other.time_ns), gyro(other.gyro), accel(other.accel),
    H(other.H), imu_itp(other.imu_itp) {
    }          
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PoseData {
    int64_t time_ns;
    Eigen::Quaterniond orient;
    Eigen::Vector3d pos;
    PoseData(){}
    PoseData(int64_t s, Eigen::Quaterniond& q, Eigen::Vector3d& t) : time_ns(s), orient(q), pos(t) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
}  

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
)

namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace livox_mid360_boxi {
    struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint8_t tag;
      uint8_t line;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
  POINT_CLOUD_REGISTER_POINT_STRUCT(livox_mid360_boxi::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, intensity, intensity)
      (uint8_t, tag, tag)
      (uint8_t, line, line)
      (double, timestamp, timestamp)
  )

class CommonUtils
{
public:
    template <typename T>
    static T readParam(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, const std::string& name, const T& alternative)
    {
        T ans;
        if (!node_params->has_parameter(name)) {
            rclcpp::ParameterValue default_parameter_value(alternative);  
            rcl_interfaces::msg::ParameterDescriptor descriptor;          
            node_params->declare_parameter(name, default_parameter_value, descriptor, false);
        }
        ans = node_params->get_parameter(name).get_parameter_value().get<T>();
        return ans;
    }

    template <typename T>
    static T readParam(const rclcpp::Node::SharedPtr &n, std::string name)
    {
        T ans;
        if (!n->has_parameter(name)) {
            n->declare_parameter<T>(name);
        }        
        if (!n->get_parameter(name, ans)) {
            RCLCPP_FATAL_STREAM(n->get_logger(), "Failed to load " << name);
            exit(1);
        }
        return ans;
    }

    template <typename T>
    static T readParam(const rclcpp::Node::SharedPtr &n, std::string name, const T& alternative)
    {
        T ans;
        if (!n->has_parameter(name)) {
            n->declare_parameter<T>(name, alternative);
        }
        n->get_parameter_or(name, ans, alternative);
        return ans;
    }
    
    static Eigen::Vector3d readVector3d(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, const std::string& name)
    {
        std::vector<double> v = CommonUtils::readParam<std::vector<double>>(node_params, name, {0.0, 0.0, 0.0});
        return Eigen::Vector3d(v[0], v[1], v[2]);
    } 

    static Eigen::Vector3d readVector3d(rclcpp::Node::SharedPtr &n, const std::string& name)
    {
        std::vector<double> v = CommonUtils::readParam<std::vector<double>>(n, name);
        return Eigen::Vector3d(v[0], v[1], v[2]);
    }          

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }      

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
                sin(y), cos(y), 0,
                0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
                0., 1., 0.,
                -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
                0., cos(r), -sin(r),
                0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
    {
        Eigen::Matrix3d R0;
        Eigen::Vector3d ng1 = g.normalized();
        Eigen::Vector3d ng2{0, 0, 1.0};
        R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
        double yaw = CommonUtils::R2ypr(R0).x();
        R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        return R0;
    }    

    static int64_t ms2ns(const float t_ms) {
        return static_cast<int64_t>(static_cast<double>(t_ms) * 1e6);
    }    

    static bool time_list(pcl::PointXYZINormal &x, pcl::PointXYZINormal &y) {return (x.intensity < y.intensity);};

    // Create Pose
    static geometry_msgs::msg::Pose pose2msg(const Eigen::Vector3d& pos,
                                             const Eigen::Quaterniond& orient)
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = pos.x();
        msg.position.y = pos.y();
        msg.position.z = pos.z();
        msg.orientation.w = orient.w();
        msg.orientation.x = orient.x();
        msg.orientation.y = orient.y();
        msg.orientation.z = orient.z();
        return msg;
    }

    // Create PoseStamped for path messages (nav_msgs::msg::Path)
    static geometry_msgs::msg::PoseStamped pose2msg(const std::string& frame_id, 
                                                    const int64_t t, const Eigen::Vector3d& pos,
                                                    const Eigen::Quaterniond& orient)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = frame_id, 
        msg.header.stamp = rclcpp::Time(t);
        msg.pose = pose2msg(pos, orient);
        
        return msg;
    }

    // Create PoseWithCovarianceStamped for pose topics (for Nav2 compatibility)
    static geometry_msgs::msg::PoseWithCovarianceStamped pose2msg(const std::string& frame_id, 
                                                                  const int64_t t, const Eigen::Vector3d& pos,
                                                                  const Eigen::Quaterniond& orient, Eigen::Vector<double, 6>& cov)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = rclcpp::Time(t);
        msg.pose.pose = pose2msg(pos, orient);

        msg.pose.covariance[0] = cov[0];
        msg.pose.covariance[7] = cov[1];
        msg.pose.covariance[14]= cov[2];
        msg.pose.covariance[21]= cov[3];
        msg.pose.covariance[28]= cov[4];
        msg.pose.covariance[35]= cov[5];
        return msg;
    }    

    static geometry_msgs::msg::Point32 getPointMsg(const Eigen::Vector3d& p)
    {
        geometry_msgs::msg::Point32 p_msg;
        p_msg.x = p.x();
        p_msg.y = p.y();
        p_msg.z = p.z();
        return p_msg;
    }    

    template<typename T>
    static bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const Eigen::aligned_vector<pcl::PointXYZINormal> &point, const T &threshold)
    {
        const int num_pts = static_cast<int>(point.size());
        if (num_pts < 3) { return false; }
        Eigen::Matrix<T, Eigen::Dynamic, 3> A(num_pts, 3);
        Eigen::Matrix<T, Eigen::Dynamic, 1> b(num_pts, 1);
        A.setZero();
        b.setOnes();
        b *= -1.0f;
        for (int j = 0; j < num_pts; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }
        Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
        T n = normvec.norm();
        if (n < static_cast<T>(1e-12)) { return false; }
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;
        for (int j = 0; j < num_pts; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }
};

struct LidarConfig {
    std::string topic;
    std::string type;
    int scan_line;
    float blind;
    Eigen::Quaterniond q_lb;
    Eigen::Vector3d t_lb;
    Eigen::Quaterniond q_bl;
    Eigen::Vector3d t_bl;
    double w_pt;
    // Sensor origin in body (base_link) frame — populated from TF at runtime,
    // NOT from YAML.  Used to compute true sensor-frame range for outlier gating.
    Eigen::Vector3d sensor_origin_body = Eigen::Vector3d::Zero();

    LidarConfig() = default;

    LidarConfig(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, const std::string& prefix) {
        std::cout << "Creating LidarConfig with prefix: \"" << prefix << "\"" << std::endl;
        topic = CommonUtils::readParam<std::string>(node_params, prefix + "topic_lidar", "");
        type = CommonUtils::readParam<std::string>(node_params, prefix + "lidar_type", "");
        scan_line = CommonUtils::readParam<int>(node_params, prefix + "scan_line", 0);
        blind = CommonUtils::readParam<float>(node_params, prefix + "blind", 0.5f);
        std::vector<double> q_lb_v = CommonUtils::readParam<std::vector<double>>(node_params, prefix + "q_lb", {1.0, 0.0, 0.0, 0.0});
        q_lb = Eigen::Quaterniond(q_lb_v.at(0), q_lb_v.at(1), q_lb_v.at(2), q_lb_v.at(3));
        t_lb = CommonUtils::readVector3d(node_params, prefix + "t_lb");
        q_bl = q_lb.inverse();
        t_bl = q_lb.inverse() * (- t_lb);
        w_pt = CommonUtils::readParam<double>(node_params, prefix + "w_pt", 0.0);
    }
};

struct Parameters {
    Eigen::Vector3d cov_acc;
    Eigen::Vector3d cov_gyro;
    Eigen::Vector3d gravity;
    double nn_thresh;
    double coeff_cov;

    Parameters() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointData {
    int64_t time_ns;
    pcl::PointXYZINormal pt;
    Eigen::Vector3d pt_b;
    pcl::PointXYZINormal pt_w;
    Eigen::Vector3d normvec;
    bool if_valid;
    double dist;
    Eigen::aligned_vector<pcl::PointXYZINormal> nearest_points;
    double zp = 0;
    Eigen::Matrix<double, 1, 24> H = Eigen::Matrix<double, 1, 24>::Zero();
    Eigen::Quaterniond q_bl;
    Eigen::Vector3d t_bl;
    double var_pt;
    // True sensor-frame range: distance from the LiDAR origin, NOT from base_link.
    // pt_b is stored in base_link frame (after the PCL extrinsic transform), so
    // pt_b.norm() would give the distance from base_link — wrong for the outlier
    // gate which scales with actual measurement range.
    float range_sensor = 0.f;

    PointData() {};
    PointData(const pcl::PointXYZINormal& pt_in, int64_t fr_start_time, const Eigen::Quaterniond& q_bl_in,
        const Eigen::Vector3d& t_bl_in, double w_pt,
        const Eigen::Vector3d& sensor_origin_body = Eigen::Vector3d::Zero())
        : pt(pt_in), pt_b(Eigen::Vector3d(pt_in.x, pt_in.y, pt_in.z)),
        q_bl(q_bl_in), t_bl(t_bl_in) {
        time_ns = fr_start_time + CommonUtils::ms2ns(pt_in.intensity);
        if_valid = false;
        dist = 0;
        var_pt = w_pt;
        normvec = Eigen::Vector3d::Zero();
        range_sensor = static_cast<float>((pt_b - sensor_origin_body).norm());
    }

    PointData(const PointData& other) : time_ns(other.time_ns), pt(other.pt),
        pt_b(other.pt_b), pt_w(other.pt_w), normvec(other.normvec),
        if_valid(other.if_valid), dist(other.dist),
        nearest_points(other.nearest_points),
        zp(other.zp), H(other.H), q_bl(other.q_bl), t_bl(other.t_bl), var_pt(other.var_pt),
        range_sensor(other.range_sensor) {
    }

    PointData& operator=(const PointData& other) {
        if (this != &other) {
            this->time_ns = other.time_ns;
            this->pt = other.pt;
            this->pt_b = other.pt_b;
            this->pt_w = other.pt_w;
            this->normvec = other.normvec;
            this->if_valid = other.if_valid;
            this->nearest_points = other.nearest_points;
            this->dist = other.dist;
            this->zp = other.zp;
            this->H = other.H;
            this->q_bl = other.q_bl;
            this->t_bl = other.t_bl;
            this->var_pt = other.var_pt;
            this->range_sensor = other.range_sensor;
        }
        return *this;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


