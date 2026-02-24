#include <scanner_assembler/srv/assembler_query.hpp>
#include <deque>
#include <functional>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <utility>

class Assembler : public rclcpp::Node {
public:
  Assembler() : Node("assembler") {
    // Parameters
    fixed_frame_ = declare_parameter("fixed_frame", "map");
    scan_buffer_size_ = declare_parameter("scan_buffer_size", 400);

    // Create interface
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    // Create buffers
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscriber
    // cloud_sub_.subscribe(this, "/cloud_in"); ////tidak compatible dengan QOS Sensor data
    cloud_sub_.subscribe(this, "/cloud_in", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    // Create synchronizer
    tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            cloud_sub_, *tf_buffer_, fixed_frame_, 10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());

    tf_filter_->registerCallback(&Assembler::handle_cloud_sub, this);

    // Create service
    service_ = create_service<scanner_assembler::srv::AssemblerQuery>(
        "assembler_service",
        std::bind(&Assembler::handle_service, this, std::placeholders::_1,
                  std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "Assembler node initialized with buffer size: %zu",
                scan_buffer_size_);
  }

private:
  void handle_cloud_sub(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Lock mutex for thread safety when modifying the queue
    std::lock_guard<std::mutex> lock(queue_mutex_);

    uint32_t n_points = msg->width * msg->height;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_;
    pcl_cloud_.resize(n_points);

    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(fixed_frame_, msg->header.frame_id,
                                      msg->header.stamp);

      // Perform transform here
      sensor_msgs::PointCloud2ConstIterator<float> x_in(*msg, std::string("x"));
      sensor_msgs::PointCloud2ConstIterator<float> y_in(*msg, std::string("y"));
      sensor_msgs::PointCloud2ConstIterator<float> z_in(*msg, std::string("z"));
      sensor_msgs::PointCloud2ConstIterator<float> int_in(
          *msg, std::string("intensity"));

      tf2::Transform tf2_transform;
      tf2::fromMsg(transform_stamped.transform, tf2_transform);

      for (std::size_t i = 0; x_in != x_in.end();
           ++x_in, ++y_in, ++z_in, ++int_in, ++i) {
        tf2::Vector3 point_local(*x_in, *y_in, *z_in);
        tf2::Vector3 point_global = tf2_transform * point_local;
        pcl_cloud_.points[i] = pcl::PointXYZI(
            point_global.x(), point_global.y(), point_global.z(), *int_in);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Transform Exception %s", ex.what());
      return;
    }

    // Pop to prevent overflow of buffer
    if (cloud_queue_.size() == scan_buffer_size_) {
      total_points -= cloud_queue_.front().first.size();
      cloud_queue_.pop_front();
    }

    cloud_queue_.push_back(std::pair(std::move(pcl_cloud_), msg->header.stamp));
    total_points += n_points;
  }

  void handle_service(
      const std::shared_ptr<scanner_assembler::srv::AssemblerQuery::Request> req,
      std::shared_ptr<scanner_assembler::srv::AssemblerQuery::Response> res) {

    rclcpp::Time begin_time(req->begin);
    rclcpp::Time end_time(req->end);

    if (end_time <= begin_time) {
      RCLCPP_ERROR(get_logger(),
                   "Invalid time range. End time (%lf) must be greater than "
                   "begin time (%lf)",
                   begin_time.seconds(), end_time.seconds());
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Service called. Requested time range: %lf to %lf",
                begin_time.seconds(), end_time.seconds());

    pcl::PointCloud<pcl::PointXYZI> merged_cloud;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);

      for (const auto &cloud_pair : cloud_queue_) {
        const auto &cloud = cloud_pair.first;
        const auto &timestamp = cloud_pair.second;

        // Merge all clouds between the timestamps
        if (timestamp >= begin_time && timestamp <= end_time) {
          merged_cloud += cloud;
        }
      }
    }

    // Manually create PointCloud2 message
    sensor_msgs::msg::PointCloud2 &cloud_out = res->cloud;

    // Set basic header information
    cloud_out.header.stamp = this->now();
    cloud_out.header.frame_id = fixed_frame_;

    // Define the point fields for a PointXYZ cloud
    cloud_out.fields.resize(4);
    cloud_out.fields[0].name = "x";
    cloud_out.fields[0].offset = 0;
    cloud_out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[0].count = 1;

    cloud_out.fields[1].name = "y";
    cloud_out.fields[1].offset = 4;
    cloud_out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[1].count = 1;

    cloud_out.fields[2].name = "z";
    cloud_out.fields[2].offset = 8;
    cloud_out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[2].count = 1;

    cloud_out.fields[3].name = "intensity";
    cloud_out.fields[3].offset = 12;
    cloud_out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[3].count = 1;

    // Set cloud width and height
    cloud_out.height = 1; // unorganized cloud
    cloud_out.width = merged_cloud.size();

    // Set other properties
    cloud_out.is_bigendian = false;
    cloud_out.point_step = 16; // 3 * sizeof(float)
    cloud_out.row_step = cloud_out.point_step * cloud_out.width;
    cloud_out.is_dense = false;

    // Allocate memory for the point data
    cloud_out.data.resize(cloud_out.row_step * cloud_out.height);

    // Copy point data from PCL cloud to PointCloud2 message
    float *cloud_data_ptr = reinterpret_cast<float *>(cloud_out.data.data());

    for (const auto &point : merged_cloud.points) {
      *cloud_data_ptr++ = point.x;         // x
      *cloud_data_ptr++ = point.y;         // y
      *cloud_data_ptr++ = point.z;         // z
      *cloud_data_ptr++ = point.intensity; // intensity
    }

    RCLCPP_INFO(get_logger(),
                "Successfully created point cloud with %zu points",
                merged_cloud.size());
  }

private:
  std::size_t scan_buffer_size_;
  std::string fixed_frame_, sensor_frame_;
  rclcpp::Service<scanner_assembler::srv::AssemblerQuery>::SharedPtr service_;
  std::deque<std::pair<pcl::PointCloud<pcl::PointXYZI>, rclcpp::Time>>
      cloud_queue_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
      tf_filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::size_t total_points{0};
  std::mutex queue_mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Assembler>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}